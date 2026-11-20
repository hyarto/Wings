//! 黑匣子：固定频率记录飞行数据，支持 CSV 导出/导入与按时间插值回放（M4）。
//!
//! 纯数据模块，不依赖渲染层；客户端用它做“飞行回放”，测试用它做事后分析。

use crate::integrator::Simulation;
use crate::KT;
use std::collections::VecDeque;

/// 单个记录帧。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub pos: [f64; 3],
    /// 姿态四元数 (x, y, z, w)，机体 → 世界。
    pub q: [f64; 4],
    pub ias_kt: f64,
    pub alt_m: f64,
    pub vs_mps: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub nz: f64,
    pub aoa_deg: f64,
    pub throttle: f64,
    pub fuel_kg: f64,
    pub ice: f64,
    pub flags: u32,
}

pub const FLAG_ON_GROUND: u32 = 1;
pub const FLAG_STALL: u32 = 2;
pub const FLAG_OVERSPEED: u32 = 4;
pub const FLAG_G_LIMIT: u32 = 8;
pub const FLAG_DAMAGE: u32 = 16;
pub const FLAG_CRASHED: u32 = 32;
pub const FLAG_GEAR_DOWN: u32 = 64;

const HEADER: &str = "t,x,y,z,qx,qy,qz,qw,ias_kt,alt_m,vs_mps,heading,pitch,roll,nz,aoa_deg,throttle,fuel_kg,ice,flags";

#[derive(Clone, Debug)]
pub struct BlackBox {
    /// 记录间隔（s）。
    pub interval: f64,
    /// 最大帧数（环形缓冲，旧数据被丢弃）。
    pub capacity: usize,
    pub samples: VecDeque<Sample>,
    next_t: f64,
}

/// 飞行摘要。
#[derive(Clone, Copy, Debug, Default)]
pub struct Summary {
    pub duration: f64,
    pub max_alt_m: f64,
    pub max_ias_kt: f64,
    pub max_nz: f64,
    pub min_nz: f64,
    pub max_vs_down: f64,
    pub landed: bool,
    pub damaged: bool,
}

impl Default for BlackBox {
    fn default() -> Self {
        Self::new(0.1, 36_000)
    }
}

impl BlackBox {
    pub fn new(interval: f64, capacity: usize) -> Self {
        Self { interval: interval.max(1e-3), capacity: capacity.max(2), samples: VecDeque::new(), next_t: 0.0 }
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
    pub fn clear(&mut self) {
        self.samples.clear();
        self.next_t = 0.0;
    }

    /// 从当前仿真状态构造一帧。
    pub fn snapshot(sim: &Simulation) -> Sample {
        let r = &sim.state.rigid;
        let d = &sim.diag;
        let w = &d.warnings;
        let dmg = &sim.state.damage;
        let mut flags = 0;
        if sim.state.on_ground {
            flags |= FLAG_ON_GROUND;
        }
        if d.stall_warning {
            flags |= FLAG_STALL;
        }
        if w.overspeed {
            flags |= FLAG_OVERSPEED;
        }
        if w.g_limit {
            flags |= FLAG_G_LIMIT;
        }
        if dmg.any() {
            flags |= FLAG_DAMAGE;
        }
        if dmg.crashed {
            flags |= FLAG_CRASHED;
        }
        if sim.state.gear_pos > 0.5 {
            flags |= FLAG_GEAR_DOWN;
        }
        Sample {
            t: sim.state.time,
            pos: [r.pos.x, r.pos.y, r.pos.z],
            q: [r.q.x, r.q.y, r.q.z, r.q.w],
            ias_kt: d.ias / KT,
            alt_m: r.pos.y,
            vs_mps: d.vertical_speed,
            heading: d.heading,
            pitch: d.pitch,
            roll: d.roll,
            nz: d.nz,
            aoa_deg: d.aoa.to_degrees(),
            throttle: sim.controls.throttle,
            fuel_kg: sim.state.fuel_kg,
            ice: sim.state.ice_load,
            flags,
        }
    }

    /// 每个物理步调用一次；按 `interval` 抽样记录。
    pub fn record(&mut self, sim: &Simulation) {
        let t = sim.state.time;
        if t + 1e-9 < self.next_t {
            return;
        }
        self.next_t = t + self.interval;
        if self.samples.len() >= self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(Self::snapshot(sim));
    }

    pub fn duration(&self) -> f64 {
        match (self.samples.front(), self.samples.back()) {
            (Some(a), Some(b)) => b.t - a.t,
            _ => 0.0,
        }
    }

    pub fn start_time(&self) -> f64 {
        self.samples.front().map_or(0.0, |s| s.t)
    }

    /// 回放：按绝对时间插值（位置线性、姿态 slerp，其余线性）。超出范围时钳制。
    pub fn at(&self, t: f64) -> Option<Sample> {
        let first = self.samples.front()?;
        let last = self.samples.back()?;
        if t <= first.t {
            return Some(*first);
        }
        if t >= last.t {
            return Some(*last);
        }
        let idx = self.samples.partition_point(|s| s.t <= t);
        let a = &self.samples[idx - 1];
        let b = &self.samples[idx];
        let f = ((t - a.t) / (b.t - a.t).max(1e-12)).clamp(0.0, 1.0);
        let l = |x: f64, y: f64| x + (y - x) * f;
        let qa = glam::DQuat::from_xyzw(a.q[0], a.q[1], a.q[2], a.q[3]);
        let qb = glam::DQuat::from_xyzw(b.q[0], b.q[1], b.q[2], b.q[3]);
        let q = qa.slerp(qb, f);
        // 航向需处理 0/360 回绕
        let mut dh = b.heading - a.heading;
        if dh > 180.0 {
            dh -= 360.0;
        } else if dh < -180.0 {
            dh += 360.0;
        }
        Some(Sample {
            t,
            pos: [l(a.pos[0], b.pos[0]), l(a.pos[1], b.pos[1]), l(a.pos[2], b.pos[2])],
            q: [q.x, q.y, q.z, q.w],
            ias_kt: l(a.ias_kt, b.ias_kt),
            alt_m: l(a.alt_m, b.alt_m),
            vs_mps: l(a.vs_mps, b.vs_mps),
            heading: (a.heading + dh * f).rem_euclid(360.0),
            pitch: l(a.pitch, b.pitch),
            roll: l(a.roll, b.roll),
            nz: l(a.nz, b.nz),
            aoa_deg: l(a.aoa_deg, b.aoa_deg),
            throttle: l(a.throttle, b.throttle),
            fuel_kg: l(a.fuel_kg, b.fuel_kg),
            ice: l(a.ice, b.ice),
            flags: a.flags | b.flags,
        })
    }

    pub fn summary(&self) -> Summary {
        let mut s = Summary { duration: self.duration(), min_nz: f64::MAX, ..Default::default() };
        if self.samples.is_empty() {
            s.min_nz = 0.0;
            return s;
        }
        let mut was_air = false;
        for p in &self.samples {
            s.max_alt_m = s.max_alt_m.max(p.alt_m);
            s.max_ias_kt = s.max_ias_kt.max(p.ias_kt);
            s.max_nz = s.max_nz.max(p.nz);
            s.min_nz = s.min_nz.min(p.nz);
            s.max_vs_down = s.max_vs_down.max(-p.vs_mps);
            let air = p.flags & FLAG_ON_GROUND == 0;
            if was_air && !air {
                s.landed = true;
            }
            was_air |= air;
            if p.flags & FLAG_DAMAGE != 0 {
                s.damaged = true;
            }
        }
        s
    }

    pub fn to_csv(&self) -> String {
        use std::fmt::Write;
        let mut out = String::with_capacity(self.samples.len() * 160 + 128);
        out.push_str(HEADER);
        out.push('\n');
        for s in &self.samples {
            let _ = writeln!(
                out,
                "{:.4},{:.4},{:.4},{:.4},{:.8},{:.8},{:.8},{:.8},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.3},{:.3},{:.3},{:.4},{}",
                s.t, s.pos[0], s.pos[1], s.pos[2], s.q[0], s.q[1], s.q[2], s.q[3], s.ias_kt, s.alt_m, s.vs_mps,
                s.heading, s.pitch, s.roll, s.nz, s.aoa_deg, s.throttle, s.fuel_kg, s.ice, s.flags
            );
        }
        out
    }

    pub fn from_csv(text: &str) -> Result<Self, String> {
        let mut bb = BlackBox::default();
        let mut lines = text.lines();
        match lines.next() {
            Some(h) if h.trim() == HEADER => {}
            _ => return Err("bad blackbox header".into()),
        }
        for (n, line) in lines.enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let f: Vec<&str> = line.split(',').collect();
            if f.len() != 20 {
                return Err(format!("line {}: expected 20 fields, got {}", n + 2, f.len()));
            }
            let p = |i: usize| -> Result<f64, String> {
                f[i].trim().parse::<f64>().map_err(|e| format!("line {}: field {}: {e}", n + 2, i))
            };
            bb.samples.push_back(Sample {
                t: p(0)?,
                pos: [p(1)?, p(2)?, p(3)?],
                q: [p(4)?, p(5)?, p(6)?, p(7)?],
                ias_kt: p(8)?,
                alt_m: p(9)?,
                vs_mps: p(10)?,
                heading: p(11)?,
                pitch: p(12)?,
                roll: p(13)?,
                nz: p(14)?,
                aoa_deg: p(15)?,
                throttle: p(16)?,
                fuel_kg: p(17)?,
                ice: p(18)?,
                flags: p(19)? as u32,
            });
        }
        if bb.samples.len() >= 2 {
            bb.interval = bb.samples[1].t - bb.samples[0].t;
        }
        Ok(bb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::sky172;
    use crate::gear::FlatGround;

    fn flown() -> (Simulation, BlackBox) {
        let mut sim = Simulation::new(sky172(), Box::new(FlatGround::default()), (0.0, 0.0), 350.0);
        sim.set_airborne(800.0, 80.0 * KT, 350.0);
        sim.controls.throttle = 0.7;
        let mut bb = BlackBox::new(0.1, 1000);
        for _ in 0..(120 * 10) {
            sim.step();
            bb.record(&sim);
        }
        (sim, bb)
    }

    #[test]
    fn records_at_interval_and_respects_capacity() {
        let (_, bb) = flown();
        assert!((98..=102).contains(&bb.len()), "len {}", bb.len());
        assert!((bb.duration() - 9.9).abs() < 0.3);
        let mut small = BlackBox::new(0.1, 10);
        let mut sim = Simulation::new(sky172(), Box::new(FlatGround::default()), (0.0, 0.0), 0.0);
        for _ in 0..600 {
            sim.step();
            small.record(&sim);
        }
        assert_eq!(small.len(), 10);
    }

    #[test]
    fn csv_roundtrip() {
        let (_, bb) = flown();
        let csv = bb.to_csv();
        let back = BlackBox::from_csv(&csv).unwrap();
        assert_eq!(back.len(), bb.len());
        let (a, b) = (bb.samples[50], back.samples[50]);
        assert!((a.alt_m - b.alt_m).abs() < 1e-2 && (a.ias_kt - b.ias_kt).abs() < 1e-2 && a.flags == b.flags);
        assert!(BlackBox::from_csv("garbage").is_err());
    }

    #[test]
    fn replay_interpolates_and_wraps_heading() {
        let (_, bb) = flown();
        let t0 = bb.samples[10].t;
        let mid = bb.at(t0 + 0.05).unwrap();
        let (a, b) = (bb.samples[10], bb.samples[11]);
        assert!(mid.alt_m >= a.alt_m.min(b.alt_m) - 1e-9 && mid.alt_m <= a.alt_m.max(b.alt_m) + 1e-9);
        assert!(bb.at(-5.0).unwrap().t == bb.samples[0].t);
        // 回绕：359° → 1° 中点应为 0°
        let mut w = BlackBox::new(1.0, 10);
        let mut s0 = bb.samples[0];
        s0.heading = 359.0;
        s0.t = 0.0;
        let mut s1 = s0;
        s1.heading = 1.0;
        s1.t = 1.0;
        w.samples.push_back(s0);
        w.samples.push_back(s1);
        let h = w.at(0.5).unwrap().heading;
        assert!(h < 0.01 || h > 359.99, "heading {h}");
    }

    #[test]
    fn summary_reports_extremes() {
        let (_, bb) = flown();
        let s = bb.summary();
        assert!(s.max_alt_m > 700.0 && s.max_ias_kt > 60.0 && !s.damaged);
    }
}
