//! 地形告警（简化 TAWS/GPWS）：前视地形碰撞预测 + 下沉率 + 过低地形。纯计算，不依赖渲染。
//!
//! 坐标系同 `lib.rs`：`+X` 东、`-Z` 北，`y` 为海拔高度（m）。地形由调用方以闭包 `height(x, z)` 提供（海面 ≤ 0）。
//!
//! 告警等级（由低到高）：
//! * `TooLowTerrain`（黄）：低于 120 m 雷达高度、不在机场附近、仍在高速飞行；
//! * `SinkRate`（黄）：雷达高度越低允许的下沉率越小（模式 1）；
//! * `Terrain`（黄）：按当前航迹/爬升率外推 40 s 内与地形净空 < 150 m；
//! * `PullUp`（红）：18 s 内净空 < 20 m（或下沉率严重超限）。

use glam::DVec3;

/// 告警类型，按严重程度升序（可直接比较大小）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum TawsAlert {
    #[default]
    None,
    TooLowTerrain,
    SinkRate,
    Terrain,
    PullUp,
}

impl TawsAlert {
    /// 0 = 无告警，1 = 警戒（黄），2 = 警告（红）。
    pub fn level(self) -> u8 {
        match self {
            TawsAlert::None => 0,
            TawsAlert::PullUp => 2,
            _ => 1,
        }
    }

    /// 驾驶舱短文本（4×6 点阵字体可显示的字符）。
    pub fn text(self) -> &'static str {
        match self {
            TawsAlert::None => "",
            TawsAlert::TooLowTerrain => "TOO LOW TERRAIN",
            TawsAlert::SinkRate => "SINK RATE",
            TawsAlert::Terrain => "TERRAIN",
            TawsAlert::PullUp => "PULL UP",
        }
    }
}

/// 告警计算输入。
#[derive(Clone, Copy, Debug)]
pub struct TawsInput {
    /// 位置（x 东、z 南）与海拔 y。
    pub pos: DVec3,
    /// 速度（世界系，m/s）。
    pub vel: DVec3,
    /// 雷达高度（机腹到正下方地面，m）。
    pub agl: f64,
    pub gear_down: bool,
    pub on_ground: bool,
    /// 到最近机场的水平距离（m）；无机场用 `f64::MAX`。
    pub airport_dist: f64,
    /// 忽略水面（两栖机水上起降）：水面不构成地形威胁。
    pub ignore_water: bool,
}

/// 告警计算结果。
#[derive(Clone, Copy, Debug, Default)]
pub struct TawsStatus {
    pub alert: TawsAlert,
    /// 前视窗口内最小预测净空（m）；无有效前视为 `None`。
    pub min_clearance: Option<f64>,
    /// 预测最小净空发生的时刻（s）。
    pub time_to_hazard: Option<f64>,
}

pub const LOOKAHEAD_S: f64 = 40.0;
pub const WARN_S: f64 = 18.0;
pub const CAUTION_CLEARANCE: f64 = 150.0;
pub const WARN_CLEARANCE: f64 = 20.0;
/// 终端区（机场附近）半径。
pub const TERMINAL_RADIUS: f64 = 6000.0;
/// 着陆构型抑制半径（放起落架且在此距离内，不做前视）。
pub const LANDING_RADIUS: f64 = 8000.0;

/// 允许的下沉率（ft/min）随雷达高度（ft）增大。
pub fn allowed_sink_fpm(agl_ft: f64) -> f64 {
    1000.0 + 1.5 * agl_ft.clamp(0.0, 2450.0)
}

fn ground_h(height: &dyn Fn(f64, f64) -> f64, x: f64, z: f64, ignore_water: bool) -> f64 {
    let h = height(x, z);
    if h <= 0.0 {
        if ignore_water {
            -1.0e4
        } else {
            0.0
        }
    } else {
        h
    }
}

/// 沿航迹方向前视采样：返回 `(t, 预测海拔, 地形最大高度)` 序列（步长 1 s）。
pub fn look_ahead(height: &dyn Fn(f64, f64) -> f64, inp: &TawsInput) -> Vec<(f64, f64, f64)> {
    let hv = DVec3::new(inp.vel.x, 0.0, inp.vel.z);
    let speed = hv.length();
    if speed < 1.0 {
        return Vec::new();
    }
    let dir = hv / speed;
    let side = DVec3::new(-dir.z, 0.0, dir.x);
    // 爬升按一半外推（保守），下降按全速外推
    let vs = if inp.vel.y > 0.0 { inp.vel.y * 0.5 } else { inp.vel.y };
    let mut out = Vec::with_capacity(LOOKAHEAD_S as usize);
    for t in 1..=(LOOKAHEAD_S as usize) {
        let t = t as f64;
        let d = speed * t;
        let c = inp.pos + dir * d;
        let w = (150.0 + 0.1 * d).min(1500.0);
        let mut hmax = f64::MIN;
        for k in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            let p = c + side * (w * k);
            hmax = hmax.max(ground_h(height, p.x, p.z, inp.ignore_water));
        }
        out.push((t, inp.pos.y + vs * t, hmax));
    }
    out
}

/// 计算当前告警。
pub fn evaluate(height: &dyn Fn(f64, f64) -> f64, inp: &TawsInput) -> TawsStatus {
    let mut st = TawsStatus::default();
    let hv = DVec3::new(inp.vel.x, 0.0, inp.vel.z);
    let speed = hv.length();
    if inp.on_ground || inp.agl < 3.0 || speed < 25.0 {
        return st;
    }
    let under = height(inp.pos.x, inp.pos.z);
    let over_water = inp.ignore_water && under <= 0.0;
    let terminal = inp.airport_dist < TERMINAL_RADIUS;
    let landing = inp.gear_down && inp.airport_dist < LANDING_RADIUS;

    // ---- 前视地形
    if !landing {
        let (caution_c, warn_c) = if terminal { (50.0, -10.0) } else { (CAUTION_CLEARANCE, WARN_CLEARANCE) };
        for (t, alt, hmax) in look_ahead(height, inp) {
            let c = alt - hmax;
            // 前方地形明显高于脚下才算“地形威胁”；平坦地形上的低空飞行由 TOO LOW TERRAIN 负责
            let rising = hmax - under.max(0.0) > 30.0;
            if st.min_clearance.map_or(true, |m| c < m) {
                st.min_clearance = Some(c);
                st.time_to_hazard = Some(t);
            }
            if t <= WARN_S && c < warn_c {
                st.alert = st.alert.max(TawsAlert::PullUp);
            } else if c < caution_c && rising {
                st.alert = st.alert.max(TawsAlert::Terrain);
            }
        }
    }

    // ---- 下沉率（模式 1）
    if !over_water {
        let sink_fpm = -inp.vel.y / crate::FT * 60.0;
        let agl_ft = inp.agl / crate::FT;
        if sink_fpm > 0.0 && agl_ft < 2450.0 && inp.agl > 10.0 {
            let mut allowed = allowed_sink_fpm(agl_ft);
            if landing {
                allowed *= 1.2;
            }
            if sink_fpm > allowed * 1.6 {
                st.alert = st.alert.max(TawsAlert::PullUp);
            } else if sink_fpm > allowed {
                st.alert = st.alert.max(TawsAlert::SinkRate);
            }
        }
        // ---- 过低地形
        if inp.agl < 120.0 && !terminal && speed > 40.0 {
            st.alert = st.alert.max(TawsAlert::TooLowTerrain);
        }
    }
    st
}

/// 告警监视器：给告警加上“最短保持时间”，避免灯/音频闪烁；严重级别升高立即生效。
#[derive(Clone, Copy, Debug, Default)]
pub struct TawsMonitor {
    pub shown: TawsAlert,
    hold: f64,
}

impl TawsMonitor {
    pub const HOLD_S: f64 = 2.5;

    pub fn update(&mut self, raw: TawsAlert, dt: f64) -> TawsAlert {
        if raw >= self.shown {
            self.shown = raw;
            self.hold = Self::HOLD_S;
        } else {
            self.hold -= dt;
            if self.hold <= 0.0 {
                self.shown = raw;
                self.hold = Self::HOLD_S;
            }
        }
        self.shown
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{FT, KT};

    /// 北面（-z）有一道 3000 m 高的山墙，墙在 z = -10 km 处，其余平坦 100 m。
    fn ridge(_x: f64, z: f64) -> f64 {
        if z < -10_000.0 {
            3000.0
        } else {
            100.0
        }
    }
    fn flat(_x: f64, _z: f64) -> f64 {
        100.0
    }

    fn inp(alt: f64, vs: f64, speed_kt: f64) -> TawsInput {
        TawsInput {
            pos: DVec3::new(0.0, alt, 0.0),
            vel: DVec3::new(0.0, vs, -speed_kt * KT), // 向北
            agl: alt - 100.0,
            gear_down: false,
            on_ground: false,
            airport_dist: f64::MAX,
            ignore_water: false,
        }
    }

    #[test]
    fn level_flight_high_over_flat_terrain_is_silent() {
        let s = evaluate(&flat, &inp(3000.0, 0.0, 200.0));
        assert_eq!(s.alert, TawsAlert::None);
    }

    #[test]
    fn terrain_ahead_gives_caution_then_pull_up() {
        // 200 kt ≈ 103 m/s：墙在 10 km → 约 97 s，远
        assert_eq!(evaluate(&ridge, &inp(1500.0, 0.0, 200.0)).alert, TawsAlert::None);
        // 靠近到 3 km（约 29 s）→ 警戒
        let mut i = inp(1500.0, 0.0, 200.0);
        i.pos.z = -7000.0;
        let s = evaluate(&ridge, &i);
        assert_eq!(s.alert, TawsAlert::Terrain, "{s:?}");
        assert!(s.time_to_hazard.unwrap() < LOOKAHEAD_S);
        // 靠近到 1.2 km（约 12 s）→ 警告
        i.pos.z = -8800.0;
        assert_eq!(evaluate(&ridge, &i).alert, TawsAlert::PullUp);
    }

    #[test]
    fn climbing_above_the_ridge_clears_the_alert() {
        let mut i = inp(3600.0, 0.0, 200.0);
        i.pos.z = -7000.0;
        assert_eq!(evaluate(&ridge, &i).alert, TawsAlert::None);
    }

    #[test]
    fn flying_parallel_to_ridge_is_silent() {
        let mut i = inp(1500.0, 0.0, 200.0);
        i.pos.z = -5000.0;
        i.vel = DVec3::new(200.0 * KT, 0.0, 0.0); // 向东
        assert_eq!(evaluate(&ridge, &i).alert, TawsAlert::None);
    }

    #[test]
    fn sink_rate_envelope_shrinks_with_altitude() {
        // 300 ft AGL 下沉 2500 fpm 超限
        let agl = 300.0 * FT;
        let mut i = inp(100.0 + agl, -2500.0 * FT / 60.0, 120.0);
        i.airport_dist = 3000.0;
        i.gear_down = true;
        assert_eq!(evaluate(&flat, &i).alert, TawsAlert::SinkRate);
        // 正常进近 700 fpm 不报
        i.vel.y = -700.0 * FT / 60.0;
        assert_eq!(evaluate(&flat, &i).alert, TawsAlert::None);
        // 严重超限 → PULL UP
        i.vel.y = -4500.0 * FT / 60.0;
        assert_eq!(evaluate(&flat, &i).alert, TawsAlert::PullUp);
    }

    #[test]
    fn normal_landing_approach_does_not_trigger_pull_up() {
        // 放轮、距机场 3 km、3° 下滑，100 ft AGL
        let mut i = inp(100.0 + 30.0, -2.0, 65.0);
        i.gear_down = true;
        i.airport_dist = 3000.0;
        assert_eq!(evaluate(&flat, &i).alert, TawsAlert::None);
    }

    #[test]
    fn too_low_terrain_only_away_from_airports() {
        let i = inp(100.0 + 60.0, 0.0, 150.0);
        assert_eq!(evaluate(&flat, &i).alert, TawsAlert::TooLowTerrain);
        let mut near = i;
        near.airport_dist = 2000.0;
        assert_eq!(evaluate(&flat, &near).alert, TawsAlert::None);
    }

    #[test]
    fn inhibited_on_ground_and_when_slow() {
        let mut i = inp(1500.0, 0.0, 200.0);
        i.pos.z = -9000.0;
        i.on_ground = true;
        assert_eq!(evaluate(&ridge, &i).alert, TawsAlert::None);
        let mut slow = inp(1500.0, 0.0, 30.0);
        slow.pos.z = -9900.0;
        assert_eq!(evaluate(&ridge, &slow).alert, TawsAlert::None);
    }

    #[test]
    fn water_ops_for_amphibians_do_not_alert() {
        let sea = |_: f64, _: f64| -5.0;
        let mut i = inp(30.0, -2.0, 90.0);
        i.agl = 30.0;
        i.pos.y = 30.0;
        assert_ne!(evaluate(&sea, &i).alert, TawsAlert::None, "ordinary aircraft: sea is terrain");
        i.ignore_water = true;
        assert_eq!(evaluate(&sea, &i).alert, TawsAlert::None);
    }

    #[test]
    fn monitor_holds_and_escalates() {
        let mut m = TawsMonitor::default();
        assert_eq!(m.update(TawsAlert::Terrain, 0.1), TawsAlert::Terrain);
        assert_eq!(m.update(TawsAlert::None, 1.0), TawsAlert::Terrain, "held");
        assert_eq!(m.update(TawsAlert::PullUp, 0.1), TawsAlert::PullUp, "escalate immediately");
        assert_eq!(m.update(TawsAlert::None, 2.0), TawsAlert::PullUp);
        assert_eq!(m.update(TawsAlert::None, 1.0), TawsAlert::None);
    }

    #[test]
    fn alert_ordering_and_levels() {
        assert!(TawsAlert::PullUp > TawsAlert::Terrain && TawsAlert::Terrain > TawsAlert::SinkRate);
        assert_eq!(TawsAlert::PullUp.level(), 2);
        assert_eq!(TawsAlert::SinkRate.level(), 1);
        assert_eq!(TawsAlert::None.level(), 0);
    }
}
