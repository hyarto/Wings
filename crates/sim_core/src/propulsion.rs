//! 引擎模型：活塞螺旋桨 / 涡桨 / 涡扇。
//!
//! 推力随空气密度衰减，是“自然升限”的主要来源之一（严禁硬编码最大高度）。

use crate::aircraft::{EngineConfig, EngineKind};
use crate::atmosphere::{AtmosphereState, RHO0};
use crate::state::EngineState;
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineOutput {
    /// 沿机头方向推力（N，反推/反桨为负）。
    pub thrust: f64,
    pub power_w: f64,
    pub rpm: f64,
    pub fuel_flow: f64,
    /// 螺旋桨反扭矩（N·m，正值 = 使机体向左滚转）。
    pub torque: f64,
    /// 螺旋桨滑流附加速度（m/s）。
    pub slipstream: f64,
}

/// p-范数平滑取小。
fn smooth_min(a: f64, b: f64) -> f64 {
    if a <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let k = 5.0;
    (a.powf(-k) + b.powf(-k)).powf(-1.0 / k)
}

/// 螺旋桨推力：低速由静推力主导，高速由 η·P/V 主导。
fn prop_thrust(p: f64, p_rated: f64, v: f64, sigma: f64, pitch_speed: f64, static_thrust: f64) -> f64 {
    if p <= 1.0 {
        return 0.0;
    }
    let t1 = static_thrust
        * (p / p_rated).powf(2.0 / 3.0)
        * sigma.powf(1.0 / 3.0)
        * (1.0 - (v / pitch_speed).powi(2)).max(0.0);
    let eta = 0.80;
    let t2 = eta * p / v.max(0.5);
    smooth_min(t1.max(0.0), t2)
}

/// 推进一台引擎，返回其输出。`v_axial` 为沿推力轴的空速（m/s）。
pub fn update(
    cfg: &EngineConfig,
    st: &mut EngineState,
    throttle_cmd: f64,
    reverse: bool,
    v_axial: f64,
    atm: &AtmosphereState,
    fuel_available: bool,
    dt: f64,
) -> EngineOutput {
    let sigma = atm.density / RHO0;
    let v = v_axial.max(0.0);
    let running = st.running && !st.failed && fuel_available;
    let out: EngineOutput;

    // 油门伺服 / 涡扇慢响应
    let tau = match cfg.kind {
        EngineKind::Piston { .. } => 0.15,
        EngineKind::Turboprop { .. } => 1.2,
        EngineKind::Turbofan { .. } => 2.2,
    };
    let target = if running { throttle_cmd.clamp(0.0, 1.0) } else { 0.0 };
    st.throttle += (target - st.throttle) * (1.0 - (-dt / tau).exp());

    let disc_area = PI * cfg.prop_diameter * cfg.prop_diameter / 4.0;

    match cfg.kind {
        EngineKind::Piston { power_w, bsfc, max_rpm, idle_rpm, prop_pitch_speed, static_thrust } => {
            // Gagg-Farrar 型功率随密度衰减
            let p_alt = ((sigma - 0.12) / 0.88).clamp(0.0, 1.2);
            let thr_eff = 0.06 + 0.94 * st.throttle;
            let p = if running { power_w * thr_eff * p_alt } else { 0.0 };
            let mut t = prop_thrust(p, power_w, v, sigma, prop_pitch_speed, static_thrust);
            // 风车/怠速螺旋桨阻力
            let windmill = if running { (1.0 - st.throttle) * 0.5 } else { 1.0 };
            t -= windmill * 0.5 * atm.density * v * v * 0.05;
            let rpm = if running {
                (idle_rpm + (max_rpm - idle_rpm) * (thr_eff * p_alt).clamp(0.0, 1.0).powf(0.45) + 6.0 * v)
                    .min(max_rpm * 1.08)
            } else {
                (v * 14.0).min(max_rpm * 0.5)
            };
            let omega = (rpm * 2.0 * PI / 60.0).max(1.0);
            out = EngineOutput {
                thrust: t,
                power_w: p,
                rpm,
                fuel_flow: if running { (p * bsfc).max(power_w * 0.04 * bsfc) } else { 0.0 },
                torque: p / omega,
                slipstream: slipstream(t.max(0.0), v, atm.density, disc_area),
            };
        }
        EngineKind::Turboprop { power_w, bsfc, max_rpm, prop_pitch_speed, static_thrust, reverse_ratio } => {
            let p_alt = sigma.powf(0.8);
            let thr_eff = 0.08 + 0.92 * st.throttle;
            let p = if running { power_w * thr_eff * p_alt } else { 0.0 };
            let mut t = prop_thrust(p, power_w, v, sigma, prop_pitch_speed, static_thrust);
            if reverse && running {
                t = -reverse_ratio * static_thrust * (p / power_w).powf(2.0 / 3.0) * (1.0 - (v / 80.0).powi(2)).max(0.2);
            }
            if !running {
                // 失效螺旋桨：风车阻力（顺桨后大幅降低）
                let cd_a = if st.feathered { 0.012 } else { 0.16 } * disc_area;
                t = -0.5 * atm.density * v * v * cd_a;
            }
            let rpm = if running { max_rpm * (0.55 + 0.45 * thr_eff.powf(0.5)) } else if st.feathered { 0.0 } else { (v * 9.0).min(max_rpm * 0.4) };
            let omega = (rpm * 2.0 * PI / 60.0).max(1.0);
            out = EngineOutput {
                thrust: t,
                power_w: p,
                rpm,
                fuel_flow: if running { (p * bsfc).max(power_w * 0.10 * bsfc) } else { 0.0 },
                torque: p / omega,
                slipstream: slipstream(t.max(0.0), v, atm.density, disc_area),
            };
        }
        EngineKind::Turbofan { thrust_n, tsfc, bypass_decay, reverse_ratio } => {
            let mach = v / atm.speed_of_sound;
            let thr_eff = 0.05 + 0.95 * st.throttle;
            let lapse = sigma.powf(0.85) * (1.0 - bypass_decay * mach).max(0.2);
            let mut t = if running { thrust_n * thr_eff.powf(1.15) * lapse } else { 0.0 };
            if reverse && running && v > 8.0 {
                t = -reverse_ratio * thrust_n * thr_eff.powf(1.15) * lapse;
            }
            if !running {
                // 风车风扇阻力
                t = -0.5 * atm.density * v * v * 0.25;
            }
            let n1 = if running { 100.0 * (0.25 + 0.75 * thr_eff.powf(0.6)) } else { (v * 0.12).min(25.0) };
            out = EngineOutput {
                thrust: t,
                power_w: t.abs() * v,
                rpm: n1,
                fuel_flow: if running { t.abs() * tsfc + thrust_n * 0.02 * tsfc } else { 0.0 },
                torque: 0.0,
                slipstream: 0.0,
            };
        }
    }

    st.rpm = out.rpm;
    st.power_w = out.power_w;
    st.thrust_n = out.thrust;
    st.fuel_flow = out.fuel_flow;
    out
}

fn slipstream(thrust: f64, v: f64, rho: f64, disc_area: f64) -> f64 {
    if thrust <= 0.0 || disc_area <= 0.0 {
        return 0.0;
    }
    ((v * v + 2.0 * thrust / (rho * disc_area)).sqrt() - v).clamp(0.0, 35.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::sky172;
    use crate::atmosphere::Atmosphere;

    fn run(alt: f64, v: f64) -> EngineOutput {
        let cfg = sky172().engines[0];
        let mut st = EngineState::default();
        let atm = Atmosphere::default().sample(alt);
        let mut out = EngineOutput::default();
        for _ in 0..200 {
            out = update(&cfg, &mut st, 1.0, false, v, &atm, true, 1.0 / 120.0);
        }
        out
    }

    #[test]
    fn thrust_decays_with_altitude_and_speed() {
        let sl = run(0.0, 0.0);
        assert!(sl.thrust > 2000.0 && sl.thrust < 2400.0, "{}", sl.thrust);
        let cruise = run(0.0, 60.0);
        assert!(cruise.thrust < sl.thrust);
        assert!(cruise.thrust > 700.0, "{}", cruise.thrust);
        assert!(run(4000.0, 40.0).thrust < run(0.0, 40.0).thrust * 0.85);
    }

    #[test]
    fn turbofan_reverse_thrust_is_negative() {
        let cfg = EngineConfig {
            kind: EngineKind::Turbofan { thrust_n: 60_000.0, tsfc: 1.6e-5, bypass_decay: 0.4, reverse_ratio: 0.4 },
            pos: glam::DVec3::ZERO,
            spin: 0.0,
            prop_diameter: 0.0,
        };
        let mut st = EngineState::default();
        let atm = Atmosphere::default().sample(0.0);
        let mut o = EngineOutput::default();
        for _ in 0..1200 {
            o = update(&cfg, &mut st, 1.0, true, 60.0, &atm, true, 1.0 / 120.0);
        }
        assert!(o.thrust < 0.0);
    }
}
