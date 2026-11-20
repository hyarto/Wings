//! 多气动面（简化叶素）升阻力、失速与地面效应。
//!
//! 每个气动面独立计算：
//! 1. 局部相对气流 `v_rel = v_cg + ω×r − v_wind`（含滑流、风/阵风）；
//! 2. 局部迎角 α 与动压 q；
//! 3. 全迎角 [-180°, +180°] 升阻力：线性区 → 临界迎角失速跌落 → 后失速平板流态；
//! 4. 舵面偏转以等效迎角 Δα 与阻力增量表达；
//! 5. 地面效应：AGL < 翼展时按 Wieselsberger 近似降低诱导阻力并略提升升力斜率；
//! 6. 结冰：升力系数最高 -40%、失速迎角最多提前 5°、寄生阻力最高 +150%。

use crate::aircraft::{AircraftConfig, SurfaceConfig, SurfaceKind};
use crate::state::{RigidState, SurfacePositions};
use crate::structure::DamageState;
use crate::smoothstep;
use glam::DVec3;
use std::f64::consts::PI;

pub const MAX_SURFACES: usize = 12;

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceDiag {
    pub alpha: f64,
    pub cl: f64,
    pub cd: f64,
    /// 失速程度 0（附着）..1（完全分离）。
    pub sigma: f64,
    pub q: f64,
}

#[derive(Clone, Debug)]
pub struct AeroOutput {
    /// 机体系合力（N）。
    pub force: DVec3,
    /// 机体系绕 CG 合力矩（N·m）。
    pub moment: DVec3,
    pub surfaces: [SurfaceDiag; MAX_SURFACES],
}

impl Default for AeroOutput {
    fn default() -> Self {
        Self { force: DVec3::ZERO, moment: DVec3::ZERO, surfaces: [SurfaceDiag::default(); MAX_SURFACES] }
    }
}

pub struct AeroInputs<'a> {
    pub rho: f64,
    /// 每个气动面处的世界系风速（与 `cfg.surfaces` 同序）。
    pub wind: &'a [DVec3],
    pub ice: f64,
    /// 机翼离地高度（m）。
    pub agl: f64,
    pub surfaces: SurfacePositions,
    /// 配平 -1..1。
    pub trim: f64,
    pub damage: &'a DamageState,
    /// 螺旋桨滑流附加速度（m/s）。
    pub slipstream: f64,
    /// 当地声速（m/s），用于马赫阻力 / 抖振。
    pub sound_speed: f64,
    /// 起落架收放位置 0..1（附加寄生阻力）。
    pub gear_pos: f64,
}

/// 激波阻力系数增量（Δ = M - M_crit，三次方陡升）。
pub fn wave_drag(mach: f64, mach_crit: f64) -> f64 {
    let d = (mach - mach_crit).max(0.0);
    100.0 * d * d * d
}

/// 有效临界马赫数：升力系数越大（高空低速大迎角），抖振起始马赫越低。
pub fn effective_mach_crit(mach_crit: f64, cl: f64) -> f64 {
    mach_crit - 0.10 * (cl - 0.3).max(0.0)
}

/// 马赫抖振：高马赫下临界迎角提前（rad）。与低速失速共同构成“棺材角”。
pub fn mach_stall_shift(mach: f64, mach_crit: f64) -> f64 {
    0.9 * (mach - (mach_crit + 0.02)).max(0.0)
}

#[derive(Clone, Copy, Debug)]
pub struct Section {
    pub cl: f64,
    pub cd: f64,
    pub sigma: f64,
    /// 附着流线性升力系数（供下洗估算）。
    pub cl_lin: f64,
}

/// 单个升力面的截面系数（全迎角）。`delta` 为舵面等效 Δα（rad）。
pub fn section(
    s: &SurfaceConfig,
    alpha: f64,
    delta: f64,
    ice: f64,
    induced_mult: f64,
    slope_boost: f64,
    flap: f64,
    spoiler: f64,
) -> Section {
    section_m(s, alpha, delta, ice, induced_mult, slope_boost, flap, spoiler, 0.0, 0.0)
}

/// 同 `section`，附带马赫效应：`mach_shift`（失速角提前，rad）与 `wave_cd`（激波阻力）。
#[allow(clippy::too_many_arguments)]
pub fn section_m(
    s: &SurfaceConfig,
    alpha: f64,
    delta: f64,
    ice: f64,
    induced_mult: f64,
    slope_boost: f64,
    flap: f64,
    spoiler: f64,
    mach_shift: f64,
    wave_cd: f64,
) -> Section {
    let ice = (ice * s.ice_sensitivity).clamp(0.0, 1.0);
    let dstall = 5f64.to_radians() * ice;
    // 襟翼增升以 flap_stall_frac 提前失速，其余舵面偏转按 0.5
    let d_flap = flap * s.gains.flap;
    let stall_shift = 0.5 * (delta - d_flap) + s.flap_stall_frac * d_flap;
    let a_pos = (s.alpha_stall_pos - stall_shift - dstall - mach_shift).max(0.05);
    let a_neg = (s.alpha_stall_neg - stall_shift + dstall + mach_shift).min(-0.05);

    // 结冰使 CLmax 最多下降 40%：综合“失速角提前”与“斜率缩放”两个效应
    let f_ice = 1.0 - 0.40 * ice;
    let base_span = (s.alpha_stall_pos - s.alpha0).max(0.05);
    let k_ice = f_ice * base_span / (base_span - dstall - mach_shift).max(0.05);

    let a_lin = alpha.clamp(a_neg, a_pos);
    let cl_lin = k_ice * s.cl_alpha * (1.0 + slope_boost) * (a_lin + delta - s.alpha0);

    // 后失速平板流态
    let (sa, ca) = alpha.sin_cos();
    let cl_fp = 1.05 * 2.0 * sa * ca; // = 1.05·sin2α
    let cd_fp = 1.55 * sa * sa;

    let over = if alpha >= 0.0 { (alpha - a_pos) / s.stall_width } else { (a_neg - alpha) / s.stall_width };
    let sigma = smoothstep(0.0, 1.0, over);

    let cl = (1.0 - sigma) * cl_lin + sigma * cl_fp;
    let cdi = cl_lin * cl_lin / (PI * s.oswald * s.aspect_ratio) * induced_mult;
    let cd0 = s.cd0 * (1.0 + 1.5 * ice);
    let cd_extra = s.flap_cd * flap * flap + s.spoiler_cd * spoiler;
    let cd = cd0 + cd_extra + wave_cd + (1.0 - sigma) * cdi + sigma * cd_fp;
    Section { cl, cd, sigma, cl_lin }
}

fn wrap_pi(a: f64) -> f64 {
    let mut a = a;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// 计算全机气动力/力矩（机体系）。
pub fn compute(cfg: &AircraftConfig, rs: &RigidState, inp: &AeroInputs, out: &mut AeroOutput) {
    out.force = DVec3::ZERO;
    out.moment = DVec3::ZERO;
    let qi = rs.q.inverse();
    let ctl = inp.surfaces;
    let elevator = (ctl.elevator + 0.4 * inp.trim).clamp(-1.0, 1.0);
    let flaps = inp.damage.flaps_jammed.unwrap_or(ctl.flaps);

    // 地面效应
    let h = inp.agl.max(0.3);
    let x = 16.0 * h / cfg.wing_span;
    let phi = x * x / (1.0 + x * x);
    let slope_boost = 0.15 * (1.0 - phi);

    let mut downwash_angle = 0.0;
    let mut wing_lift_sum = 0.0;

    for pass in 0..2 {
        for (i, s) in cfg.surfaces.iter().enumerate() {
            if (pass == 0) == (s.downwash > 0.0) {
                continue;
            }
            if (s.kind.is_left_wing() && inp.damage.left_wing_failed)
                || (s.kind.is_right_wing() && inp.damage.right_wing_failed)
            {
                continue;
            }
            let wind = inp.wind.get(i).copied().unwrap_or(DVec3::ZERO);
            let mut v = qi * (rs.vel - wind) + rs.w.cross(s.pos);

            if s.kind == SurfaceKind::Fuselage {
                let speed = v.length();
                let mach = speed / inp.sound_speed.max(1.0);
                let wave = 1.0 + 40.0 * (mach - cfg.mach_crit).max(0.0).powi(2);
                let gear_cd = cfg.gear_cda * inp.gear_pos;
                let f = DVec3::new(
                    -0.5 * inp.rho * speed * (s.body_cda.x + 4.0 * gear_cd) * v.x,
                    -0.5 * inp.rho * speed * (s.body_cda.y + 4.0 * gear_cd) * v.y,
                    -0.5 * inp.rho * speed * (s.body_cda.z * wave + gear_cd) * v.z,
                ) * (1.0 + 1.5 * inp.ice);
                out.force += f;
                out.moment += s.pos.cross(f);
                continue;
            }

            // 螺旋桨滑流：附加沿机头方向的气流
            v += crate::FWD * (inp.slipstream * s.slipstream);

            let c = crate::FWD;
            let n = s.normal;
            let span = c.cross(n);
            let vc = v.dot(c);
            let vn = v.dot(n);
            let v2 = c * vc + n * vn;
            let speed = v2.length();
            if speed < 0.5 {
                continue;
            }
            let mut alpha = (-vn).atan2(vc) + s.incidence;
            if pass == 1 {
                alpha -= downwash_angle * s.downwash;
            }
            alpha = wrap_pi(alpha);

            let delta = elevator * s.gains.elevator
                + ctl.aileron * s.gains.aileron
                + ctl.rudder * s.gains.rudder
                + flaps * s.gains.flap
                + ctl.spoiler * s.gains.spoiler;

            let (induced_mult, boost) = if s.kind.is_wing() { (phi, slope_boost) } else { (1.0, 0.0) };
            let mach = speed / inp.sound_speed.max(1.0);
            let mcrit = if s.kind.is_wing() {
                effective_mach_crit(cfg.mach_crit, (s.cl_alpha * (alpha + delta - s.alpha0)).max(0.0))
            } else {
                cfg.mach_crit
            };
            let (m_shift, m_wave) = if mach > mcrit - 0.1 {
                (mach_stall_shift(mach, mcrit), wave_drag(mach, mcrit))
            } else {
                (0.0, 0.0)
            };
            let sec = section_m(s, alpha, delta, inp.ice, induced_mult, boost, flaps, ctl.spoiler, m_shift, m_wave);

            let q = 0.5 * inp.rho * speed * speed;
            let d_hat = -v2 / speed;
            let l_hat = d_hat.cross(span);
            let f = (l_hat * sec.cl + d_hat * sec.cd) * (q * s.area);
            // 翼型俯仰力矩（抬头为正，绕 span 轴；span 对水平翼为 +X）
            let cm = s.cm0 + s.cm_flap * flaps;
            let m_cm = span * (q * s.area * s.chord * cm);

            out.force += f;
            out.moment += s.pos.cross(f) + m_cm;
            if i < MAX_SURFACES {
                out.surfaces[i] = SurfaceDiag { alpha, cl: sec.cl, cd: sec.cd, sigma: sec.sigma, q };
            }
            if pass == 0 && s.kind.is_wing() {
                wing_lift_sum += sec.cl_lin * s.area;
            }
        }
        if pass == 0 {
            let cl_avg = wing_lift_sum / cfg.wing_area.max(1e-6);
            let ar = cfg.wing_span * cfg.wing_span / cfg.wing_area.max(1e-6);
            downwash_angle = 2.0 * cl_avg / (PI * ar);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::sky172;

    #[test]
    fn lift_curve_has_peak_then_drops() {
        let cfg = sky172();
        let wing = &cfg.surfaces[1];
        let mut peak = (0.0, 0.0);
        let mut prev = 0.0;
        let mut dropped = false;
        for i in 0..400 {
            let a = (i as f64 * 0.1).to_radians();
            let sec = section(wing, a, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
            if sec.cl > peak.0 {
                peak = (sec.cl, a.to_degrees());
            }
            if sec.cl < prev - 0.3 {
                dropped = true;
            }
            prev = sec.cl.max(prev);
        }
        assert!(peak.1 > 14.0 && peak.1 < 19.0, "peak alpha {}", peak.1);
        assert!(peak.0 > 1.2 && peak.0 < 1.7, "clmax {}", peak.0);
        assert!(dropped);
    }

    #[test]
    fn ice_reduces_clmax_by_40_percent() {
        let cfg = sky172();
        let wing = &cfg.surfaces[1];
        let clmax = |ice: f64| {
            (0..200)
                .map(|i| section(wing, (i as f64 * 0.1).to_radians(), 0.0, ice, 1.0, 0.0, 0.0, 0.0).cl)
                .fold(0.0, f64::max)
        };
        let ratio = clmax(1.0) / clmax(0.0);
        assert!((ratio - 0.6).abs() < 0.06, "ratio {ratio}");
    }

    #[test]
    fn symmetric_for_negative_alpha_and_flat_plate_beyond_stall() {
        let cfg = sky172();
        let wing = &cfg.surfaces[1];
        let s = section(wing, 90f64.to_radians(), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        assert!(s.cl.abs() < 0.05);
        assert!(s.cd > 1.3);
    }
}
