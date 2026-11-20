//! M4 机型：TWIN-6 / JET-39 / LINER-73 / SEA-415（数据驱动，共用 `Layout` 构造器）。
//!
//! 参数取自对应原型机的公开数据（质量、翼面积、推力/功率、限制载荷、速度限制），
//! 气动/配平系数经无头试飞（`tests/m4_fleet.rs` + `examples/perf.rs`）整定。

use crate::aircraft::*;
use crate::structure::StructureLimits;
use crate::{FT, G, KT};
use glam::DVec3;

fn d(x: f64) -> f64 {
    x.to_radians()
}

/// 常规布局（上/下单翼 + 平尾 + 垂尾 + 机身）的气动参数。
struct Layout {
    // 机翼
    span: f64,
    area: f64,
    chord: f64,
    wing_y: f64,
    wing_z: f64,
    dihedral_deg: f64,
    cl_alpha: f64,
    alpha0_deg: f64,
    stall_in_deg: f64,
    stall_out_deg: f64,
    cd0: f64,
    oswald: f64,
    cm0: f64,
    /// 内翼（襟翼段）面积占比。
    inner_share: f64,
    inner_x_frac: f64,
    outer_x_frac: f64,
    aileron: f64,
    flap_gain: f64,
    flap_cd: f64,
    flap_cm: f64,
    spoiler_gain: f64,
    spoiler_cd: f64,
    slip_inner: f64,
    // 平尾
    h_area: f64,
    h_y: f64,
    h_z: f64,
    h_cl_alpha: f64,
    h_incidence_deg: f64,
    h_ar: f64,
    elevator: f64,
    h_slip: f64,
    h_downwash: f64,
    // 垂尾
    v_area: f64,
    v_y: f64,
    v_z: f64,
    v_ar: f64,
    rudder: f64,
    v_slip: f64,
    // 机身 (侧向, 垂向, 轴向) Cd·A
    body_cda: DVec3,
}

fn airframe(l: &Layout) -> Vec<SurfaceConfig> {
    let dihedral = d(l.dihedral_deg);
    let ar = l.span * l.span / l.area;
    let wing = |kind: SurfaceKind, side: f64, inner: bool| {
        let x = l.span * if inner { l.inner_x_frac } else { l.outer_x_frac };
        let share = if inner { l.inner_share } else { 1.0 - l.inner_share };
        let mut s = SurfaceConfig::blank(kind, DVec3::new(side * x, l.wing_y, l.wing_z));
        s.area = l.area * share / 2.0;
        s.chord = l.chord * if inner { 1.15 } else { 0.85 };
        s.normal = DVec3::new(-side * dihedral.sin(), dihedral.cos(), 0.0);
        s.cl_alpha = l.cl_alpha;
        s.alpha0 = d(l.alpha0_deg);
        s.alpha_stall_pos = d(if inner { l.stall_in_deg } else { l.stall_out_deg });
        s.alpha_stall_neg = d(-14.0);
        s.stall_width = d(5.0);
        s.cd0 = l.cd0;
        s.aspect_ratio = ar;
        s.oswald = l.oswald;
        s.cm0 = l.cm0;
        s.cm_flap = if inner { l.flap_cm } else { 0.0 };
        s.gains = ControlGains {
            aileron: if inner { 0.0 } else { -side * l.aileron },
            flap: if inner { l.flap_gain } else { 0.0 },
            spoiler: l.spoiler_gain,
            ..Default::default()
        };
        s.flap_cd = if inner { l.flap_cd } else { 0.0 };
        s.spoiler_cd = l.spoiler_cd;
        s.slipstream = if inner { l.slip_inner } else { 0.0 };
        s
    };
    let mut htail = SurfaceConfig::blank(SurfaceKind::HorizontalStab, DVec3::new(0.0, l.h_y, l.h_z));
    htail.area = l.h_area;
    htail.chord = (l.h_area / (l.h_ar.sqrt() * 1.0).max(1.0)).sqrt().max(0.8);
    htail.cl_alpha = l.h_cl_alpha;
    htail.alpha_stall_pos = d(15.0);
    htail.alpha_stall_neg = d(-15.0);
    htail.incidence = d(l.h_incidence_deg);
    htail.cd0 = 0.010;
    htail.aspect_ratio = l.h_ar;
    htail.oswald = 0.8;
    htail.gains = ControlGains { elevator: -l.elevator, ..Default::default() };
    htail.slipstream = l.h_slip;
    htail.downwash = l.h_downwash;
    htail.ice_sensitivity = 1.3;

    let mut vtail = SurfaceConfig::blank(SurfaceKind::VerticalStab, DVec3::new(0.0, l.v_y, l.v_z));
    vtail.area = l.v_area;
    vtail.chord = (l.v_area / l.v_ar).sqrt().max(0.8);
    vtail.normal = DVec3::X;
    vtail.cl_alpha = 2.6;
    vtail.alpha_stall_pos = d(18.0);
    vtail.alpha_stall_neg = d(-18.0);
    vtail.cd0 = 0.010;
    vtail.aspect_ratio = l.v_ar;
    vtail.oswald = 0.8;
    vtail.gains = ControlGains { rudder: -l.rudder, ..Default::default() };
    vtail.slipstream = l.v_slip;
    vtail.ice_sensitivity = 1.2;

    let mut fus = SurfaceConfig::blank(SurfaceKind::Fuselage, DVec3::new(0.0, 0.0, -0.1));
    fus.body_cda = l.body_cda;
    fus.area = 0.0;

    vec![
        wing(SurfaceKind::WingLeftOuter, -1.0, false),
        wing(SurfaceKind::WingLeftInner, -1.0, true),
        wing(SurfaceKind::WingRightInner, 1.0, true),
        wing(SurfaceKind::WingRightOuter, 1.0, false),
        htail,
        vtail,
        fus,
    ]
}

#[allow(clippy::too_many_arguments)]
fn wheel(
    pos: DVec3,
    rest_len: f64,
    radius: f64,
    k: f64,
    c_comp: f64,
    c_reb: f64,
    travel: f64,
    steer_deg: f64,
    brake: BrakeGroup,
    brake_power: f64,
    retractable: bool,
) -> ContactConfig {
    ContactConfig {
        kind: ContactKind::Wheel,
        pos,
        rest_len,
        radius,
        k,
        c_comp,
        c_reb,
        travel,
        steer_max: d(steer_deg),
        brake,
        mu_roll: 0.022,
        retractable,
        water_drag: 0.0,
        brake_power,
    }
}

fn hull(pos: DVec3, k: f64) -> ContactConfig {
    ContactConfig {
        kind: ContactKind::Hull,
        pos,
        rest_len: 0.0,
        radius: 0.05,
        k,
        c_comp: k * 0.04,
        c_reb: k * 0.04,
        travel: 0.2,
        steer_max: 0.0,
        brake: BrakeGroup::None,
        mu_roll: 0.5,
        retractable: false,
        water_drag: 0.0,
        brake_power: 1.0,
    }
}

/// 浮筒/船体点（吃水即压缩量）。
fn float_pt(pos: DVec3, k: f64, c: f64, water_drag: f64, steer_deg: f64) -> ContactConfig {
    ContactConfig {
        kind: ContactKind::Float,
        pos,
        rest_len: 0.0,
        radius: 0.02,
        k,
        c_comp: c,
        c_reb: c * 0.8,
        travel: 1.4,
        steer_max: d(steer_deg),
        brake: BrakeGroup::None,
        mu_roll: 0.5,
        retractable: false,
        water_drag,
        brake_power: 0.0,
    }
}

/// 三点式主轮静止重心离地高度（按 92% 重量由主轮承担估算）。
fn rest_height(main: &ContactConfig, n_main: f64, weight_n: f64) -> f64 {
    let comp = 0.92 * weight_n / (n_main * main.k);
    -main.pos.y + main.rest_len + main.radius - comp
}

/// 尾部擦地点高度：保证起飞抬头 `clear_deg` 度内不擦尾。
fn tail_strike_y(cg_h: f64, tail_z: f64, main_z: f64, clear_deg: f64) -> f64 {
    -cg_h + (tail_z - main_z) * d(clear_deg).tan() + 0.05
}

// ===================================================================================
// TWIN-6：DHC-6 风格双发涡桨 STOL
// ===================================================================================
pub fn twin6() -> AircraftConfig {
    let l = Layout {
        span: 19.8,
        area: 39.0,
        chord: 1.98,
        wing_y: 1.35,
        wing_z: 0.15,
        dihedral_deg: 1.0,
        cl_alpha: 5.1,
        alpha0_deg: -1.5,
        stall_in_deg: 15.0,
        stall_out_deg: 17.5,
        cd0: 0.0115,
        oswald: 0.80,
        cm0: -0.05,
        inner_share: 0.66,
        inner_x_frac: 0.15,
        outer_x_frac: 0.38,
        aileron: 0.16,
        flap_gain: 0.60,
        flap_cd: 0.12,
        flap_cm: -0.12,
        spoiler_gain: 0.0,
        spoiler_cd: 0.0,
        slip_inner: 0.5,
        h_area: 8.0,
        h_y: 3.2,
        h_z: 7.6,
        h_cl_alpha: 3.6,
        h_incidence_deg: -2.5,
        h_ar: 4.2,
        elevator: 0.34,
        h_slip: 0.15,
        h_downwash: 0.35,
        v_area: 6.4,
        v_y: 2.6,
        v_z: 7.6,
        v_ar: 1.6,
        rudder: 0.45,
        v_slip: 0.25,
        body_cda: DVec3::new(7.0, 12.0, 0.95),
    };
    let main = |side: f64| {
        wheel(
            DVec3::new(side * 1.45, -1.05, 0.9),
            0.55,
            0.40,
            260_000.0,
            24_000.0,
            38_000.0,
            0.32,
            0.0,
            if side < 0.0 { BrakeGroup::Left } else { BrakeGroup::Right },
            0.85,
            false,
        )
    };
    let nose = wheel(DVec3::new(0.0, -1.05, -4.6), 0.55, 0.32, 120_000.0, 12_000.0, 18_000.0, 0.32, 25.0, BrakeGroup::None, 0.0, false);
    let weight = (3400.0 + 900.0 + 0.7 * 1100.0) * G;
    let cg_h = rest_height(&main(1.0), 2.0, weight);
    let engine = |side: f64| EngineConfig {
        kind: EngineKind::Turboprop {
            power_w: 559_000.0,
            bsfc: 8.6e-8,
            max_rpm: 2200.0,
            prop_pitch_speed: 165.0,
            static_thrust: 9_200.0,
            reverse_ratio: 0.55,
        },
        pos: DVec3::new(side * 3.7, 1.0, -1.9),
        spin: 1.0,
        prop_diameter: 2.59,
    };
    AircraftConfig {
        code: "TWIN-6",
        name: "Twin-6 Otter STOL",
        mass_empty: 3400.0,
        payload: 900.0,
        fuel_capacity: 1100.0,
        inertia: DVec3::new(40_000.0, 56_000.0, 31_600.0),
        wing_area: l.area,
        wing_span: l.span,
        mean_chord: l.chord,
        surfaces: airframe(&l),
        engines: vec![engine(-1.0), engine(1.0)],
        contacts: vec![
            nose,
            main(-1.0),
            main(1.0),
            hull(DVec3::new(0.0, tail_strike_y(cg_h, 7.8, 0.9, 14.0), 7.8), 400_000.0),
            hull(DVec3::new(-9.9, 1.55, 0.2), 400_000.0),
            hull(DVec3::new(9.9, 1.55, 0.2), 400_000.0),
        ],
        structure: StructureLimits {
            limit_load_pos: 3.5,
            limit_load_neg: -1.4,
            ultimate_factor: 1.5,
            v_ne: 166.0 * KT,
            v_fe: 103.0 * KT,
            v_le: 10_000.0,
            max_touchdown_sink: 12.0 * FT,
            gear_collapse_sink: 18.0 * FT,
        },
        flap_detents: vec![0.0, 0.27, 0.54, 0.81, 1.0],
        has_deice: true,
        has_pitot_heat: true,
        max_ice_mass: 160.0,
        v_rotate: 58.0 * KT,
        v_best_climb: 80.0 * KT,
        v_approach: 70.0 * KT,
        control_rate: 3.0,
        gear_time: 0.0,
        rest_cg_height: cg_h,
        flap_time: 9.0,
        mach_crit: 0.70,
        gear_cda: 0.0,
        amphibious: false,
        has_speedbrake: false,
    }
}

// ===================================================================================
// JET-39：L-39 风格单发轻型涡扇教练机（+8/-4 G）
// ===================================================================================
pub fn jet39() -> AircraftConfig {
    let l = Layout {
        span: 9.46,
        area: 18.8,
        chord: 2.1,
        wing_y: 0.35,
        wing_z: 0.10,
        dihedral_deg: 2.0,
        cl_alpha: 4.3,
        alpha0_deg: -1.0,
        stall_in_deg: 16.5,
        stall_out_deg: 18.5,
        cd0: 0.0105,
        oswald: 0.80,
        cm0: -0.03,
        inner_share: 0.55,
        inner_x_frac: 0.14,
        outer_x_frac: 0.38,
        aileron: 0.075,
        flap_gain: 0.40,
        flap_cd: 0.10,
        flap_cm: -0.08,
        spoiler_gain: 0.0,
        spoiler_cd: 0.05,
        slip_inner: 0.0,
        h_area: 5.2,
        h_y: 0.9,
        h_z: 5.3,
        h_cl_alpha: 3.4,
        h_incidence_deg: -2.0,
        h_ar: 3.2,
        elevator: 0.42,
        h_slip: 0.0,
        h_downwash: 0.4,
        v_area: 3.2,
        v_y: 1.4,
        v_z: 5.2,
        v_ar: 1.5,
        rudder: 0.35,
        v_slip: 0.0,
        body_cda: DVec3::new(2.6, 3.2, 0.24),
    };
    let main = |side: f64| {
        wheel(
            DVec3::new(side * 1.8, -0.55, 0.55),
            0.40,
            0.30,
            230_000.0,
            20_000.0,
            30_000.0,
            0.30,
            0.0,
            if side < 0.0 { BrakeGroup::Left } else { BrakeGroup::Right },
            0.7,
            true,
        )
    };
    let nose = wheel(DVec3::new(0.0, -0.55, -3.4), 0.40, 0.22, 110_000.0, 10_000.0, 15_000.0, 0.30, 25.0, BrakeGroup::None, 0.0, true);
    let weight = (3500.0 + 190.0 + 0.7 * 1100.0) * G;
    let cg_h = rest_height(&main(1.0), 2.0, weight);
    AircraftConfig {
        code: "JET-39",
        name: "Jet-39 Albatross",
        mass_empty: 3500.0,
        payload: 190.0,
        fuel_capacity: 1100.0,
        inertia: DVec3::new(30_000.0, 35_000.0, 5_800.0),
        wing_area: l.area,
        wing_span: l.span,
        mean_chord: l.chord,
        surfaces: airframe(&l),
        engines: vec![EngineConfig {
            kind: EngineKind::Turbofan { thrust_n: 16_870.0, tsfc: 2.1e-5, bypass_decay: 0.45, reverse_ratio: 0.0 },
            pos: DVec3::new(0.0, 0.0, 1.2),
            spin: 0.0,
            prop_diameter: 0.0,
        }],
        contacts: vec![
            nose,
            main(-1.0),
            main(1.0),
            hull(DVec3::new(0.0, tail_strike_y(cg_h, 5.8, 0.55, 12.0), 5.8), 300_000.0),
            hull(DVec3::new(-4.8, 0.55, 0.4), 300_000.0),
            hull(DVec3::new(4.8, 0.55, 0.4), 300_000.0),
        ],
        structure: StructureLimits {
            limit_load_pos: 8.0,
            limit_load_neg: -4.0,
            ultimate_factor: 1.5,
            v_ne: 437.0 * KT,
            v_fe: 215.0 * KT,
            v_le: 215.0 * KT,
            max_touchdown_sink: 4.0,
            gear_collapse_sink: 5.8,
        },
        flap_detents: vec![0.0, 0.5, 1.0],
        has_deice: false,
        has_pitot_heat: true,
        max_ice_mass: 60.0,
        v_rotate: 100.0 * KT,
        v_best_climb: 270.0 * KT,
        v_approach: 112.0 * KT,
        control_rate: 5.0,
        gear_time: 6.0,
        rest_cg_height: cg_h,
        flap_time: 6.0,
        mach_crit: 0.76,
        gear_cda: 0.14,
        amphibious: false,
        has_speedbrake: true,
    }
}

// ===================================================================================
// LINER-73：B737-300 风格双发涡扇干线客机
// ===================================================================================
pub fn liner73() -> AircraftConfig {
    let l = Layout {
        span: 28.9,
        area: 91.0,
        chord: 3.9,
        wing_y: -0.9,
        wing_z: 0.5,
        dihedral_deg: 5.0,
        cl_alpha: 5.0,
        alpha0_deg: -2.0,
        stall_in_deg: 16.0,
        stall_out_deg: 18.0,
        cd0: 0.0095,
        oswald: 0.82,
        cm0: -0.06,
        inner_share: 0.66,
        inner_x_frac: 0.15,
        outer_x_frac: 0.36,
        aileron: 0.12,
        flap_gain: 0.42,
        flap_cd: 0.13,
        flap_cm: -0.12,
        spoiler_gain: -0.06,
        spoiler_cd: 0.06,
        slip_inner: 0.0,
        h_area: 22.0,
        h_y: 1.8,
        h_z: 15.5,
        h_cl_alpha: 3.9,
        h_incidence_deg: -2.0,
        h_ar: 4.8,
        elevator: 0.36,
        h_slip: 0.0,
        h_downwash: 0.4,
        v_area: 18.0,
        v_y: 4.5,
        v_z: 15.3,
        v_ar: 1.6,
        rudder: 0.35,
        v_slip: 0.0,
        body_cda: DVec3::new(30.0, 75.0, 0.85),
    };
    let main = |side: f64| {
        wheel(
            DVec3::new(side * 2.6, -1.4, 1.2),
            0.90,
            0.55,
            1_700_000.0,
            170_000.0,
            260_000.0,
            0.50,
            0.0,
            if side < 0.0 { BrakeGroup::Left } else { BrakeGroup::Right },
            0.5,
            true,
        )
    };
    let nose = wheel(DVec3::new(0.0, -1.4, -11.2), 0.90, 0.45, 1_000_000.0, 90_000.0, 140_000.0, 0.50, 70.0, BrakeGroup::None, 0.0, true);
    let weight = (32_000.0 + 11_000.0 + 0.7 * 15_000.0) * G;
    let cg_h = rest_height(&main(1.0), 2.0, weight);
    let engine = |side: f64| EngineConfig {
        kind: EngineKind::Turbofan { thrust_n: 88_800.0, tsfc: 1.7e-5, bypass_decay: 0.55, reverse_ratio: 0.45 },
        pos: DVec3::new(side * 5.0, -1.4, -3.0),
        spin: 0.0,
        prop_diameter: 0.0,
    };
    AircraftConfig {
        code: "LINER-73",
        name: "Liner-73 Classic",
        mass_empty: 32_000.0,
        payload: 11_000.0,
        fuel_capacity: 15_000.0,
        inertia: DVec3::new(2_800_000.0, 3_600_000.0, 1_000_000.0),
        wing_area: l.area,
        wing_span: l.span,
        mean_chord: l.chord,
        surfaces: airframe(&l),
        engines: vec![engine(-1.0), engine(1.0)],
        contacts: vec![
            nose,
            main(-1.0),
            main(1.0),
            hull(DVec3::new(0.0, tail_strike_y(cg_h, 16.5, 1.2, 11.0), 16.5), 3_000_000.0),
            hull(DVec3::new(-14.4, 0.9, 1.0), 3_000_000.0),
            hull(DVec3::new(14.4, 0.9, 1.0), 3_000_000.0),
        ],
        structure: StructureLimits {
            limit_load_pos: 2.5,
            limit_load_neg: -1.0,
            ultimate_factor: 1.5,
            v_ne: 340.0 * KT,
            v_fe: 215.0 * KT,
            v_le: 270.0 * KT,
            max_touchdown_sink: 3.0,
            gear_collapse_sink: 4.6,
        },
        flap_detents: vec![0.0, 1.0 / 40.0, 5.0 / 40.0, 10.0 / 40.0, 15.0 / 40.0, 25.0 / 40.0, 30.0 / 40.0, 1.0],
        has_deice: true,
        has_pitot_heat: true,
        max_ice_mass: 900.0,
        v_rotate: 140.0 * KT,
        v_best_climb: 280.0 * KT,
        v_approach: 140.0 * KT,
        control_rate: 1.6,
        gear_time: 10.0,
        rest_cg_height: cg_h,
        flap_time: 40.0,
        mach_crit: 0.74,
        gear_cda: 1.5,
        amphibious: false,
        has_speedbrake: true,
    }
}

// ===================================================================================
// SEA-415：CL-415 风格双发涡桨水陆两栖机（船体 + 翼尖浮筒，可收放轮式起落架）
// ===================================================================================
pub fn sea415() -> AircraftConfig {
    let l = Layout {
        span: 28.6,
        area: 100.3,
        chord: 3.5,
        wing_y: 2.5,
        wing_z: 0.3,
        dihedral_deg: 1.0,
        cl_alpha: 4.9,
        alpha0_deg: -1.5,
        stall_in_deg: 15.0,
        stall_out_deg: 17.0,
        cd0: 0.0125,
        oswald: 0.80,
        cm0: -0.05,
        inner_share: 0.62,
        inner_x_frac: 0.17,
        outer_x_frac: 0.38,
        aileron: 0.14,
        flap_gain: 0.50,
        flap_cd: 0.12,
        flap_cm: -0.12,
        spoiler_gain: 0.0,
        spoiler_cd: 0.0,
        slip_inner: 0.45,
        h_area: 24.0,
        h_y: 4.2,
        h_z: 10.0,
        h_cl_alpha: 3.6,
        h_incidence_deg: -2.0,
        h_ar: 4.0,
        elevator: 0.34,
        h_slip: 0.1,
        h_downwash: 0.35,
        v_area: 15.0,
        v_y: 3.4,
        v_z: 9.8,
        v_ar: 1.5,
        rudder: 0.40,
        v_slip: 0.2,
        body_cda: DVec3::new(25.0, 45.0, 2.1),
    };
    let main = |side: f64| {
        wheel(
            DVec3::new(side * 2.2, -1.3, 1.0),
            0.70,
            0.45,
            700_000.0,
            70_000.0,
            110_000.0,
            0.45,
            0.0,
            if side < 0.0 { BrakeGroup::Left } else { BrakeGroup::Right },
            0.6,
            true,
        )
    };
    let nose = wheel(DVec3::new(0.0, -1.3, -6.8), 0.70, 0.35, 400_000.0, 40_000.0, 60_000.0, 0.45, 60.0, BrakeGroup::None, 0.0, true);
    let weight = (12_900.0 + 1_300.0 + 0.7 * 3_300.0) * G;
    let cg_h = rest_height(&main(1.0), 2.0, weight);
    let engine = |side: f64| EngineConfig {
        kind: EngineKind::Turboprop {
            power_w: 1_775_000.0,
            bsfc: 8.2e-8,
            max_rpm: 1200.0,
            prop_pitch_speed: 170.0,
            static_thrust: 25_000.0,
            reverse_ratio: 0.5,
        },
        pos: DVec3::new(side * 5.2, 2.3, -2.2),
        spin: 1.0,
        prop_diameter: 3.97,
    };
    AircraftConfig {
        code: "SEA-415",
        name: "Sea-415 Waterbomber",
        mass_empty: 12_900.0,
        payload: 1_300.0,
        fuel_capacity: 3_300.0,
        inertia: DVec3::new(250_000.0, 450_000.0, 235_000.0),
        wing_area: l.area,
        wing_span: l.span,
        mean_chord: l.chord,
        surfaces: airframe(&l),
        engines: vec![engine(-1.0), engine(1.0)],
        contacts: vec![
            nose,
            main(-1.0),
            main(1.0),
            hull(DVec3::new(0.0, tail_strike_y(cg_h, 10.2, 1.0, 12.0), 10.2), 1_200_000.0),
            hull(DVec3::new(-14.3, 2.9, 0.3), 1_200_000.0),
            hull(DVec3::new(14.3, 2.9, 0.3), 1_200_000.0),
            // 船体/浮筒：艏、舯左右、艉（浮力力矩平衡 Σk·z = 0）+ 翼尖浮筒
            float_pt(DVec3::new(0.0, -1.5, -6.0), 100_000.0, 38_000.0, 0.05, 0.0),
            float_pt(DVec3::new(-1.1, -1.5, 0.5), 100_000.0, 42_000.0, 0.06, 0.0),
            float_pt(DVec3::new(1.1, -1.5, 0.5), 100_000.0, 42_000.0, 0.06, 0.0),
            float_pt(DVec3::new(0.0, -1.4, 8.0), 62_500.0, 26_000.0, 0.04, 20.0),
            float_pt(DVec3::new(-13.0, 1.2, 0.3), 40_000.0, 20_000.0, 0.03, 0.0),
            float_pt(DVec3::new(13.0, 1.2, 0.3), 40_000.0, 20_000.0, 0.03, 0.0),
        ],
        structure: StructureLimits {
            limit_load_pos: 3.5,
            limit_load_neg: -1.0,
            ultimate_factor: 1.5,
            v_ne: 200.0 * KT,
            v_fe: 135.0 * KT,
            v_le: 160.0 * KT,
            max_touchdown_sink: 4.0,
            gear_collapse_sink: 6.5,
        },
        flap_detents: vec![0.0, 0.33, 0.66, 1.0],
        has_deice: true,
        has_pitot_heat: true,
        max_ice_mass: 450.0,
        v_rotate: 82.0 * KT,
        v_best_climb: 125.0 * KT,
        v_approach: 90.0 * KT,
        control_rate: 2.4,
        gear_time: 9.0,
        rest_cg_height: cg_h,
        flap_time: 14.0,
        mach_crit: 0.65,
        gear_cda: 0.9,
        amphibious: true,
        has_speedbrake: false,
    }
}
