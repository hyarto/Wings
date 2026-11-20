//! 机型数据定义（数据驱动）。M1 内置 SKY-172；其余机型在 M4 补齐。

use crate::structure::StructureLimits;
use crate::{FT, KT};
use glam::DVec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceKind {
    WingLeftOuter,
    WingLeftInner,
    WingRightInner,
    WingRightOuter,
    HorizontalStab,
    VerticalStab,
    Fuselage,
}

impl SurfaceKind {
    pub fn is_left_wing(self) -> bool {
        matches!(self, Self::WingLeftOuter | Self::WingLeftInner)
    }
    pub fn is_right_wing(self) -> bool {
        matches!(self, Self::WingRightOuter | Self::WingRightInner)
    }
    pub fn is_wing(self) -> bool {
        self.is_left_wing() || self.is_right_wing()
    }
}

/// 每单位操纵输入对等效迎角 Δα（rad）的贡献。
#[derive(Clone, Copy, Debug, Default)]
pub struct ControlGains {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub flap: f64,
    pub spoiler: f64,
}

#[derive(Clone, Debug)]
pub struct SurfaceConfig {
    pub kind: SurfaceKind,
    /// 气动中心相对 CG 的机体系位置。
    pub pos: DVec3,
    pub area: f64,
    pub chord: f64,
    /// 升力正方向（机体系单位向量；水平翼 = +Y，垂尾 = +X）。
    pub normal: DVec3,
    pub cl_alpha: f64,
    /// 零升迎角（rad，弯度翼型为负）。
    pub alpha0: f64,
    pub alpha_stall_pos: f64,
    pub alpha_stall_neg: f64,
    /// 失速跌落过渡宽度（rad）。
    pub stall_width: f64,
    /// 安装角（rad）。
    pub incidence: f64,
    pub cd0: f64,
    pub aspect_ratio: f64,
    pub oswald: f64,
    pub cm0: f64,
    pub cm_flap: f64,
    pub gains: ControlGains,
    /// 襟翼放下时的附加型阻系数。
    pub flap_cd: f64,
    pub spoiler_cd: f64,
    /// 受螺旋桨滑流影响比例 0..1。
    pub slipstream: f64,
    /// 受机翼下洗影响系数（水平尾翼）。
    pub downwash: f64,
    /// 对结冰的敏感度（尾翼更敏感）。
    pub ice_sensitivity: f64,
    /// 仅 Fuselage 使用：沿机体 X/Y/Z 轴的 Cd·A（m²）。
    pub body_cda: DVec3,
}

impl SurfaceConfig {
    fn blank(kind: SurfaceKind, pos: DVec3) -> Self {
        Self {
            kind,
            pos,
            area: 0.0,
            chord: 1.0,
            normal: DVec3::Y,
            cl_alpha: 4.5,
            alpha0: 0.0,
            alpha_stall_pos: 16f64.to_radians(),
            alpha_stall_neg: -16f64.to_radians(),
            stall_width: 6f64.to_radians(),
            incidence: 0.0,
            cd0: 0.01,
            aspect_ratio: 7.0,
            oswald: 0.8,
            cm0: 0.0,
            cm_flap: 0.0,
            gains: ControlGains::default(),
            flap_cd: 0.0,
            spoiler_cd: 0.0,
            slipstream: 0.0,
            downwash: 0.0,
            ice_sensitivity: 1.0,
            body_cda: DVec3::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum EngineKind {
    /// 活塞螺旋桨（额定功率 W）。
    Piston { power_w: f64, bsfc: f64, max_rpm: f64, idle_rpm: f64, prop_pitch_speed: f64, static_thrust: f64 },
    /// 涡桨（轴功率 W，含反桨能力）。
    Turboprop { power_w: f64, bsfc: f64, max_rpm: f64, prop_pitch_speed: f64, static_thrust: f64, reverse_ratio: f64 },
    /// 涡扇（海平面静推力 N）。
    Turbofan { thrust_n: f64, tsfc: f64, bypass_decay: f64, reverse_ratio: f64 },
}

#[derive(Clone, Copy, Debug)]
pub struct EngineConfig {
    pub kind: EngineKind,
    pub pos: DVec3,
    /// +1 = 螺旋桨从驾驶员视角顺时针旋转（反扭矩使机体向左滚）。
    pub spin: f64,
    pub prop_diameter: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrakeGroup {
    None,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactKind {
    Wheel,
    /// 机体硬点（尾橇、翼尖、机腹）：接触即为刮擦/撞地。
    Hull,
}

#[derive(Clone, Copy, Debug)]
pub struct ContactConfig {
    pub kind: ContactKind,
    /// 支柱上端安装点（机体系）。
    pub pos: DVec3,
    pub rest_len: f64,
    pub radius: f64,
    pub k: f64,
    pub c_comp: f64,
    pub c_reb: f64,
    pub travel: f64,
    pub steer_max: f64,
    pub brake: BrakeGroup,
    pub mu_roll: f64,
    pub retractable: bool,
}

#[derive(Clone, Debug)]
pub struct AircraftConfig {
    pub code: &'static str,
    pub name: &'static str,
    pub mass_empty: f64,
    pub payload: f64,
    pub fuel_capacity: f64,
    /// (绕 X 俯仰, 绕 Y 偏航, 绕 Z 滚转) 转动惯量 kg·m²，对应空机+默认载荷。
    pub inertia: DVec3,
    pub wing_area: f64,
    pub wing_span: f64,
    pub mean_chord: f64,
    pub surfaces: Vec<SurfaceConfig>,
    pub engines: Vec<EngineConfig>,
    pub contacts: Vec<ContactConfig>,
    pub structure: StructureLimits,
    /// 襟翼挡位（0..1）。
    pub flap_detents: Vec<f64>,
    pub has_deice: bool,
    pub has_pitot_heat: bool,
    pub max_ice_mass: f64,
    /// 失速告警 / 参考速度 (IAS m/s)
    pub v_rotate: f64,
    pub v_best_climb: f64,
    pub v_approach: f64,
    /// 舵面最大偏转速率（归一化/秒）。
    pub control_rate: f64,
    /// 起落架收放时间（s），固定起落架为 0。
    pub gear_time: f64,
    /// 重心离地高度（静止，m）——用于放置飞机。
    pub rest_cg_height: f64,
}

impl AircraftConfig {
    pub fn by_code(code: &str) -> Option<AircraftConfig> {
        builtin().into_iter().find(|a| a.code.eq_ignore_ascii_case(code))
    }
    pub fn default_fuel_kg(&self) -> f64 {
        self.fuel_capacity * 0.7
    }
    pub fn max_takeoff_mass(&self) -> f64 {
        self.mass_empty + self.payload + self.fuel_capacity
    }
    pub fn total_engine_power_w(&self) -> f64 {
        self.engines
            .iter()
            .map(|e| match e.kind {
                EngineKind::Piston { power_w, .. } | EngineKind::Turboprop { power_w, .. } => power_w,
                EngineKind::Turbofan { thrust_n, .. } => thrust_n * 100.0,
            })
            .sum()
    }
}

/// 全部内置机型。
pub fn builtin() -> Vec<AircraftConfig> {
    vec![sky172()]
}

/// SKY-172：Cessna 172 风格单发活塞教练机。
pub fn sky172() -> AircraftConfig {
    let d = |x: f64| x.to_radians();
    let dihedral = d(2.0);
    let wing_y = 0.65;
    let wing_z = 0.12;
    let wing_area = 16.2;
    let span = 11.0;
    let ar = span * span / wing_area;

    let wing = |kind: SurfaceKind, side: f64, x: f64, area: f64, chord: f64, stall: f64, inner: bool| {
        let mut s = SurfaceConfig::blank(kind, DVec3::new(side * x, wing_y, wing_z));
        s.area = area;
        s.chord = chord;
        // 上反角：升力向量向内倾斜
        s.normal = DVec3::new(-side * dihedral.sin(), dihedral.cos(), 0.0);
        s.cl_alpha = 4.7;
        s.alpha0 = d(-1.5);
        s.alpha_stall_pos = d(stall);
        s.alpha_stall_neg = d(-14.0);
        s.stall_width = d(5.0);
        s.cd0 = 0.0115;
        s.aspect_ratio = ar;
        s.oswald = 0.77;
        s.cm0 = -0.045;
        s.cm_flap = if inner { -0.10 } else { 0.0 };
        // 副翼在外翼；襟翼在内翼。左右副翼反向。
        s.gains = ControlGains {
            aileron: if inner { 0.0 } else { -side * 0.16 },
            flap: if inner { 0.115 } else { 0.0 },
            spoiler: 0.0,
            ..Default::default()
        };
        s.flap_cd = if inner { 0.075 } else { 0.0 };
        s.slipstream = if inner { 0.45 } else { 0.0 };
        s
    };
    // 内翼 15.5° 先失速（良性失速特性），外翼 17.5° 保留副翼效能
    let wl_i = wing(SurfaceKind::WingLeftInner, -1.0, 1.4, 4.55, 1.6, 15.5, true);
    let wl_o = wing(SurfaceKind::WingLeftOuter, -1.0, 4.1, 3.55, 1.3, 17.5, false);
    let wr_i = wing(SurfaceKind::WingRightInner, 1.0, 1.4, 4.55, 1.6, 15.5, true);
    let wr_o = wing(SurfaceKind::WingRightOuter, 1.0, 4.1, 3.55, 1.3, 17.5, false);

    let mut htail = SurfaceConfig::blank(SurfaceKind::HorizontalStab, DVec3::new(0.0, 0.40, 4.90));
    htail.area = 3.4;
    htail.chord = 1.1;
    htail.cl_alpha = 3.7;
    htail.alpha_stall_pos = d(15.0);
    htail.alpha_stall_neg = d(-15.0);
    htail.incidence = d(-4.6);
    htail.cd0 = 0.010;
    htail.aspect_ratio = 3.8;
    htail.oswald = 0.8;
    htail.gains = ControlGains { elevator: -0.30, ..Default::default() };
    htail.slipstream = 0.55;
    htail.downwash = 0.5;
    htail.ice_sensitivity = 1.3;

    let mut vtail = SurfaceConfig::blank(SurfaceKind::VerticalStab, DVec3::new(0.0, 1.20, 5.00));
    vtail.area = 1.6;
    vtail.chord = 1.2;
    vtail.normal = DVec3::X;
    vtail.cl_alpha = 2.6;
    vtail.alpha_stall_pos = d(18.0);
    vtail.alpha_stall_neg = d(-18.0);
    vtail.cd0 = 0.010;
    vtail.aspect_ratio = 1.4;
    vtail.oswald = 0.8;
    vtail.gains = ControlGains { rudder: -0.33, ..Default::default() };
    vtail.slipstream = 0.6;
    vtail.ice_sensitivity = 1.2;

    let mut fus = SurfaceConfig::blank(SurfaceKind::Fuselage, DVec3::new(0.0, 0.0, -0.1));
    // X = 侧向，Y = 垂向，Z = 轴向（含固定起落架/支柱/发动机整流罩寄生阻力）
    fus.body_cda = DVec3::new(2.4, 3.0, 0.30);
    fus.area = 0.0;

    let main = |side: f64| ContactConfig {
        kind: ContactKind::Wheel,
        pos: DVec3::new(side * 1.25, -0.50, 0.35),
        rest_len: 0.43,
        radius: 0.17,
        k: 95_000.0,
        c_comp: 7_500.0,
        c_reb: 12_000.0,
        travel: 0.22,
        steer_max: 0.0,
        brake: if side < 0.0 { BrakeGroup::Left } else { BrakeGroup::Right },
        mu_roll: 0.025,
        retractable: false,
    };
    let nose = ContactConfig {
        kind: ContactKind::Wheel,
        pos: DVec3::new(0.0, -0.50, -1.35),
        rest_len: 0.43,
        radius: 0.16,
        k: 45_000.0,
        c_comp: 4_500.0,
        c_reb: 7_000.0,
        travel: 0.22,
        steer_max: d(10.0),
        brake: BrakeGroup::None,
        mu_roll: 0.03,
        retractable: false,
    };
    let hull = |pos: DVec3| ContactConfig {
        kind: ContactKind::Hull,
        pos,
        rest_len: 0.0,
        radius: 0.05,
        k: 300_000.0,
        c_comp: 12_000.0,
        c_reb: 12_000.0,
        travel: 0.15,
        steer_max: 0.0,
        brake: BrakeGroup::None,
        mu_roll: 0.5,
        retractable: false,
    };

    AircraftConfig {
        code: "SKY-172",
        name: "Sky 172 Skyhawk",
        mass_empty: 770.0,
        payload: 160.0,
        fuel_capacity: 144.0,
        inertia: DVec3::new(1825.0, 2667.0, 1285.0),
        wing_area,
        wing_span: span,
        mean_chord: 1.5,
        surfaces: vec![wl_o, wl_i, wr_i, wr_o, htail, vtail, fus],
        engines: vec![EngineConfig {
            kind: EngineKind::Piston {
                power_w: 119_300.0,
                bsfc: 7.8e-8,
                max_rpm: 2700.0,
                idle_rpm: 650.0,
                prop_pitch_speed: 130.0,
                static_thrust: 2250.0,
            },
            pos: DVec3::new(0.0, 0.10, -2.0),
            spin: 1.0,
            prop_diameter: 1.9,
        }],
        contacts: vec![
            nose,
            main(-1.0),
            main(1.0),
            hull(DVec3::new(0.0, -0.10, 5.20)),    // 尾橇
            hull(DVec3::new(-5.5, 0.60, 0.20)),    // 左翼尖
            hull(DVec3::new(5.5, 0.60, 0.20)),     // 右翼尖
        ],
        structure: StructureLimits {
            limit_load_pos: 3.8,
            limit_load_neg: -1.52,
            ultimate_factor: 1.5,
            v_ne: 163.0 * KT,
            v_fe: 85.0 * KT,
            v_le: 10_000.0,
            max_touchdown_sink: 10.0 * FT,
            gear_collapse_sink: 15.0 * FT,
        },
        flap_detents: vec![0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
        has_deice: false,
        has_pitot_heat: true,
        max_ice_mass: 45.0,
        v_rotate: 55.0 * KT,
        v_best_climb: 74.0 * KT,
        v_approach: 65.0 * KT,
        control_rate: 4.0,
        gear_time: 0.0,
        rest_cg_height: 1.07,
    }
}
