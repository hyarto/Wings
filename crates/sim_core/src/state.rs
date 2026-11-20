//! 飞机完整物理状态与控制输入。

use crate::structure::DamageState;
use glam::{DQuat, DVec3};

/// 刚体状态（全部 f64）。位置/速度为世界系，角速度为机体系。
#[derive(Clone, Copy, Debug)]
pub struct RigidState {
    pub pos: DVec3,
    pub vel: DVec3,
    /// 机体 → 世界。
    pub q: DQuat,
    /// 机体系角速度。
    pub w: DVec3,
}

impl RigidState {
    pub fn new(pos: DVec3, q: DQuat) -> Self {
        Self { pos, vel: DVec3::ZERO, q, w: DVec3::ZERO }
    }
}

/// 驾驶员/手柄输入（归一化）。
#[derive(Clone, Copy, Debug)]
pub struct ControlInputs {
    /// 升降舵：+1 = 拉杆（抬头），-1 = 推杆。
    pub pitch: f64,
    /// 副翼：+1 = 右滚。
    pub roll: f64,
    /// 方向舵/前轮转向：+1 = 右脚蹬。
    pub yaw: f64,
    pub throttle: f64,
    /// 襟翼目标位置 0..1。
    pub flaps: f64,
    pub gear_down: bool,
    pub brake_left: f64,
    pub brake_right: f64,
    pub parking_brake: bool,
    /// 升降舵配平 -1..1（+ = 抬头）。
    pub pitch_trim: f64,
    /// 减速板/扰流板 0..1。
    pub speedbrake: f64,
    pub reverse: bool,
    pub pitot_heat: bool,
    pub deice: bool,
    /// 顺桨（仅对已失效的涡桨发动机生效）。
    pub feather: bool,
}

impl Default for ControlInputs {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            flaps: 0.0,
            gear_down: true,
            brake_left: 0.0,
            brake_right: 0.0,
            parking_brake: false,
            pitch_trim: 0.0,
            speedbrake: 0.0,
            reverse: false,
            pitot_heat: false,
            deice: false,
            feather: false,
        }
    }
}

/// 舵面实际位置（受速率限制地追随指令）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfacePositions {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub flaps: f64,
    pub spoiler: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct EngineState {
    /// 经过伺服延迟的油门。
    pub throttle: f64,
    /// 螺旋桨/风扇转速 RPM（活塞、涡桨）或 N1%（涡扇，0..100）。
    pub rpm: f64,
    pub power_w: f64,
    pub thrust_n: f64,
    pub fuel_flow: f64, // kg/s
    pub running: bool,
    pub failed: bool,
    /// 螺旋桨已顺桨（失效后风车阻力大幅降低）。
    pub feathered: bool,
}

impl Default for EngineState {
    fn default() -> Self {
        Self { throttle: 0.0, rpm: 0.0, power_w: 0.0, thrust_n: 0.0, fuel_flow: 0.0, running: true, failed: false, feathered: false }
    }
}

#[derive(Clone, Debug)]
pub struct AircraftState {
    pub rigid: RigidState,
    pub surfaces: SurfacePositions,
    /// 起落架收放位置 0（收）..1（放）。
    pub gear_pos: f64,
    pub engines: Vec<EngineState>,
    pub fuel_kg: f64,
    /// 结冰量 0..1。
    pub ice_load: f64,
    /// 皮托管是否被冰堵塞，及冻结的空速示数。
    pub pitot_blocked: bool,
    pub pitot_ice_timer: f64,
    pub frozen_ias: f64,
    pub damage: DamageState,
    /// 上一步各接触点是否接触（用于触地检测）。
    pub contact_prev: [bool; 16],
    pub prev_vy: f64,
    /// 离地持续时间（s），用于触地事件判定。
    pub air_time: f64,
    /// 最近一次触地下沉率（m/s）。
    pub last_touchdown_sink: f64,
    pub on_ground: bool,
    pub time: f64,
}
