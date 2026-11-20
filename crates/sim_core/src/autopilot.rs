//! 简易 PID 自动驾驶/“虚拟飞行员”。用于无头试飞测试与客户端演示，不属于物理模型。

use crate::integrator::Simulation;
use crate::KT;

#[derive(Clone, Copy, Debug, Default)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub i: f64,
    pub i_limit: f64,
    prev_err: Option<f64>,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, i_limit: f64) -> Self {
        Self { kp, ki, kd, i: 0.0, i_limit, prev_err: None }
    }
    pub fn update(&mut self, err: f64, dt: f64) -> f64 {
        self.i = (self.i + err * dt * self.ki).clamp(-self.i_limit, self.i_limit);
        let d = match self.prev_err {
            Some(p) => (err - p) / dt,
            None => 0.0,
        };
        self.prev_err = Some(err);
        self.kp * err + self.i + self.kd * d
    }
    pub fn reset(&mut self) {
        self.i = 0.0;
        self.prev_err = None;
    }
}

/// 俯仰保持：用升降舵保持目标俯仰角（度）。
#[derive(Clone, Debug)]
pub struct PitchHold {
    pid: Pid,
}
impl Default for PitchHold {
    fn default() -> Self {
        Self { pid: Pid::new(0.08, 0.04, 0.0, 0.6) }
    }
}
impl PitchHold {
    pub fn control(&mut self, sim: &Simulation, target_pitch_deg: f64, dt: f64) -> f64 {
        let err = target_pitch_deg - sim.diag.pitch;
        // 阻尼项使用俯仰角速度（rad/s → 度/s）
        let q = sim.state.rigid.w.x.to_degrees();
        (self.pid.update(err, dt) - 0.05 * q).clamp(-1.0, 1.0)
    }
}

/// 滚转保持：副翼保持目标坡度（度）。
#[derive(Clone, Debug)]
pub struct BankHold {
    pid: Pid,
}
impl Default for BankHold {
    fn default() -> Self {
        Self { pid: Pid::new(0.03, 0.0, 0.0, 0.3) }
    }
}
impl BankHold {
    pub fn control(&mut self, sim: &Simulation, target_bank_deg: f64, dt: f64) -> f64 {
        let err = target_bank_deg - sim.diag.roll;
        let p = (-sim.state.rigid.w.z).to_degrees();
        (self.pid.update(err, dt) - 0.02 * p).clamp(-1.0, 1.0)
    }
}

/// 速度保持：通过调整目标俯仰来保持表速（用于爬升/巡航）。
#[derive(Clone, Debug)]
pub struct SpeedToPitch {
    pid: Pid,
}
impl Default for SpeedToPitch {
    fn default() -> Self {
        Self { pid: Pid::new(0.6, 0.05, 0.0, 12.0) }
    }
}
impl SpeedToPitch {
    /// 返回目标俯仰角（度）。速度偏高 → 抬头。
    pub fn target_pitch(&mut self, sim: &Simulation, target_ias: f64, dt: f64) -> f64 {
        let err = (sim.diag.ias_true - target_ias) / KT;
        (3.0 + self.pid.update(err, dt)).clamp(-15.0, 20.0)
    }
}

/// 航向保持：返回目标坡度（度）。
pub fn heading_to_bank(sim: &Simulation, target_heading_deg: f64, max_bank: f64) -> f64 {
    let mut err = target_heading_deg - sim.diag.heading;
    while err > 180.0 {
        err -= 360.0;
    }
    while err < -180.0 {
        err += 360.0;
    }
    (err * 1.2).clamp(-max_bank, max_bank)
}

/// 航向 → 方向舵（地面滑跑中保持直线）。
pub fn heading_to_rudder(sim: &Simulation, target_heading_deg: f64) -> f64 {
    let mut err = target_heading_deg - sim.diag.heading;
    while err > 180.0 {
        err -= 360.0;
    }
    while err < -180.0 {
        err += 360.0;
    }
    let r = -sim.state.rigid.w.y.to_degrees();
    (0.08 * err - 0.05 * r).clamp(-1.0, 1.0)
}
