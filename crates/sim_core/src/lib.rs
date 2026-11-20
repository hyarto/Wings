//! sim_core — 纯 Rust 6-DOF 硬核飞行动力学核心。
//!
//! **严禁依赖 bevy**：全部逻辑可通过 `cargo test` 无头运行。
//!
//! # 坐标系约定（与 Bevy 完全一致，便于渲染层直接使用）
//! * 世界系：`+X` 东，`+Y` 上，`-Z` 北（`+Z` 南）。`Y = 0` 为海平面。
//! * 机体系：`+X` 右翼，`+Y` 机顶，`-Z` 机头（前向）。
//! * 角速度 `ω` 以机体系表示：
//!   * 抬头（俯仰率 q）= 绕 `+X` 正向旋转；
//!   * 右滚（滚转率 p）= 绕 `-Z` 正向旋转（即 `ω.z < 0`）；
//!   * 右偏航（偏航率 r）= 绕 `-Y` 正向旋转（即 `ω.y < 0`）。
//! * 单位：SI（m, kg, s, N, rad）。

pub mod aero;
pub mod aircraft;
pub mod atmosphere;
pub mod autopilot;
pub mod gear;
pub mod integrator;
pub mod noise;
pub mod propulsion;
pub mod state;
pub mod structure;
pub mod weather;

/// 固定物理步长（120 Hz）。
pub const FIXED_DT: f64 = 1.0 / 120.0;
/// 标准重力加速度。
pub const G: f64 = 9.80665;
/// 节 → m/s。
pub const KT: f64 = 0.514_444_444;
/// 英尺 → 米。
pub const FT: f64 = 0.3048;

/// 机体系前向单位向量。
pub const FWD: glam::DVec3 = glam::DVec3::new(0.0, 0.0, -1.0);
/// 机体系上向单位向量。
pub const UP: glam::DVec3 = glam::DVec3::new(0.0, 1.0, 0.0);
/// 机体系右向单位向量。
pub const RIGHT: glam::DVec3 = glam::DVec3::new(1.0, 0.0, 0.0);

#[inline]
pub fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[inline]
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// 由 (航向°[0 = 北，顺时针], 俯仰°, 横滚°[右滚为正]) 构造姿态四元数（机体→世界）。
pub fn attitude_from_euler(heading_deg: f64, pitch_deg: f64, roll_deg: f64) -> glam::DQuat {
    use glam::DQuat;
    let yaw = DQuat::from_rotation_y(-heading_deg.to_radians());
    let pitch = DQuat::from_rotation_x(pitch_deg.to_radians());
    // 右滚 = 绕 -Z 正向 = 绕 +Z 负向
    let roll = DQuat::from_rotation_z(-roll_deg.to_radians());
    yaw * pitch * roll
}

/// 由姿态四元数提取 (航向°[0,360), 俯仰°, 横滚°[右滚为正])。
pub fn euler_from_attitude(q: glam::DQuat) -> (f64, f64, f64) {
    use glam::DVec3;
    let fwd = q * FWD;
    let right = q * RIGHT;
    let pitch = fwd.y.clamp(-1.0, 1.0).asin();
    let mut heading = fwd.x.atan2(-fwd.z).to_degrees();
    if heading < 0.0 {
        heading += 360.0;
    }
    let flat = DVec3::new(fwd.x, 0.0, fwd.z);
    let hr = if flat.length() > 1e-6 { flat.normalize().cross(DVec3::Y) } else { DVec3::X };
    // right = (cosφ·hr, -sinφ·cosθ ...)：右翼下沉 → right.y < 0 → 横滚为正
    let cos_p = pitch.cos().max(1e-6);
    let roll = (-right.y / cos_p).atan2(right.dot(hr));
    (heading, pitch.to_degrees(), roll.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn euler_roundtrip() {
        for &(h, p, r) in &[(0.0, 0.0, 0.0), (90.0, 10.0, 20.0), (270.0, -15.0, -35.0), (180.0, 5.0, 60.0)] {
            let q = attitude_from_euler(h, p, r);
            let (h2, p2, r2) = euler_from_attitude(q);
            assert!((h - h2).abs() < 1e-6 && (p - p2).abs() < 1e-6 && (r - r2).abs() < 1e-6, "{h} {p} {r} -> {h2} {p2} {r2}");
        }
    }
    #[test]
    fn heading_90_points_east() {
        let q = attitude_from_euler(90.0, 0.0, 0.0);
        let f = q * FWD;
        assert!(f.x > 0.99);
    }
}
