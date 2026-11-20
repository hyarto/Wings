//! 起落架 / 机体硬点接触：弹簧-阻尼支柱、轮胎摩擦（干/湿/冰）、前轮转向、差动刹车、驻车刹车。

use crate::aircraft::{AircraftConfig, BrakeGroup, ContactKind};
use crate::state::RigidState;
use glam::DVec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceType {
    Asphalt,
    Grass,
    Dirt,
    Snow,
    Water,
}

#[derive(Clone, Copy, Debug)]
pub struct GroundSample {
    pub height: f64,
    pub normal: DVec3,
    pub surface: SurfaceType,
}

/// 地面查询接口：由 `world_gen`/客户端实现，sim_core 只依赖该 trait。
pub trait Ground: Send + Sync {
    fn sample(&self, x: f64, z: f64) -> GroundSample;
}

/// 平坦地面（测试/M1 原型）。
#[derive(Clone, Copy, Debug)]
pub struct FlatGround {
    pub elevation: f64,
    pub surface: SurfaceType,
}

impl Default for FlatGround {
    fn default() -> Self {
        Self { elevation: 0.0, surface: SurfaceType::Asphalt }
    }
}

impl Ground for FlatGround {
    fn sample(&self, _x: f64, _z: f64) -> GroundSample {
        GroundSample { height: self.elevation, normal: DVec3::Y, surface: self.surface }
    }
}

/// 场面状态：来自天气。
#[derive(Clone, Copy, Debug, Default)]
pub struct GroundConditions {
    /// 湿度 0..1（降雨 → 湿跑道）。
    pub wetness: f64,
    /// 结冰/积雪 0..1。
    pub icing: f64,
}

/// 轮胎-地面摩擦系数：干沥青 ≈0.8，雨湿 ≈0.45，结冰/积雪 ≈0.15。
pub fn friction_coefficient(surface: SurfaceType, cond: GroundConditions) -> f64 {
    let dry = match surface {
        SurfaceType::Asphalt => 0.80,
        SurfaceType::Grass => 0.50,
        SurfaceType::Dirt => 0.55,
        SurfaceType::Snow => 0.20,
        SurfaceType::Water => 0.03,
    };
    if surface == SurfaceType::Water {
        return dry;
    }
    let wet = dry * (1.0 - 0.4375 * cond.wetness.clamp(0.0, 1.0));
    let ice = cond.icing.clamp(0.0, 1.0);
    if wet > 0.15 {
        wet * (1.0 - ice) + 0.15 * ice
    } else {
        wet
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GearInputs {
    /// 起落架放下比例 0..1。
    pub extension: f64,
    /// 前轮转向 -1..1（+ = 右）。
    pub steer: f64,
    pub brake_left: f64,
    pub brake_right: f64,
    pub conditions: GroundConditions,
    pub collapsed: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactDiag {
    pub in_contact: bool,
    pub compression: f64,
    pub normal_force: f64,
    /// 沿轮向的地速（m/s）。
    pub roll_speed: f64,
    /// 压缩速率（>0 = 正在压缩）。
    pub compress_rate: f64,
    pub is_hull: bool,
}

#[derive(Clone, Debug)]
pub struct GearOutput {
    /// 机体系合力（N）。
    pub force: DVec3,
    /// 机体系绕 CG 合力矩。
    pub moment: DVec3,
    pub contacts: [ContactDiag; 16],
    pub wheel_contact: bool,
    pub hull_contact: bool,
    /// 机体硬点最大撞击速度（m/s）。
    pub hull_impact_speed: f64,
    /// 浮筒/船体点正在水中（M4）。
    pub float_contact: bool,
    /// 轮子陷入水中（起落架放下入水 → 翻覆）。
    pub wheel_water: bool,
    /// 浮筒/船体点最大水中冲击速度（m/s）。
    pub water_impact_speed: f64,
    /// 吃水总量（m，所有水中接触点压缩量之和）。
    pub total_draft: f64,
    /// 所有主轮/前轮的总法向力。
    pub total_normal: f64,
}

impl Default for GearOutput {
    fn default() -> Self {
        Self {
            force: DVec3::ZERO,
            moment: DVec3::ZERO,
            contacts: [ContactDiag::default(); 16],
            wheel_contact: false,
            hull_contact: false,
            hull_impact_speed: 0.0,
            float_contact: false,
            wheel_water: false,
            water_impact_speed: 0.0,
            total_draft: 0.0,
            total_normal: 0.0,
        }
    }
}

/// 水密度（kg/m³）。
pub const RHO_WATER: f64 = 1000.0;
/// 轮子（及支柱）入水时的水动阻力面积系数（m²/m）：极大，使入水翻覆。
pub const WHEEL_WATER_DRAG: f64 = 1.6;

pub fn evaluate(cfg: &AircraftConfig, rs: &RigidState, ground: &dyn Ground, inp: &GearInputs, out: &mut GearOutput) {
    *out = GearOutput::default();
    let q = rs.q;
    let qi = q.inverse();
    let w_world = q * rs.w;
    let axis = q * (-DVec3::Y);

    for (i, c) in cfg.contacts.iter().enumerate() {
        let is_float = c.kind == ContactKind::Float;
        let mut is_hull = c.kind == ContactKind::Hull;
        let (mut rest_len, mut radius, mut k, mut mu_roll, mut brake_group) =
            (c.rest_len, c.radius, c.k, c.mu_roll, c.brake);
        if !is_hull {
            if c.retractable {
                if inp.extension < 0.8 {
                    continue;
                }
                rest_len *= inp.extension;
            }
            if inp.collapsed {
                // 起落架坍塌：机腹刮擦
                rest_len = 0.02;
                radius = 0.05;
                k *= 2.0;
                mu_roll = 0.5;
                brake_group = BrakeGroup::None;
            }
        }
        let attach = rs.pos + q * c.pos;
        let probe = attach + axis * (rest_len + radius);
        let g = ground.sample(probe.x, probe.z);
        let on_water = g.surface == SurfaceType::Water;
        // 浮筒在陆地上等同于机体硬点
        if is_float && !on_water {
            is_hull = true;
        }
        let water_mode = on_water && (is_float || !is_hull) && c.kind != ContactKind::Hull;
        let n = g.normal;
        let an = axis.dot(n);
        if an > -0.05 {
            continue;
        }
        let p_g = DVec3::new(probe.x, g.height, probe.z);
        let t = (p_g - attach).dot(n) / an;
        let compression = rest_len + radius - t;
        if compression <= 0.0 {
            continue;
        }
        let contact_pt = attach + axis * t;
        let r_w = contact_pt - rs.pos;
        let v_attach = rs.vel + w_world.cross(attach - rs.pos);
        let rate = v_attach.dot(axis);
        let v_pt = rs.vel + w_world.cross(r_w);

        let c_damp = if rate > 0.0 { c.c_comp } else { c.c_reb };
        let mut nf = k * compression + c_damp * rate;
        if compression > c.travel {
            nf += 20.0 * k * (compression - c.travel);
        }
        nf = nf.max(0.0);

        // 轮向
        let steer = c.steer_max * inp.steer;
        let fwd_body = DVec3::new(steer.sin(), 0.0, -steer.cos());
        let fwd_w = q * fwd_body;
        let f_proj = fwd_w - n * fwd_w.dot(n);
        let f = if f_proj.length() > 1e-6 { f_proj.normalize() } else { continue };
        let right = f.cross(n);
        let vx = v_pt.dot(f);
        let vy = v_pt.dot(right);

        if water_mode {
            // 浮力（弹簧-阻尼 = 排水体积）+ 水动阻力（正比于吃水与速度平方）
            let wd = if is_float { c.water_drag } else { WHEEL_WATER_DRAG };
            let depth = compression.min(c.travel.max(0.3) * 2.0);
            let visc = 2500.0;
            let kx = (0.5 * RHO_WATER * vx.abs() + visc) * wd * depth;
            let ky = (0.5 * RHO_WATER * vy.abs() + visc) * wd * depth * 6.0;
            let fx = -kx * vx;
            let fy = -ky * vy;
            let force_w = n * nf + f * fx + right * fy;
            out.force += qi * force_w;
            out.moment += qi * r_w.cross(force_w);
            out.total_normal += nf;
            out.total_draft += compression;
            if i < 16 {
                out.contacts[i] = ContactDiag {
                    in_contact: true,
                    compression,
                    normal_force: nf,
                    roll_speed: vx,
                    compress_rate: rate,
                    is_hull: false,
                };
            }
            if is_float {
                out.float_contact = true;
                out.water_impact_speed = out.water_impact_speed.max(rate.max(0.0));
            } else {
                out.wheel_contact = true;
                out.wheel_water = true;
            }
            continue;
        }

        let mu = if is_hull {
            0.5
        } else if inp.collapsed {
            0.4
        } else {
            friction_coefficient(g.surface, inp.conditions)
        };
        let brake = match brake_group {
            BrakeGroup::Left => inp.brake_left,
            BrakeGroup::Right => inp.brake_right,
            BrakeGroup::None => 0.0,
        }
        .clamp(0.0, 1.0);

        let long_coeff = mu_roll + brake * mu * c.brake_power;
        let mut fx = -long_coeff * nf * (vx / 0.4).tanh();
        let mut fy = -mu * nf * (vy / 0.25).tanh();
        let max_f = mu * nf;
        let mag = fx.hypot(fy);
        if mag > max_f && mag > 1e-9 {
            let s = max_f / mag;
            fx *= s;
            fy *= s;
        }

        let force_w = n * nf + f * fx + right * fy;
        out.force += qi * force_w;
        out.moment += qi * r_w.cross(force_w);
        out.total_normal += nf;
        if i < 16 {
            out.contacts[i] = ContactDiag {
                in_contact: true,
                compression,
                normal_force: nf,
                roll_speed: vx,
                compress_rate: rate,
                is_hull,
            };
        }
        if is_hull {
            out.hull_contact = true;
            out.hull_impact_speed = out.hull_impact_speed.max(rate.max(0.0));
        } else {
            out.wheel_contact = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn friction_values() {
        let dry = friction_coefficient(SurfaceType::Asphalt, GroundConditions::default());
        let wet = friction_coefficient(SurfaceType::Asphalt, GroundConditions { wetness: 1.0, icing: 0.0 });
        let ice = friction_coefficient(SurfaceType::Asphalt, GroundConditions { wetness: 1.0, icing: 1.0 });
        assert!((dry - 0.8).abs() < 1e-9);
        assert!((wet - 0.45).abs() < 1e-9);
        assert!((ice - 0.15).abs() < 1e-9);
    }
}
