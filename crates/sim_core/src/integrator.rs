//! 固定步长（120 Hz）RK4 + 子步积分器，以及把全部子系统串起来的 `Simulation`。

use crate::aero::{self, AeroInputs, AeroOutput};
use crate::aircraft::AircraftConfig;
use crate::atmosphere::{calibrated_airspeed, AtmosphereState};
use crate::gear::{self, Ground, GearInputs, GearOutput};
use crate::propulsion;
use crate::state::*;
use crate::structure::{self, StructureInput, StructureWarnings};
use crate::weather::{self, IcingInputs, WeatherState};
use crate::{euler_from_attitude, FIXED_DT, FWD, G};
use glam::{DQuat, DVec3};

/// 每步更新的诊断量（仪表/HUD/测试读取）。
#[derive(Clone, Debug)]
pub struct Diagnostics {
    pub atm: AtmosphereState,
    pub tas: f64,
    /// 真实表速（CAS）。
    pub ias_true: f64,
    /// 仪表显示表速（皮托管结冰时冻结）。
    pub ias: f64,
    pub mach: f64,
    /// 迎角（rad，抬头气流为正）。
    pub aoa: f64,
    pub sideslip: f64,
    pub nz: f64,
    pub agl: f64,
    pub ground_height: f64,
    pub vertical_speed: f64,
    pub ground_speed: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub mass: f64,
    pub wind: DVec3,
    pub in_cloud: bool,
    pub visibility: f64,
    pub icing_intensity: f64,
    pub stall_warning: bool,
    pub warnings: StructureWarnings,
    pub aero: AeroOutput,
    pub gear: GearOutput,
    /// 本步发生触地事件时的下沉率。
    pub touchdown: Option<f64>,
}

#[derive(Clone, Copy)]
struct Deriv {
    dpos: DVec3,
    dvel: DVec3,
    dq: DQuat,
    dw: DVec3,
}

fn advance(s: &RigidState, d: &Deriv, h: f64) -> RigidState {
    RigidState { pos: s.pos + d.dpos * h, vel: s.vel + d.dvel * h, q: s.q + d.dq * h, w: s.w + d.dw * h }
}

struct Dynamics<'a> {
    cfg: &'a AircraftConfig,
    ground: &'a dyn Ground,
    wind: &'a [DVec3],
    rho: f64,
    ice: f64,
    surfaces: SurfacePositions,
    trim: f64,
    damage: &'a crate::structure::DamageState,
    slipstream: f64,
    thrust_force: DVec3,
    thrust_moment: DVec3,
    gear_in: GearInputs,
    mass: f64,
    inertia: DVec3,
}

impl Dynamics<'_> {
    fn eval(&self, s: &RigidState, aero_out: &mut AeroOutput, gear_out: &mut GearOutput) -> (Deriv, DVec3) {
        let gh = self.ground.sample(s.pos.x, s.pos.z).height;
        let inp = AeroInputs {
            rho: self.rho,
            wind: self.wind,
            ice: self.ice,
            agl: s.pos.y - gh,
            surfaces: self.surfaces,
            trim: self.trim,
            damage: self.damage,
            slipstream: self.slipstream,
        };
        aero::compute(self.cfg, s, &inp, aero_out);
        gear::evaluate(self.cfg, s, self.ground, &self.gear_in, gear_out);

        let f_body = aero_out.force + gear_out.force + self.thrust_force;
        let m_body = aero_out.moment + gear_out.moment + self.thrust_moment;
        let acc = s.q * f_body / self.mass + DVec3::new(0.0, -G, 0.0);

        let iw = self.inertia * s.w;
        let dw = (m_body - s.w.cross(iw)) / self.inertia;
        let omega_q = DQuat::from_xyzw(s.w.x, s.w.y, s.w.z, 0.0);
        let dq = (s.q * omega_q) * 0.5;
        (Deriv { dpos: s.vel, dvel: acc, dq, dw }, f_body)
    }
}

pub struct Simulation {
    pub config: AircraftConfig,
    pub state: AircraftState,
    pub controls: ControlInputs,
    pub weather: WeatherState,
    pub ground: Box<dyn Ground>,
    pub diag: Diagnostics,
    /// 每个 FIXED_DT 内的 RK4 子步数。
    pub substeps: u32,
    accumulator: f64,
    scratch_aero: AeroOutput,
    scratch_gear: GearOutput,
}

impl Simulation {
    /// 在地面（跑道）上放置飞机：`pos_xz` = (x, z)，航向 0 = 北。
    pub fn new(config: AircraftConfig, ground: Box<dyn Ground>, pos_xz: (f64, f64), heading_deg: f64) -> Self {
        let gh = ground.sample(pos_xz.0, pos_xz.1).height;
        let pos = DVec3::new(pos_xz.0, gh + config.rest_cg_height - 0.04, pos_xz.1);
        let q = crate::attitude_from_euler(heading_deg, 0.0, 0.0);
        let engines = config.engines.iter().map(|_| EngineState::default()).collect();
        let fuel = config.default_fuel_kg();
        let weather = WeatherState::calm();
        let atm = weather.atmosphere.sample(pos.y);
        let state = AircraftState {
            rigid: RigidState::new(pos, q),
            surfaces: SurfacePositions::default(),
            gear_pos: 1.0,
            engines,
            fuel_kg: fuel,
            ice_load: 0.0,
            pitot_blocked: false,
            pitot_ice_timer: 0.0,
            frozen_ias: 0.0,
            damage: Default::default(),
            contact_prev: [false; 16],
            prev_vy: 0.0,
            air_time: 0.0,
            last_touchdown_sink: 0.0,
            on_ground: true,
            time: 0.0,
        };
        let diag = Diagnostics {
            atm,
            tas: 0.0,
            ias_true: 0.0,
            ias: 0.0,
            mach: 0.0,
            aoa: 0.0,
            sideslip: 0.0,
            nz: 1.0,
            agl: config.rest_cg_height,
            ground_height: gh,
            vertical_speed: 0.0,
            ground_speed: 0.0,
            heading: heading_deg,
            pitch: 0.0,
            roll: 0.0,
            mass: config.mass_empty + config.payload + fuel,
            wind: DVec3::ZERO,
            in_cloud: false,
            visibility: 30_000.0,
            icing_intensity: 0.0,
            stall_warning: false,
            warnings: Default::default(),
            aero: AeroOutput::default(),
            gear: GearOutput::default(),
            touchdown: None,
        };
        let mut s = Self {
            config,
            state,
            controls: ControlInputs::default(),
            weather,
            ground,
            diag,
            substeps: 2,
            accumulator: 0.0,
            scratch_aero: AeroOutput::default(),
            scratch_gear: GearOutput::default(),
        };
        s.state.engines.iter_mut().for_each(|e| e.running = true);
        s
    }

    /// 直接放置在空中：高度（MSL m）、表速（m/s）、航向。
    pub fn set_airborne(&mut self, alt_msl: f64, ias: f64, heading_deg: f64) {
        let atm = self.weather.atmosphere.sample(alt_msl);
        let tas = crate::atmosphere::tas_from_cas(ias, &atm);
        let q = crate::attitude_from_euler(heading_deg, 3.0, 0.0);
        self.state.rigid.pos.y = alt_msl;
        self.state.rigid.q = q;
        self.state.rigid.vel = (q * FWD).with_y(0.0).normalize() * tas;
        self.state.rigid.w = DVec3::ZERO;
        self.state.on_ground = false;
        self.state.air_time = 10.0;
        for e in &mut self.state.engines {
            e.throttle = 0.6;
        }
    }

    pub fn total_mass(&self) -> f64 {
        let c = &self.config;
        c.mass_empty + c.payload + self.state.fuel_kg + self.state.ice_load * c.max_ice_mass
    }

    /// 以真实时间推进，内部按固定步长迭代；返回执行的步数。
    pub fn advance(&mut self, real_dt: f64) -> u32 {
        self.accumulator = (self.accumulator + real_dt).min(8.0 * FIXED_DT);
        let mut n = 0;
        while self.accumulator >= FIXED_DT {
            self.step();
            self.accumulator -= FIXED_DT;
            n += 1;
        }
        n
    }

    /// 推进一个固定步长。
    pub fn step(&mut self) {
        let dt = FIXED_DT;
        let ctl = self.controls;

        // ---------- 1. 作动器 ----------
        {
            let rate = self.config.control_rate * dt;
            let st = &mut self.state.surfaces;
            let follow = |cur: &mut f64, target: f64, r: f64| *cur += (target - *cur).clamp(-r, r);
            follow(&mut st.elevator, ctl.pitch.clamp(-1.0, 1.0), rate);
            follow(&mut st.aileron, ctl.roll.clamp(-1.0, 1.0), rate);
            follow(&mut st.rudder, ctl.yaw.clamp(-1.0, 1.0), rate);
            if self.state.damage.flaps_jammed.is_none() {
                follow(&mut st.flaps, ctl.flaps.clamp(0.0, 1.0), 0.09 * dt);
            }
            follow(&mut st.spoiler, ctl.speedbrake.clamp(0.0, 1.0), 1.5 * dt);
            if self.config.gear_time > 0.0 {
                let tgt = if ctl.gear_down { 1.0 } else { 0.0 };
                let r = dt / self.config.gear_time;
                self.state.gear_pos += (tgt - self.state.gear_pos).clamp(-r, r);
            } else {
                self.state.gear_pos = 1.0;
            }
        }

        // ---------- 2. 环境 ----------
        let rs0 = self.state.rigid;
        let qi = rs0.q.inverse();
        let g0 = self.ground.sample(rs0.pos.x, rs0.pos.z);
        let atm = self.weather.atmosphere.sample(rs0.pos.y);
        let t = self.state.time;
        let wind_cg = self.weather.wind_at(rs0.pos, rs0.pos.y - g0.height, t);
        let wind: Vec<DVec3> = self
            .config
            .surfaces
            .iter()
            .map(|s| {
                let wp = rs0.pos + rs0.q * s.pos;
                self.weather.wind_at(wp, wp.y - g0.height, t)
            })
            .collect();
        let v_air_b = qi * (rs0.vel - wind_cg);
        let tas = v_air_b.length();

        // ---------- 3. 推进 ----------
        let mut thrust_force = DVec3::ZERO;
        let mut thrust_moment = DVec3::ZERO;
        let mut slipstream: f64 = 0.0;
        let deice_active = ctl.deice && self.config.has_deice;
        for (j, ecfg) in self.config.engines.iter().enumerate() {
            let est = &mut self.state.engines[j];
            let v_axial = v_air_b.dot(FWD);
            let fuel_ok = self.state.fuel_kg > 0.0;
            let out = propulsion::update(ecfg, est, ctl.throttle, ctl.reverse, v_axial, &atm, fuel_ok, dt);
            let derate = if deice_active { 0.97 } else { 1.0 };
            let thrust = out.thrust * derate;
            let f = FWD * thrust;
            thrust_force += f;
            thrust_moment += ecfg.pos.cross(f);
            thrust_moment.z += out.torque * ecfg.spin;
            thrust_moment.y += ecfg.spin * thrust.max(0.0) * 0.10;
            slipstream = slipstream.max(out.slipstream);
            self.state.fuel_kg = (self.state.fuel_kg - out.fuel_flow * dt).max(0.0);
        }

        // ---------- 4. 质量 ----------
        let mass = self.total_mass();
        let nominal = self.config.mass_empty + self.config.payload + self.config.default_fuel_kg();
        let inertia = self.config.inertia * (mass / nominal);

        // ---------- 5. RK4 ----------
        let mut brake_l = ctl.brake_left;
        let mut brake_r = ctl.brake_right;
        if ctl.parking_brake {
            brake_l = 1.0;
            brake_r = 1.0;
        }
        let conditions = self.weather.ground_conditions(atm.temperature_c());
        let dynamics = Dynamics {
            cfg: &self.config,
            ground: self.ground.as_ref(),
            wind: &wind,
            rho: atm.density,
            ice: self.state.ice_load,
            surfaces: self.state.surfaces,
            trim: ctl.pitch_trim,
            damage: &self.state.damage,
            slipstream,
            thrust_force,
            thrust_moment,
            gear_in: GearInputs {
                extension: self.state.gear_pos,
                steer: self.state.surfaces.rudder,
                brake_left: brake_l,
                brake_right: brake_r,
                conditions,
                collapsed: self.state.damage.gear_collapsed,
            },
            mass,
            inertia,
        };
        let sub = self.substeps.max(1);
        let h = dt / sub as f64;
        let mut s = rs0;
        let mut f_body_first = DVec3::ZERO;
        for i in 0..sub {
            let (k1, fb) = dynamics.eval(&s, &mut self.scratch_aero, &mut self.scratch_gear);
            if i == 0 {
                f_body_first = fb;
            }
            let (k2, _) = dynamics.eval(&advance(&s, &k1, 0.5 * h), &mut self.scratch_aero, &mut self.scratch_gear);
            let (k3, _) = dynamics.eval(&advance(&s, &k2, 0.5 * h), &mut self.scratch_aero, &mut self.scratch_gear);
            let (k4, _) = dynamics.eval(&advance(&s, &k3, h), &mut self.scratch_aero, &mut self.scratch_gear);
            let sum = Deriv {
                dpos: (k1.dpos + 2.0 * k2.dpos + 2.0 * k3.dpos + k4.dpos) / 6.0,
                dvel: (k1.dvel + 2.0 * k2.dvel + 2.0 * k3.dvel + k4.dvel) / 6.0,
                dq: (k1.dq + k2.dq * 2.0 + k3.dq * 2.0 + k4.dq) * (1.0 / 6.0),
                dw: (k1.dw + 2.0 * k2.dw + 2.0 * k3.dw + k4.dw) / 6.0,
            };
            s = advance(&s, &sum, h);
            s.q = s.q.normalize();
        }
        // 诊断用：对新状态做一次完整求值（气动/起落架）
        let (_, _) = dynamics.eval(&s, &mut self.scratch_aero, &mut self.scratch_gear);
        let aero_out = self.scratch_aero.clone();
        let gear_out = self.scratch_gear.clone();
        drop(dynamics);
        self.state.rigid = s;
        self.state.time += dt;

        // ---------- 6. 触地 / 撞地 ----------
        let any_wheel = gear_out.wheel_contact;
        let mut touchdown = None;
        if any_wheel {
            if self.state.air_time > 0.5 {
                let sink = (-self.state.prev_vy).max(0.0);
                touchdown = Some(sink);
                self.state.last_touchdown_sink = sink;
                structure::evaluate_touchdown(&self.config.structure, &mut self.state.damage, sink);
            }
            self.state.air_time = 0.0;
        } else {
            self.state.air_time += dt;
        }
        self.state.on_ground = any_wheel || gear_out.hull_contact;
        self.state.prev_vy = s.vel.y;
        for (i, c) in gear_out.contacts.iter().enumerate() {
            self.state.contact_prev[i] = c.in_contact;
        }
        if gear_out.hull_contact && gear_out.hull_impact_speed > 3.0 {
            self.state.damage.crashed = true;
        }
        let gh_new = self.ground.sample(s.pos.x, s.pos.z).height;
        if s.pos.y < gh_new - 0.3 {
            self.state.damage.crashed = true;
        }

        // ---------- 7. 空速 / 迎角 ----------
        let wind_new = self.weather.wind_at(s.pos, s.pos.y - gh_new, self.state.time);
        let atm_new = self.weather.atmosphere.sample(s.pos.y);
        let v_b = s.q.inverse() * (s.vel - wind_new);
        let tas_new = v_b.length();
        let ias_true = calibrated_airspeed(tas_new, &atm_new);
        let aoa = (-v_b.y).atan2(-v_b.z);
        let sideslip = if tas_new > 1.0 { (v_b.x / tas_new).clamp(-1.0, 1.0).asin() } else { 0.0 };
        let nz = f_body_first.y / (mass * G);

        // ---------- 8. 结冰 ----------
        let in_cloud = self.weather.in_cloud(s.pos);
        let precip = self.weather.precip_at(s.pos);
        let icing = IcingInputs {
            in_cloud,
            precip: if precip > 0.0 { self.weather.precip } else { weather::PrecipKind::None },
            precip_intensity: precip,
            sat_c: atm_new.temperature_c(),
            tas: tas_new,
            deice_active: deice_active && self.state.engines.iter().any(|e| e.running),
            pitot_heat: ctl.pitot_heat && self.config.has_pitot_heat,
        };
        let was_blocked = self.state.pitot_blocked;
        weather::update_ice(
            &mut self.state.ice_load,
            &mut self.state.pitot_blocked,
            &mut self.state.pitot_ice_timer,
            &icing,
            dt,
        );
        if self.state.pitot_blocked && !was_blocked {
            self.state.frozen_ias = ias_true;
        }
        let ias_display = if self.state.pitot_blocked { self.state.frozen_ias } else { ias_true };

        // ---------- 9. 结构 ----------
        let warnings = structure::update(
            &self.config.structure,
            &mut self.state.damage,
            &StructureInput {
                // 机翼载荷仅来自气动法向力（起落架冲击不经由机翼传递）
                nz: (aero_out.force.y + thrust_force.y) / (mass * G),
                ias: ias_true,
                flaps: self.state.surfaces.flaps,
                gear_pos: self.state.gear_pos,
                roll_rate: -s.w.z,
                dt,
            },
        );

        // ---------- 10. 诊断 ----------
        let (heading, pitch, roll) = euler_from_attitude(s.q);
        let wing_stall = self
            .config
            .surfaces
            .iter()
            .find(|c| c.kind.is_wing())
            .map(|c| c.alpha_stall_pos - 2.5f64.to_radians())
            .unwrap_or(0.25);
        self.diag = Diagnostics {
            atm: atm_new,
            tas: tas_new,
            ias_true,
            ias: ias_display,
            mach: tas_new / atm_new.speed_of_sound,
            aoa,
            sideslip,
            nz,
            agl: s.pos.y - gh_new,
            ground_height: gh_new,
            vertical_speed: s.vel.y,
            ground_speed: DVec3::new(s.vel.x, 0.0, s.vel.z).length(),
            heading,
            pitch,
            roll,
            mass,
            wind: wind_new,
            in_cloud,
            visibility: self.weather.visibility(s.pos, gh_new),
            icing_intensity: weather::icing_intensity(&icing),
            stall_warning: aoa > wing_stall && !self.state.on_ground,
            warnings,
            aero: aero_out,
            gear: gear_out,
            touchdown,
        };
        let _ = tas;
    }
}
