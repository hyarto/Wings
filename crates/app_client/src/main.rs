//! STRATO-32 客户端：Bevy 320×160 像素风复古飞行模拟器。
//!
//! 运行：`cargo run -p app_client --release -- [--seed N] [--icao XXXX] [--day D] [--hour H]`
//! 开发期加速：`cargo run -p app_client --features dev`

mod aircraft_mesh;
mod airport_visuals;
mod audio;
mod cockpit_ui;
mod input;
mod map_screen;
mod mesh_util;
mod origin;
mod pixel_pipeline;
mod terrain_mesh;
mod weather_fx;

#[cfg(test)]
mod headless_tests;

use bevy::camera::{Camera, Viewport};
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy::window::WindowPlugin;
use cockpit_ui::{FlightHud, IlsHud, PixelBuffer, ViewMode};
use origin::{sync_anchors, FloatingOrigin};
use pixel_pipeline::{HudBuffer, SceneCamera, COCKPIT_VIEW_H, H, W};
use sim_core::aircraft::AircraftConfig;
use sim_core::gear::{Ground, GroundSample, SurfaceType};
use sim_core::integrator::Simulation;
use sim_core::{FT, KT};
use std::sync::Arc;
use world_gen::climate;
use world_gen::coords::world_to_latlon;
use world_gen::World;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Screen {
    Flight,
    Map,
}

/// 把 `world_gen::World` 适配为 sim_core 的地面查询接口（含机场跑道地表）。
pub struct WorldGround {
    pub world: Arc<World>,
    pub day: f64,
}

impl Ground for WorldGround {
    fn sample(&self, x: f64, z: f64) -> GroundSample {
        let h = self.world.height(x, z);
        if h <= 0.0 {
            return GroundSample { height: 0.0, normal: glam::DVec3::Y, surface: SurfaceType::Water };
        }
        let e = 2.0;
        let (hx, hz) = (self.world.height(x + e, z), self.world.height(x, z + e));
        let normal = glam::DVec3::new(-(hx - h) / e, 1.0, -(hz - h) / e).normalize();
        let mut surface = None;
        if let Some(ap) = self.world.airport_near(x, z) {
            let (along, lat) = ap.runway_local(x, z);
            if along > -15.0 && along < ap.length + 15.0 && lat.abs() <= ap.width / 2.0 + 3.0 {
                surface = Some(if ap.asphalt { SurfaceType::Asphalt } else { SurfaceType::Dirt });
            }
        }
        let surface = surface.unwrap_or_else(|| {
            let (lat, _) = world_to_latlon(x, z);
            if climate::sea_level_temp(lat, self.day, false) - 0.0065 * h < -1.0 {
                SurfaceType::Snow
            } else {
                SurfaceType::Grass
            }
        });
        GroundSample { height: h, normal, surface }
    }
}

#[derive(Resource)]
pub struct GameState {
    pub sim: Simulation,
    pub world: Arc<World>,
    pub day: f64,
    pub hour: f64,
    pub view: ViewMode,
    pub screen: Screen,
    pub paused: bool,
    pub airport: usize,
    pub preset: usize,
    pub flap_idx: usize,
    pub time: f64,
    pub flyby: Option<DVec3>,
    pub td_played: f64,
}

impl GameState {
    pub fn new(seed: u64, icao: Option<&str>, day: f64, hour: f64) -> Self {
        let world = Arc::new(World::new(seed));
        let aps = world.airports();
        let idx = icao
            .and_then(|c| aps.iter().position(|a| a.icao.eq_ignore_ascii_case(c)))
            .or_else(|| aps.iter().position(|a| a.kind == world_gen::airport::AirportKind::International && a.lat.abs() < 55.0))
            .unwrap_or(0);
        let ap = &aps[idx];
        let cfg = AircraftConfig::by_code("SKY-172").expect("SKY-172");
        let sim = Simulation::new(cfg, Box::new(WorldGround { world: world.clone(), day }), (ap.center.0, ap.center.1), ap.heading0);
        let mut g = Self {
            sim,
            world: world.clone(),
            day,
            hour,
            view: ViewMode::Cockpit,
            screen: Screen::Flight,
            paused: false,
            airport: idx,
            preset: 0,
            flap_idx: 0,
            time: 0.0,
            flyby: None,
            td_played: 0.0,
        };
        g.depart(idx);
        g
    }

    /// 从指定机场起飞：停在迎风一端的跑道入口，朝向跑道方向，天气按当前预设。
    pub fn depart(&mut self, idx: usize) {
        let ap = self.world.airports()[idx].clone();
        let (from, _) = climate::prevailing_wind(ap.lat);
        let diff = |h: f64| ((h - from + 540.0) % 360.0 - 180.0).abs();
        let end = if diff(ap.heading_of_end(0)) <= diff(ap.heading_of_end(1)) { 0 } else { 1 };
        let t = ap.ends[end];
        let (dx, dz) = ap.landing_dir(end);
        let pos = (t.0 + dx * 14.0, t.1 + dz * 14.0);
        let cfg = AircraftConfig::by_code("SKY-172").expect("SKY-172");
        let mut sim = Simulation::new(cfg, Box::new(WorldGround { world: self.world.clone(), day: self.day }), pos, ap.heading_of_end(end));
        weather_fx::apply_preset(&mut sim, self.preset, ap.lat, self.day, Some(ap.heading_of_end(end)));
        sim.controls.parking_brake = true;
        self.sim = sim;
        self.airport = idx;
        self.flap_idx = 0;
        self.flyby = None;
        self.td_played = 0.0;
    }

    pub fn respawn(&mut self) {
        let i = self.airport;
        self.depart(i);
    }

    pub fn cycle_weather(&mut self) {
        self.preset = (self.preset + 1) % weather_fx::PRESET_NAMES.len();
        let r = self.sim.state.rigid;
        let (lat, _) = world_to_latlon(r.pos.x, r.pos.z);
        let hdg = self.sim.diag.heading;
        weather_fx::apply_preset(&mut self.sim, self.preset, lat, self.day, Some(hdg));
    }
}

fn step_sim(mut game: ResMut<GameState>, time: Res<Time>) {
    if game.screen == Screen::Flight && !game.paused {
        let dt = time.delta_secs_f64().min(0.1);
        game.sim.advance(dt);
        game.time += dt;
    }
}

pub fn update_origin(game: Res<GameState>, mut origin: ResMut<FloatingOrigin>) {
    let p = game.sim.state.rigid.pos;
    origin.0 = DVec3::new(p.x, p.y, p.z);
}

fn camera_system(
    mut game: ResMut<GameState>,
    origin: Res<FloatingOrigin>,
    mut cam: Query<(&mut Transform, &mut Camera, &mut Projection), With<SceneCamera>>,
) {
    let Ok((mut t, mut camera, mut proj)) = cam.single_mut() else { return };
    let r = game.sim.state.rigid;
    let q = Quat::from_xyzw(r.q.x as f32, r.q.y as f32, r.q.z as f32, r.q.w as f32);
    let (heading, pitch) = (game.sim.diag.heading as f32, game.sim.diag.pitch as f32);
    let (fov, viewport) = match game.view {
        ViewMode::Cockpit => (40f32, Some(UVec2::new(W, COCKPIT_VIEW_H))),
        _ => (50f32, None),
    };
    match game.view {
        ViewMode::Cockpit | ViewMode::Hud => {
            t.translation = q * Vec3::new(0.0, 0.62, -0.55);
            t.rotation = q;
        }
        ViewMode::Chase => {
            let yaw = Quat::from_rotation_y(-heading.to_radians());
            let pit = Quat::from_rotation_x((pitch * 0.4).to_radians());
            t.translation = yaw * pit * Vec3::new(0.0, 3.5, 15.0);
            t.look_at(Vec3::new(0.0, 0.6, 0.0), Vec3::Y);
        }
        ViewMode::Flyby => {
            let v = DVec3::new(r.vel.x, r.vel.y, r.vel.z);
            let ac = DVec3::new(r.pos.x, r.pos.y, r.pos.z);
            let need = match game.flyby {
                None => true,
                Some(a) => (a - ac).length() > 700.0 || (v.length() > 5.0 && (a - ac).dot(v) < -50.0),
            };
            if need {
                let dir = if v.length() > 5.0 { v.normalize() } else { DVec3::NEG_Z };
                let side = DVec3::new(-dir.z, 0.0, dir.x).normalize_or_zero();
                let mut a = ac + dir * (v.length().max(30.0) * 5.0) + side * 45.0;
                let gh = game.world.height(a.x, a.z).max(0.0);
                a.y = a.y.max(gh + 6.0);
                game.flyby = Some(a);
            }
            let a = game.flyby.unwrap();
            t.translation = (a - origin.0).as_vec3();
            t.look_at(Vec3::ZERO, Vec3::Y);
        }
    }
    if let Projection::Perspective(p) = &mut *proj {
        if (p.fov - fov.to_radians()).abs() > 1e-4 {
            p.fov = fov.to_radians();
        }
    }
    let want = viewport.map(|s| Viewport { physical_position: UVec2::ZERO, physical_size: s, ..default() });
    let same = match (&camera.viewport, &want) {
        (None, None) => true,
        (Some(a), Some(b)) => a.physical_size == b.physical_size,
        _ => false,
    };
    if !same {
        camera.viewport = want;
    }
}

fn hud_system(game: Res<GameState>, mut hud: ResMut<HudBuffer>) {
    let s = &game.sim;
    let d = &s.diag;
    let st = &s.state;
    let r = &st.rigid;
    let eng = st.engines.first().copied().unwrap_or_default();
    // 飞行航迹矢量
    let vb = r.q.inverse() * r.vel;
    let fwd = (-vb.z).max(0.1);
    let fpv = (vb.x.atan2(fwd).to_degrees(), vb.y.atan2(fwd).to_degrees());

    // ILS：最近的带 ILS 机场
    let mut ils: Option<IlsHud> = None;
    let mut best = f64::MAX;
    for ap in game.world.airports().iter().filter(|a| !a.ils.is_empty()) {
        let dist = world_gen::coords::wrapped_distance(ap.center, (r.pos.x, r.pos.z));
        if dist > 45_000.0 || dist > best {
            continue;
        }
        let mut pick: Option<(f64, IlsHud)> = None;
        for i in &ap.ils {
            let dev = ap.ils_deviation(i, r.pos.x, r.pos.z, r.pos.y);
            let score = dev.loc_deg.abs() + if dev.loc_valid { 0.0 } else { 1000.0 };
            if pick.as_ref().map_or(true, |p| score < p.0) {
                pick = Some((score, IlsHud { ident: i.ident.clone(), loc_deg: dev.loc_deg, gs_deg: dev.gs_deg, loc_valid: dev.loc_valid, gs_valid: dev.gs_valid, dist_km: dev.dist_to_threshold / 1000.0 }));
            }
        }
        if let Some((_, h)) = pick {
            if h.loc_valid {
                best = dist;
                ils = Some(h);
            }
        }
    }
    let nav_text = game
        .world
        .nearest_airport(r.pos.x, r.pos.z)
        .map(|(a, dist)| format!("{} {:.0}K", a.icao, dist / 1000.0))
        .unwrap_or_default();

    let dmg = &st.damage;
    let damage = if dmg.crashed {
        Some("CRASH".to_string())
    } else if dmg.left_wing_failed || dmg.right_wing_failed {
        Some("WING FAILURE".to_string())
    } else if dmg.gear_collapsed {
        Some("GEAR COLLAPSED".to_string())
    } else if dmg.hard_landing {
        Some("HARD LANDING".to_string())
    } else if dmg.flaps_jammed.is_some() {
        Some("FLAPS JAMMED".to_string())
    } else {
        None
    };
    let precip = s.weather.precip_at(r.pos);
    let engine_label = match s.config.engines.first().map(|e| e.kind) {
        Some(sim_core::aircraft::EngineKind::Turbofan { .. }) => "N1",
        _ => "RPM",
    };
    let h = FlightHud {
        ias_kt: d.ias / KT,
        ias_failed: false,
        gs_kt: d.ground_speed / KT,
        alt_ft: r.pos.y / FT,
        vs_fpm: d.vertical_speed / FT * 60.0,
        agl_ft: (d.agl - s.config.rest_cg_height) / FT,
        heading: d.heading,
        pitch: d.pitch,
        roll: d.roll,
        aoa: d.aoa.to_degrees(),
        nz: d.nz,
        throttle: s.controls.throttle,
        rpm: eng.rpm,
        fuel_kg: st.fuel_kg,
        flaps_deg: st.surfaces.flaps * 30.0,
        gear_down: st.gear_pos > 0.5,
        trim: s.controls.pitch_trim,
        brake: s.controls.brake_left > 0.1,
        parking: s.controls.parking_brake,
        stall: d.stall_warning,
        ice: st.ice_load,
        ovspd: d.warnings.overspeed || d.warnings.flap_overspeed,
        glim: d.warnings.g_limit,
        gear_warn: d.warnings.gear_overspeed || (st.gear_pos < 0.5 && d.agl < 150.0 && d.vertical_speed < -1.0),
        ils,
        nav_text,
        fpv,
        damage,
        frost: st.ice_load.min(1.0) as f64,
        rain: if s.weather.precip == sim_core::weather::PrecipKind::Rain { precip } else { 0.0 },
        time: game.time,
        code: s.config.code.to_string(),
        engine_label,
    };
    cockpit_ui::draw_flight(&mut hud.0, &h, game.view);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let seed: u64 = arg("--seed").and_then(|s| s.parse().ok()).unwrap_or(32);
    let day: f64 = arg("--day").and_then(|s| s.parse().ok()).unwrap_or(120.0);
    let hour: f64 = arg("--hour").and_then(|s| s.parse().ok()).unwrap_or(10.0);
    let icao = arg("--icao");
    let game = GameState::new(seed, icao.as_deref(), day, hour);
    println!("STRATO-32  seed {seed}  {} airports  start {}", game.world.airports().len(), game.world.airports()[game.airport].icao);

    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window { title: "STRATO-32".into(), resolution: (960, 480).into(), ..default() }),
                    ..default()
                })
                .set(ImagePlugin::default_nearest()),
        )
        .add_plugins((
            pixel_pipeline::PixelPipelinePlugin,
            origin::OriginPlugin,
            terrain_mesh::TerrainPlugin,
            airport_visuals::AirportVisualsPlugin,
            aircraft_mesh::AircraftMeshPlugin,
            weather_fx::WeatherFxPlugin,
            audio::AudioPlugin,
            map_screen::MapPlugin,
        ))
        .insert_resource(game)
        .insert_resource(HudBuffer(PixelBuffer::new()))
        .add_systems(
            Update,
            (input::read_input, step_sim, update_origin, sync_anchors, camera_system, hud_system.run_if(|g: Res<GameState>| g.screen == Screen::Flight)).chain(),
        )
        .run();
    let _ = (H, W);
}
