//! 天气与环境视觉：动态雾（能见度）、多层低模云、3D 雨雪粒子、太阳/天空/环境光。

use crate::mesh_util::MeshBuilder;
use crate::origin::{FloatingOrigin, WorldAnchor};
use crate::pixel_pipeline::SceneCamera;
use crate::GameState;
use bevy::math::DVec3;
use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;
use sim_core::integrator::Simulation;
use sim_core::weather::{CloudLayer, PrecipKind, ShearLayer};
use std::collections::HashMap;
use world_gen::climate;

pub const PRESET_NAMES: [&str; 6] = ["CLEAR", "RAIN", "SNOW", "FOG", "ICING", "STORM"];

/// 应用天气预设。`runway_heading` 用于让风与跑道对齐/形成侧风。
pub fn apply_preset(sim: &mut Simulation, preset: usize, lat: f64, day: f64, runway_heading: Option<f64>) {
    let w = &mut sim.weather;
    *w = sim_core::weather::WeatherState::calm();
    w.seed = 1 + preset as u32 * 7;
    let base_delta = climate::isa_delta_t(lat, day, false);
    w.atmosphere.delta_t = base_delta;
    let (prev_from, prev_speed) = climate::prevailing_wind(lat);
    let from = runway_heading.unwrap_or(prev_from);
    match preset % PRESET_NAMES.len() {
        0 => {
            w.set_surface_wind(from, prev_speed.min(6.0));
            w.set_wind_aloft(prev_from, prev_speed * 2.5);
            w.turbulence = 0.15;
        }
        1 => {
            w.set_surface_wind(from, 8.0);
            w.set_wind_aloft(prev_from, 18.0);
            w.gust = 4.0;
            w.turbulence = 0.4;
            w.clouds.push(CloudLayer { base: 600.0, top: 3500.0, coverage: 1.0 });
            w.precip = PrecipKind::Rain;
            w.precip_intensity = 0.6;
            w.fog_visibility = 6000.0;
            w.atmosphere.delta_t = base_delta.max(-6.0).min(8.0);
        }
        2 => {
            w.set_surface_wind(from, 6.0);
            w.set_wind_aloft(prev_from, 14.0);
            w.turbulence = 0.3;
            w.clouds.push(CloudLayer { base: 500.0, top: 3000.0, coverage: 1.0 });
            w.precip = PrecipKind::Snow;
            w.precip_intensity = 0.7;
            w.fog_visibility = 2500.0;
            w.atmosphere.delta_t = base_delta.min(-16.0);
        }
        3 => {
            w.set_surface_wind(from, 2.0);
            w.fog_visibility = 350.0;
        }
        4 => {
            w.set_surface_wind(from, 5.0);
            w.set_wind_aloft(prev_from, 12.0);
            w.turbulence = 0.3;
            w.clouds.push(CloudLayer { base: 1200.0, top: 4500.0, coverage: 1.0 });
            w.atmosphere.delta_t = -10.0;
        }
        _ => {
            // 侧风风暴：与跑道成 70° 夹角
            let cross = (from + 70.0).rem_euclid(360.0);
            w.set_surface_wind(cross, 14.0);
            w.set_wind_aloft(cross, 24.0);
            w.gust = 8.0;
            w.turbulence = 0.8;
            w.clouds.push(CloudLayer { base: 1500.0, top: 3500.0, coverage: 0.6 });
            w.shear_layers.push(ShearLayer { altitude: 300.0 + 0.0, thickness: 120.0, delta: DVec3::new(6.0, 0.0, 6.0).to_array().into() });
            w.fog_visibility = 9000.0;
        }
    }
}

#[derive(Resource, Default)]
pub struct WeatherFx {
    sig: u64,
    clouds: HashMap<(i32, i32, i32), Entity>,
}

#[derive(Resource)]
pub struct FxAssets {
    cloud_mesh: Handle<Mesh>,
    cloud_mat: Handle<StandardMaterial>,
    rain_mesh: Handle<Mesh>,
    snow_mesh: Handle<Mesh>,
    rain_mat: Handle<StandardMaterial>,
    snow_mat: Handle<StandardMaterial>,
}

#[derive(Component)]
pub struct Particle {
    off: Vec3,
}
#[derive(Component)]
pub struct Cloud;

const N_PARTICLES: usize = 170;
const BOX: Vec3 = Vec3::new(20.0, 12.0, 20.0);

fn rnd(seed: u32) -> f32 {
    let mut x = seed.wrapping_mul(747796405).wrapping_add(2891336453);
    x = ((x >> ((x >> 28) + 4)) ^ x).wrapping_mul(277803737);
    ((x >> 22) ^ x) as f32 / u32::MAX as f32
}

fn setup_fx(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mut cb = MeshBuilder::new();
    cb.blob(Vec3::ZERO, Vec3::ONE, [0.96, 0.96, 0.98, 1.0]);
    let cloud_mesh = meshes.add(cb.build());
    let cloud_mat = mats.add(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 1.0, reflectance: 0.0, ..default() });
    let rain_mesh = meshes.add(Cuboid::new(0.025, 0.025, 0.9));
    let snow_mesh = meshes.add(Cuboid::new(0.09, 0.09, 0.09));
    let rain_mat = mats.add(StandardMaterial { base_color: Color::srgb(0.7, 0.8, 1.0), unlit: true, ..default() });
    let snow_mat = mats.add(StandardMaterial { base_color: Color::WHITE, unlit: true, ..default() });
    for i in 0..N_PARTICLES {
        let off = Vec3::new((rnd(i as u32 * 3) * 2.0 - 1.0) * BOX.x, (rnd(i as u32 * 3 + 1) * 2.0 - 1.0) * BOX.y, (rnd(i as u32 * 3 + 2) * 2.0 - 1.0) * BOX.z);
        commands.spawn((Mesh3d(rain_mesh.clone()), MeshMaterial3d(rain_mat.clone()), Transform::from_translation(off), Visibility::Hidden, Particle { off }));
    }
    commands.insert_resource(FxAssets { cloud_mesh, cloud_mat, rain_mesh, snow_mesh, rain_mat, snow_mat });
    commands.insert_resource(WeatherFx::default());
}

fn weather_signature(sim: &Simulation) -> u64 {
    let w = &sim.weather;
    let mut h = w.seed as u64;
    for c in &w.clouds {
        h = h.wrapping_mul(31).wrapping_add(c.base as u64 * 7 + c.top as u64 + (c.coverage * 100.0) as u64);
    }
    h
}

fn manage_clouds(mut commands: Commands, game: Res<GameState>, assets: Option<Res<FxAssets>>, mut fx: ResMut<WeatherFx>, mut tick: Local<f32>, time: Res<Time>) {
    let Some(assets) = assets else { return };
    *tick -= time.delta_secs();
    let sig = weather_signature(&game.sim);
    if sig != fx.sig {
        for (_, e) in fx.clouds.drain() {
            commands.entity(e).despawn();
        }
        fx.sig = sig;
        *tick = 0.0;
    }
    if *tick > 0.0 {
        return;
    }
    *tick = 0.5;
    let ac = game.sim.state.rigid.pos;
    let cs = 2400.0;
    let (ci, cj) = ((ac.x / cs).floor() as i32, (ac.z / cs).floor() as i32);
    let far: Vec<(i32, i32, i32)> = fx.clouds.keys().copied().filter(|k| (k.1 - ci).abs() > 8 || (k.2 - cj).abs() > 8).collect();
    for k in far {
        if let Some(e) = fx.clouds.remove(&k) {
            commands.entity(e).despawn();
        }
    }
    for (li, layer) in game.sim.weather.clouds.iter().enumerate() {
        let mid = 0.5 * (layer.base + layer.top);
        for dj in -6..=6 {
            for di in -6..=6 {
                let key = (li as i32, ci + di, cj + dj);
                if fx.clouds.contains_key(&key) {
                    continue;
                }
                let seed = (key.1.wrapping_mul(73856093) ^ key.2.wrapping_mul(19349663) ^ key.0.wrapping_mul(83492791)) as u32;
                let jx = (rnd(seed) - 0.5) * cs * 0.7;
                let jz = (rnd(seed ^ 0x55) - 0.5) * cs * 0.7;
                let pos = DVec3::new((key.1 as f64 + 0.5) * cs + jx as f64, mid, (key.2 as f64 + 0.5) * cs + jz as f64);
                if game.sim.weather.cloud_density(bevy_to_glam(pos)) < 0.25 {
                    continue;
                }
                let thick = ((layer.top - layer.base) * 0.42).clamp(60.0, 420.0) as f32;
                let r = cs as f32 * (0.5 + 0.35 * rnd(seed ^ 0x99));
                let e = commands
                    .spawn((
                        Mesh3d(assets.cloud_mesh.clone()),
                        MeshMaterial3d(assets.cloud_mat.clone()),
                        Transform::from_scale(Vec3::new(r, thick, r)).with_rotation(Quat::from_rotation_y(rnd(seed ^ 0x77) * 6.28)),
                        WorldAnchor(DVec3::new(pos.x, layer.base + thick as f64 * 0.7, pos.z)),
                        Cloud,
                    ))
                    .id();
                fx.clouds.insert(key, e);
            }
        }
    }
}

fn bevy_to_glam(v: DVec3) -> sim_core_glam::DVec3 {
    sim_core_glam::DVec3::new(v.x, v.y, v.z)
}
mod sim_core_glam {
    pub use glam::DVec3;
}

/// 雨/雪粒子：相对机体坐标系内循环，运动方向 = 机体相对空气的反向 + 下落速度。
fn update_particles(
    game: Res<GameState>,
    time: Res<Time>,
    assets: Option<Res<FxAssets>>,
    cam: Query<&Transform, (With<SceneCamera>, Without<Particle>)>,
    mut q: Query<(&mut Particle, &mut Transform, &mut Visibility, &mut Mesh3d, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let Some(assets) = assets else { return };
    let Ok(ct) = cam.single() else { return };
    let w = &game.sim.weather;
    let p = game.sim.state.rigid.pos;
    let intensity = w.precip_at(sim_core_glam::DVec3::new(p.x, p.y, p.z));
    let snow = w.precip == PrecipKind::Snow;
    let v = game.sim.state.rigid.vel;
    let wind = game.sim.diag.wind;
    let fall = if snow { 1.5 } else { 9.0 };
    let rel = Vec3::new((wind.x - v.x) as f32, (wind.y - v.y) as f32 - fall, (wind.z - v.z) as f32);
    let dt = time.delta_secs();
    let active = ((intensity.clamp(0.0, 1.0) * N_PARTICLES as f64) as usize).min(N_PARTICLES);
    for (i, (mut part, mut t, mut vis, mut mesh, mut mat)) in q.iter_mut().enumerate() {
        if intensity <= 0.0 || i >= active || game.view == crate::cockpit_ui::ViewMode::Chase && false {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Visible;
        mesh.0 = if snow { assets.snow_mesh.clone() } else { assets.rain_mesh.clone() };
        mat.0 = if snow { assets.snow_mat.clone() } else { assets.rain_mat.clone() };
        part.off += rel * dt;
        // 回绕到盒内
        for (a, b) in [(0, BOX.x), (1, BOX.y), (2, BOX.z)] {
            let o = part.off[a];
            if o > b {
                part.off[a] = o - 2.0 * b;
            } else if o < -b {
                part.off[a] = o + 2.0 * b;
            }
        }
        t.translation = ct.translation + part.off;
        if !snow && rel.length() > 0.1 {
            t.rotation = Quat::from_rotation_arc(Vec3::Z, rel.normalize());
        }
    }
}

/// 太阳方向（指向太阳的单位向量，世界系：X 东 / Y 上 / Z 南）。
pub fn sun_dir(lat_deg: f64, day: f64, hour: f64) -> Vec3 {
    let decl = climate::solar_declination(day).to_radians();
    let phi = lat_deg.to_radians();
    let ha = ((hour - 12.0) * 15.0).to_radians();
    let sin_el = phi.sin() * decl.sin() + phi.cos() * decl.cos() * ha.cos();
    let east = -decl.cos() * ha.sin();
    let north = decl.sin() * phi.cos() - decl.cos() * phi.sin() * ha.cos();
    Vec3::new(east as f32, sin_el as f32, -north as f32).normalize()
}

fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
}

/// 每帧更新：太阳光、天空色、环境光、距离雾（随能见度）。
pub fn update_environment(
    mut commands: Commands,
    game: Res<GameState>,
    mut sun: Query<(&mut Transform, &mut DirectionalLight)>,
    mut cam: Query<(Entity, &mut Camera), With<SceneCamera>>,
    mut ambient: ResMut<GlobalAmbientLight>,
    origin: Res<FloatingOrigin>,
) {
    let _ = origin;
    let r = &game.sim.state.rigid;
    let (lat, _) = world_gen::coords::world_to_latlon(r.pos.x, r.pos.z);
    let sd = sun_dir(lat, game.day, game.hour);
    let day_f = ((sd.y + 0.12) / 0.4).clamp(0.0, 1.0);
    let overcast = game.sim.weather.clouds.iter().any(|c| c.coverage > 0.9) as i32 as f32;
    let vis = game.sim.diag.visibility.max(50.0) as f32;
    let haze = (1.0 - vis / 25_000.0).clamp(0.0, 1.0);
    let gray_amt = (overcast * 0.65).max(haze * 0.85);
    let (night, dayc, grayc) = ([0.01, 0.015, 0.04], [0.45, 0.66, 0.93], [0.62, 0.64, 0.68]);
    let mut sky = lerp3(night, dayc, day_f);
    // 晨昏暖色
    let dusk = (1.0 - (sd.y.abs() / 0.18).min(1.0)) * 0.5;
    sky = lerp3(sky, [0.95, 0.5, 0.25], dusk * (1.0 - overcast * 0.6));
    let gray = lerp3([0.01, 0.012, 0.02], grayc, day_f);
    let fog_col = lerp3(sky, gray, gray_amt);
    let clear = lerp3(sky, gray, overcast * 0.7);
    if let Ok((mut t, mut l)) = sun.single_mut() {
        *t = Transform::default().looking_to(-sd, Vec3::Y);
        l.illuminance = 300.0 + 11_000.0 * sd.y.max(0.0).sqrt() * (1.0 - 0.55 * overcast);
        l.color = Color::srgb(1.0, 0.95 - dusk * 0.2, 0.88 - dusk * 0.35);
    }
    ambient.brightness = 60.0 + 900.0 * day_f * (1.0 - 0.3 * overcast);
    ambient.color = Color::srgb(0.75 + 0.1 * (1.0 - day_f), 0.8, 1.0);
    if let Ok((e, mut cam)) = cam.single_mut() {
        cam.clear_color = ClearColorConfig::Custom(Color::srgb(clear[0], clear[1], clear[2]));
        commands.entity(e).insert(DistanceFog {
            color: Color::srgb(fog_col[0], fog_col[1], fog_col[2]),
            falloff: FogFalloff::ExponentialSquared { density: 1.9 / vis },
            ..default()
        });
    }
}

pub fn setup_sun(mut commands: Commands) {
    commands.spawn((DirectionalLight { illuminance: 10_000.0, shadows_enabled: false, ..default() }, Transform::default().looking_to(Vec3::new(-0.3, -1.0, -0.4), Vec3::Y)));
}

pub struct WeatherFxPlugin;
impl Plugin for WeatherFxPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup_fx, setup_sun)).add_systems(Update, (manage_clouds, update_particles, update_environment));
    }
}
