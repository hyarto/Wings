//! 复古像素航图 / 大地图（按 M 切换）：查看当前星球地形、所有生成机场、选择起飞机场、调整日期（季节）与时间。
//! 地形图在线程池异步生成（日期变化时重新着色：雪线随季节移动）。

use crate::cockpit_ui::{draw_map, MapMarker, MapView, MAP_H};
use crate::pixel_pipeline::{HudBuffer, W};
use crate::{GameState, Screen};
use bevy::prelude::*;
use bevy::tasks::futures_lite::future;
use bevy::tasks::{AsyncComputeTaskPool, Task};
use world_gen::climate;
use world_gen::coords::*;
use world_gen::World;

const MAP_W: usize = W as usize;
const MONTH_START: [(f64, &str); 12] = [
    (0.0, "JAN"), (31.0, "FEB"), (59.0, "MAR"), (90.0, "APR"), (120.0, "MAY"), (151.0, "JUN"),
    (181.0, "JUL"), (212.0, "AUG"), (243.0, "SEP"), (273.0, "OCT"), (304.0, "NOV"), (334.0, "DEC"),
];

pub fn date_string(day: f64) -> String {
    let d = day.rem_euclid(365.0);
    let (start, name) = MONTH_START.iter().rev().find(|(s, _)| d >= *s).copied().unwrap();
    format!("{} {:02}", name, (d - start) as i32 + 1)
}

pub fn project(x: f64, z: f64) -> (i32, i32) {
    let px = ((wrap_x(x) + WORLD_WIDTH / 2.0) / WORLD_WIDTH * MAP_W as f64) as i32;
    let py = ((clamp_z(z) + WORLD_HEIGHT / 2.0) / WORLD_HEIGHT * MAP_H as f64) as i32;
    (px.clamp(0, MAP_W as i32 - 1), py.clamp(0, MAP_H - 1))
}

pub fn unproject(px: i32, py: i32) -> (f64, f64) {
    (
        (px as f64 + 0.5) / MAP_W as f64 * WORLD_WIDTH - WORLD_WIDTH / 2.0,
        (py as f64 + 0.5) / MAP_H as f64 * WORLD_HEIGHT - WORLD_HEIGHT / 2.0,
    )
}

/// 生成 320×MAP_H 的 RGB 航图（含山体光影与季节性积雪）。
pub fn render_map_terrain(world: &World, day: f64) -> Vec<[u8; 3]> {
    let (w, h) = (MAP_W, MAP_H as usize);
    let mut hs = vec![0.0f64; w * h];
    for py in 0..h {
        for px in 0..w {
            let (x, z) = unproject(px as i32, py as i32);
            hs[py * w + px] = world.height(x, z);
        }
    }
    let at = |x: i64, y: i64| hs[(y.clamp(0, h as i64 - 1) as usize) * w + (x.rem_euclid(w as i64) as usize)];
    let mut out = vec![[0u8; 3]; w * h];
    for py in 0..h {
        let (_, lat_z) = unproject(0, py as i32);
        let (lat, _) = world_to_latlon(0.0, lat_z);
        for px in 0..w {
            let (x, z) = unproject(px as i32, py as i32);
            let hgt = hs[py * w + px];
            let (dx, dy) = ((at(px as i64 + 1, py as i64) - at(px as i64 - 1, py as i64)), (at(px as i64, py as i64 + 1) - at(px as i64, py as i64 - 1)));
            let slope = (dx.hypot(dy) / 8000.0).atan();
            let ocean = hgt <= 0.0;
            let t_sl = climate::sea_level_temp(lat, day, ocean);
            let hum = climate::humidity(lat, world.terrain.humidity_noise(x, z));
            let b = climate::classify(hgt, slope * 3.0, t_sl, hum, climate::snow_line(lat, day));
            let mut c = climate::biome_color(b);
            // 山体光影（西北光源）与海深渐变
            let shade = if ocean { 0.75 + (hgt / 4500.0).max(-1.0) as f32 * 0.35 * -1.0 * -1.0 } else { 1.0 + ((dx - dy) / 2600.0).clamp(-0.45, 0.45) as f32 };
            for v in &mut c {
                *v = (*v * shade).clamp(0.0, 1.0);
            }
            out[py * w + px] = [(c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8];
        }
    }
    out
}

#[derive(Resource)]
pub struct MapState {
    cursor: (i32, i32),
    selected: Option<usize>,
    terrain: Option<Vec<[u8; 3]>>,
    terrain_day: f64,
    task: Option<(f64, Task<Vec<[u8; 3]>>)>,
    markers: Vec<MapMarker>,
}

fn setup_map(mut commands: Commands, game: Res<GameState>) {
    let markers = game
        .world
        .airports()
        .iter()
        .map(|a| {
            let (px, py) = project(a.center.0, a.center.1);
            let kind = match a.kind {
                world_gen::airport::AirportKind::Dirt => 0,
                world_gen::airport::AirportKind::Regional => 1,
                world_gen::airport::AirportKind::International => 2,
            };
            MapMarker { px, py, kind }
        })
        .collect();
    let p = game.sim.state.rigid.pos;
    commands.insert_resource(MapState { cursor: project(p.x, p.z), selected: None, terrain: None, terrain_day: -999.0, task: None, markers });
}

fn map_system(keys: Res<ButtonInput<KeyCode>>, time: Res<Time>, mut game: ResMut<GameState>, mut map: ResMut<MapState>, mut hud: ResMut<HudBuffer>) {
    let dt = time.delta_secs();
    let shift = keys.pressed(KeyCode::ShiftLeft);
    let speed = if shift { 150.0 } else { 55.0 };
    let (mut mx, mut my) = (0.0f32, 0.0f32);
    for (k, dx, dy) in [(KeyCode::ArrowLeft, -1.0, 0.0), (KeyCode::ArrowRight, 1.0, 0.0), (KeyCode::ArrowUp, 0.0, -1.0), (KeyCode::ArrowDown, 0.0, 1.0), (KeyCode::KeyA, -1.0, 0.0), (KeyCode::KeyD, 1.0, 0.0), (KeyCode::KeyW, 0.0, -1.0), (KeyCode::KeyS, 0.0, 1.0)] {
        if keys.pressed(k) {
            mx += dx;
            my += dy;
        }
    }
    let (cx, cy) = (map.cursor.0 as f32 + mx * speed * dt, map.cursor.1 as f32 + my * speed * dt);
    // 浮点累积：用子像素缓存太复杂，这里以 1 像素步进并限速
    if mx != 0.0 || my != 0.0 {
        let step = (speed * dt).max(1.0);
        let nx = map.cursor.0 as f32 + mx.signum() * if mx != 0.0 { step } else { 0.0 };
        let ny = map.cursor.1 as f32 + my.signum() * if my != 0.0 { step } else { 0.0 };
        map.cursor = ((nx as i32).clamp(0, MAP_W as i32 - 1), (ny as i32).clamp(0, MAP_H - 1));
    }
    let _ = (cx, cy);

    // 日期 / 时间
    let day_step = if shift { 30.0 } else { 5.0 };
    if keys.just_pressed(KeyCode::BracketLeft) {
        game.day = (game.day - day_step).rem_euclid(365.0);
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        game.day = (game.day + day_step).rem_euclid(365.0);
    }
    let hour_step = if shift { 6.0 } else { 1.0 };
    if keys.just_pressed(KeyCode::Comma) {
        game.hour = (game.hour - hour_step).rem_euclid(24.0);
    }
    if keys.just_pressed(KeyCode::Period) {
        game.hour = (game.hour + hour_step).rem_euclid(24.0);
    }
    if keys.just_pressed(KeyCode::KeyC) {
        game.cycle_weather();
    }

    // 选择最近的机场（光标 9 像素内）
    let cur = map.cursor;
    map.selected = map
        .markers
        .iter()
        .enumerate()
        .map(|(i, m)| (i, ((m.px - cur.0).pow(2) + (m.py - cur.1).pow(2)) as f32))
        .filter(|(_, d)| *d <= 81.0)
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|(i, _)| i);

    if keys.just_pressed(KeyCode::Enter) {
        if let Some(i) = map.selected {
            game.depart(i);
            game.screen = Screen::Flight;
        }
    }
    if keys.just_pressed(KeyCode::Escape) {
        game.screen = Screen::Flight;
    }

    // 异步生成地形图（日期变化 → 重新着色）
    if let Some((d, task)) = map.task.as_mut() {
        if let Some(img) = future::block_on(future::poll_once(task)) {
            let d = *d;
            map.terrain = Some(img);
            map.terrain_day = d;
            map.task = None;
        }
    }
    if map.task.is_none() && (map.terrain_day - game.day).abs() > 0.5 {
        let world = game.world.clone();
        let day = game.day;
        map.task = Some((day, AsyncComputeTaskPool::get().spawn(async move { render_map_terrain(&world, day) })));
    }

    // 绘制
    let p = game.sim.state.rigid.pos;
    let (ax, ay) = project(p.x, p.z);
    let (_, _, hdg) = (0, 0, game.sim.diag.heading);
    let ap_line = match map.selected {
        Some(i) => {
            let a = &game.world.airports()[i];
            format!("{} {} RWY {} {:.0}M ELEV {:.0}M{}", a.icao, a.name.to_uppercase(), a.runway_name(), a.length, a.elevation, if a.ils.is_empty() { "" } else { " ILS" })
        }
        None => "ARROWS: CURSOR   ENTER: DEPART   M/ESC: BACK".to_string(),
    };
    let status0 = format!("{} {:02.0}:00 {} {}", date_string(game.day), game.hour, crate::weather_fx::PRESET_NAMES[game.preset % 6], ap_line.chars().take(0).collect::<String>());
    let status1 = if map.selected.is_some() { ap_line } else { "[ ] DAY  , . HOUR  C WEATHER  ARROWS CURSOR".to_string() };
    let view = MapView {
        terrain: map.terrain.as_deref(),
        markers: &map.markers,
        selected: map.selected,
        cursor: map.cursor,
        aircraft: Some((ax, ay, hdg)),
        status: [status0, status1],
        time: time.elapsed_secs_f64(),
    };
    draw_map(&mut hud.0, &view);
}

pub struct MapPlugin;
impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostStartup, setup_map).add_systems(Update, map_system.run_if(|g: Res<GameState>| g.screen == Screen::Map));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projection_roundtrip_and_dates() {
        let (px, py) = project(0.0, 0.0);
        assert!((px - 160).abs() <= 1 && (py - 70).abs() <= 1);
        let (x, z) = unproject(px, py);
        assert_eq!(project(x, z), (px, py));
        assert_eq!(date_string(0.0), "JAN 01");
        assert_eq!(date_string(196.0), "JUL 16");
    }
}
