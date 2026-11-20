//! 开始菜单（M4）：选机型、出发机场、起飞方式（跑道/水面）、天气、季节（日期）、时间、世界种子。
//!
//! 纯像素 UI（320×160），与航图一样直接写入 `HudBuffer`。Tab 随时打开/关闭。

use crate::airport_visuals::AirportVisuals;
use crate::cockpit_ui::{PixelBuffer, AMBER, BLACK, CYAN, GRAY, GREEN, PANEL, WHITE, YELLOW};
use crate::map_screen::{date_string, make_map_state, MapState};
use crate::pixel_pipeline::{HudBuffer, H, W};
use crate::terrain_mesh::{ChunkManager, TerrainChunk};
use crate::weather_fx::PRESET_NAMES;
use crate::{GameState, Screen};
use bevy::prelude::*;
use sim_core::aircraft::{AircraftConfig, SurfaceKind, CODES};
use sim_core::{KT};

pub const ROWS: usize = 8;
pub const LABELS: [&str; ROWS] = ["AIRCRAFT", "AIRPORT", "START", "WEATHER", "DATE", "TIME", "SEED", ""];

/// 规格表：典型巡航 / 升限。
pub const SPEC: [(&str, &str); 5] = [
    ("SKY-172", "CRUISE 115KT  CEIL 4200M"),
    ("TWIN-6", "CRUISE 160KT  CEIL 7500M"),
    ("JET-39", "CRUISE 380KT  CEIL 11000M"),
    ("LINER-73", "CRUISE 450KT  CEIL 12500M"),
    ("SEA-415", "CRUISE 180KT  CEIL 6000M"),
];

#[derive(Resource, Clone, Debug, PartialEq)]
pub struct MenuState {
    pub row: usize,
    pub aircraft: usize,
    pub airport: usize,
    pub water: bool,
    pub preset: usize,
    pub day: f64,
    pub hour: f64,
    pub seed: u64,
    /// 打开菜单时的世界种子（种子改变后机场序号不再对应）。
    pub seed0: u64,
}

impl MenuState {
    pub fn from_game(g: &GameState) -> Self {
        Self { row: 7, aircraft: g.aircraft, airport: g.airport, water: g.water_start, preset: g.preset, day: g.day, hour: g.hour, seed: g.seed, seed0: g.seed }
    }

    pub fn is_amphibious(&self) -> bool {
        AircraftConfig::by_code(CODES[self.aircraft % CODES.len()]).map_or(false, |c| c.amphibious)
    }

    pub fn move_row(&mut self, dir: i32) {
        self.row = (self.row as i32 + dir).rem_euclid(ROWS as i32) as usize;
    }

    /// 调整当前行的值。`big` = Shift 大步长。
    pub fn adjust(&mut self, dir: i32, big: bool, n_airports: usize) {
        let d = dir.signum();
        match self.row {
            0 => self.aircraft = (self.aircraft as i32 + d).rem_euclid(CODES.len() as i32) as usize,
            1 => {
                if self.seed == self.seed0 && n_airports > 0 {
                    let step = if big { 10 } else { 1 };
                    self.airport = (self.airport as i32 + d * step).rem_euclid(n_airports as i32) as usize;
                }
            }
            2 => {
                if self.is_amphibious() {
                    self.water = !self.water;
                }
            }
            3 => self.preset = (self.preset as i32 + d).rem_euclid(PRESET_NAMES.len() as i32) as usize,
            4 => self.day = (self.day + d as f64 * if big { 30.0 } else { 5.0 }).rem_euclid(365.0),
            5 => self.hour = (self.hour + d as f64 * if big { 6.0 } else { 1.0 }).rem_euclid(24.0),
            6 => self.seed = if d > 0 { self.seed.saturating_add(if big { 100 } else { 1 }) } else { self.seed.saturating_sub(if big { 100 } else { 1 }) },
            _ => {}
        }
    }

    /// 应用菜单选择并出发。种子变化会重新生成星球。
    pub fn apply(&self, game: &mut GameState) {
        game.preset = self.preset;
        game.day = self.day;
        game.hour = self.hour;
        game.aircraft = self.aircraft % CODES.len();
        game.water_start = self.water && self.is_amphibious();
        let mut airport = self.airport;
        if self.seed != game.seed {
            game.set_world(self.seed);
            airport = game.airport;
        }
        airport = airport.min(game.world.airports().len() - 1);
        game.depart(airport);
        game.screen = Screen::Flight;
    }
}

fn airport_label(game: &GameState, m: &MenuState) -> String {
    if m.seed != m.seed0 {
        return "AUTO (NEW WORLD)".to_string();
    }
    let a = &game.world.airports()[m.airport.min(game.world.airports().len() - 1)];
    let k = match a.kind {
        world_gen::airport::AirportKind::Dirt => "DIRT",
        world_gen::airport::AirportKind::Regional => "REG",
        world_gen::airport::AirportKind::International => "INTL",
    };
    let name: String = a.name.to_uppercase().chars().take(11).collect();
    format!("{} {} {}", a.icao, k, name)
}

/// 机型俯视剪影（按真实几何缩放），画在 (cx, cy) 为中心的区域。
pub fn draw_topview(buf: &mut PixelBuffer, cfg: &AircraftConfig, cx: i32, cy: i32, radius: f32, col: [u8; 4]) {
    let htail = cfg.surfaces.iter().find(|s| s.kind == SurfaceKind::HorizontalStab);
    let vtail = cfg.surfaces.iter().find(|s| s.kind == SurfaceKind::VerticalStab);
    let nose_z = cfg.contacts.first().map_or(-3.0, |c| c.pos.z) - 1.5;
    let tail_z = vtail.map_or(5.0, |v| v.pos.z) + 1.0;
    let half_len = ((tail_z - nose_z) * 0.5) as f32;
    let scale = radius / (half_len.max((cfg.wing_span * 0.5) as f32));
    let mid_z = ((tail_z + nose_z) * 0.5) as f32;
    let px = |x: f64| cx + (x as f32 * scale).round() as i32;
    let pz = |z: f64| cy + ((z as f32 - mid_z) * scale).round() as i32;
    // 机身
    let fw = ((cfg.wing_span as f32 * 0.035 * scale).round() as i32).max(1);
    buf.rect(cx - fw, pz(nose_z), fw * 2 + 1, pz(tail_z) - pz(nose_z), col);
    // 机翼
    let wz = cfg.surfaces.iter().find(|s| s.kind.is_wing()).map_or(0.0, |s| s.pos.z);
    let sweep = if cfg.mach_crit > 0.72 && cfg.code == "LINER-73" { 0.42f64 } else { 0.0 };
    let half = cfg.wing_span * 0.5;
    let tip_dz = half * sweep.tan();
    let chord_px = ((cfg.mean_chord as f32 * scale).round() as i32).max(1);
    for s in [-1.0f64, 1.0] {
        buf.line(cx, pz(wz), px(s * half), pz(wz + tip_dz), col);
        buf.line(cx, pz(wz) + chord_px, px(s * half), pz(wz + tip_dz) + (chord_px / 2).max(1), col);
        buf.line(px(s * half), pz(wz + tip_dz), px(s * half), pz(wz + tip_dz) + (chord_px / 2).max(1), col);
    }
    // 平尾
    if let Some(h) = htail {
        let hs = (h.area * h.aspect_ratio).sqrt() * 0.5;
        buf.line(px(-hs), pz(h.pos.z), px(hs), pz(h.pos.z), col);
        buf.line(px(-hs), pz(h.pos.z) + 1, px(hs), pz(h.pos.z) + 1, col);
    }
    // 发动机
    for e in &cfg.engines {
        buf.rect(px(e.pos.x) - 1, pz(e.pos.z), 3, 4.max((scale * 3.0) as i32), GREEN);
    }
    // 浮筒
    for c in cfg.contacts.iter().filter(|c| c.kind == sim_core::aircraft::ContactKind::Float && c.pos.x.abs() > 3.0) {
        buf.rect(px(c.pos.x) - 1, pz(c.pos.z) - 2, 3, 5, CYAN);
    }
}

pub fn draw_menu(buf: &mut PixelBuffer, m: &MenuState, game: &GameState, t: f64) {
    buf.clear(BLACK);
    buf.rect(0, 0, W as i32, 12, PANEL);
    buf.text_center(W as i32 / 2, 3, "STRATO-32  FLIGHT SETUP", AMBER, 1);
    let cfg = AircraftConfig::by_code(CODES[m.aircraft % CODES.len()]).expect("aircraft");
    let values = [
        format!("< {} >", cfg.code),
        format!("< {} >", airport_label(game, m)),
        format!("< {} >", if m.water && m.is_amphibious() { "WATER" } else if m.is_amphibious() { "RUNWAY" } else { "RUNWAY" }),
        format!("< {} >", PRESET_NAMES[m.preset % PRESET_NAMES.len()]),
        format!("< {} >", date_string(m.day)),
        format!("< {:02.0}:00 >", m.hour),
        format!("< {} >", m.seed),
        "FLY!".to_string(),
    ];
    for i in 0..ROWS {
        let y = 18 + i as i32 * 13;
        let sel = i == m.row;
        if sel {
            buf.rect(4, y - 2, 176, 11, PANEL);
            buf.frame(4, y - 2, 176, 11, if (t * 3.0) as i32 % 2 == 0 { AMBER } else { YELLOW });
        }
        if !LABELS[i].is_empty() {
            buf.text(8, y, LABELS[i], if sel { WHITE } else { GRAY }, 1);
            let dim = i == 2 && !m.is_amphibious() || (i == 1 && m.seed != m.seed0 && false);
            buf.text(60, y, &values[i], if dim { GRAY } else if sel { GREEN } else { WHITE }, 1);
        } else {
            buf.text_center(92, y, &values[i], if sel { GREEN } else { AMBER }, 1);
        }
    }
    // 右侧：机型信息 + 俯视剪影
    buf.frame(186, 16, 130, 124, GRAY);
    buf.text_center(251, 20, &cfg.name.to_uppercase(), WHITE, 1);
    buf.text_center(251, 28, cfg.blurb(), CYAN, 1);
    draw_topview(buf, &cfg, 251, 66, 28.0, WHITE);
    let spec = SPEC.iter().find(|s| s.0 == cfg.code).map_or("", |s| s.1);
    buf.text_center(251, 100, spec, AMBER, 1);
    buf.text_center(251, 109, &format!("MTOW {:.0}KG  WING {:.0}M2", cfg.max_takeoff_mass(), cfg.wing_area), GRAY, 1);
    buf.text_center(251, 118, &format!("VNE {:.0}KT  LIM +{:.1}/{:.1}G", cfg.structure.v_ne / KT, cfg.structure.limit_load_pos, cfg.structure.limit_load_neg), GRAY, 1);
    let feat = format!("{}{}{}", if cfg.engines.len() > 1 { "TWIN " } else { "" }, if cfg.has_speedbrake { "SPDBRK " } else { "" }, if cfg.amphibious { "FLOATS" } else if cfg.gear_time > 0.0 { "RETRACT" } else { "FIXED GEAR" });
    buf.text_center(251, 127, &feat, GRAY, 1);
    buf.text_center(W as i32 / 2, H as i32 - 9, "UP/DN ROW  L/R CHANGE  SHIFT BIG  ENTER FLY  TAB/ESC BACK", GRAY, 1);
}

fn menu_system(keys: Res<ButtonInput<KeyCode>>, time: Res<Time>, mut game: ResMut<GameState>, mut menu: ResMut<MenuState>, mut hud: ResMut<HudBuffer>) {
    let pressed = |a: KeyCode, b: KeyCode| keys.just_pressed(a) || keys.just_pressed(b);
    if pressed(KeyCode::ArrowDown, KeyCode::KeyS) {
        menu.move_row(1);
    }
    if pressed(KeyCode::ArrowUp, KeyCode::KeyW) {
        menu.move_row(-1);
    }
    let big = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let n = game.world.airports().len();
    if pressed(KeyCode::ArrowRight, KeyCode::KeyD) {
        menu.adjust(1, big, n);
    }
    if pressed(KeyCode::ArrowLeft, KeyCode::KeyA) {
        menu.adjust(-1, big, n);
    }
    if keys.just_pressed(KeyCode::Enter) {
        let m = menu.clone();
        m.apply(&mut game);
        return;
    }
    if keys.just_pressed(KeyCode::Escape) {
        game.screen = Screen::Flight;
        return;
    }
    draw_menu(&mut hud.0, &menu, &game, time.elapsed_secs_f64());
}

/// 世界种子变化后销毁旧地形 / 机场实体并重建航图状态。
#[allow(clippy::too_many_arguments)]
fn world_reset_system(
    mut commands: Commands,
    game: Res<GameState>,
    mut seen: Local<u32>,
    chunks: Query<Entity, With<TerrainChunk>>,
    mut mgr: ResMut<ChunkManager>,
    mut vis: ResMut<AirportVisuals>,
    mut map: ResMut<MapState>,
) {
    if *seen == game.world_epoch {
        return;
    }
    *seen = game.world_epoch;
    for e in &chunks {
        commands.entity(e).despawn();
    }
    mgr.loaded.clear();
    mgr.pending.clear();
    vis.clear_all(&mut commands);
    *map = make_map_state(&game);
}

pub struct MenuPlugin;
impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostStartup, |mut c: Commands, g: Res<GameState>| c.insert_resource(MenuState::from_game(&g)))
            .add_systems(Update, (menu_system.run_if(|g: Res<GameState>| g.screen == Screen::Menu), world_reset_system));
    }
}
