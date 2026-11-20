//! 地形雷达图（等高线）+ 纵剖面 + 地形告警（TAWS）系统。
//!
//! * `N`：关闭 → 小图 → 大图（含前方地形剖面）循环；`[` / `]`：量程 5 / 10 / 20 / 40 km。
//! * 告警出现时（黄/红）若雷达图关闭，会自动弹出小图并保持到告警解除后 10 s。
//! * 地形图为航向向上；颜色按地形相对本机海拔分级（红：≥ 本机高度 −30 m；琥珀：−30…−300 m；绿：−300…−800 m），
//!   等高线按量程自适应（25/50/100/200 m），每 5 条为计曲线（白色）。

use crate::cockpit_ui::*;
use crate::{GameState, Screen};
use bevy::prelude::*;
use sim_core::taws::{self, TawsAlert, TawsInput, TawsMonitor, TawsStatus};
use sim_core::terrain_map::{self, Band, Cell, TerrainGrid};
use sim_core::FT;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RadarMode {
    #[default]
    Off,
    Small,
    Large,
}

impl RadarMode {
    pub fn next(self) -> Self {
        match self {
            RadarMode::Off => RadarMode::Small,
            RadarMode::Small => RadarMode::Large,
            RadarMode::Large => RadarMode::Off,
        }
    }
}

pub const RANGES_M: [f64; 4] = [5_000.0, 10_000.0, 20_000.0, 40_000.0];
/// 网格半边长 = 量程 × 此系数；网格 n×n。
const GRID_SPAN: f64 = 1.5;
const GRID_N: usize = 160;
const ROWS_PER_FRAME: usize = 24;
const PROFILE_N: usize = 64;
/// 告警解除后自动弹出的雷达图保持时间（s）。
const POPUP_HOLD_S: f64 = 10.0;

#[derive(Resource)]
pub struct TerrainRadar {
    pub mode: RadarMode,
    pub range_idx: usize,
    pub grid: Option<TerrainGrid>,
    building: Option<TerrainGrid>,
    pub monitor: TawsMonitor,
    pub status: TawsStatus,
    /// 经过保持逻辑后的当前告警。
    pub alert: TawsAlert,
    pub popup: f64,
    pub profile: Vec<f32>,
    timer: f64,
    epoch: u32,
}

impl Default for TerrainRadar {
    fn default() -> Self {
        Self {
            mode: RadarMode::Off,
            range_idx: 1,
            grid: None,
            building: None,
            monitor: TawsMonitor::default(),
            status: TawsStatus::default(),
            alert: TawsAlert::None,
            popup: 0.0,
            profile: Vec::new(),
            timer: 1.0,
            epoch: 0,
        }
    }
}

impl TerrainRadar {
    pub fn range(&self) -> f64 {
        RANGES_M[self.range_idx.min(RANGES_M.len() - 1)]
    }

    /// 当前是否显示（手动开启或告警自动弹出）。
    pub fn visible(&self) -> bool {
        self.mode != RadarMode::Off || self.popup > 0.0
    }

    /// 实际显示的模式（自动弹出时为小图）。
    pub fn shown_mode(&self) -> RadarMode {
        if self.mode != RadarMode::Off {
            self.mode
        } else if self.popup > 0.0 {
            RadarMode::Small
        } else {
            RadarMode::Off
        }
    }

    pub fn cycle_mode(&mut self) {
        self.mode = self.mode.next();
        self.popup = 0.0;
    }

    pub fn range_up(&mut self) {
        self.range_idx = (self.range_idx + 1) % RANGES_M.len();
    }

    pub fn range_down(&mut self) {
        self.range_idx = self.range_idx.saturating_sub(1);
    }

    /// 推进网格增量构建；需要时（首次/量程变化/飞机靠近网格边缘）重新开始。
    pub fn update_grid(&mut self, height: &dyn Fn(f64, f64) -> f64, x: f64, z: f64) {
        let range = self.range();
        let half = range * GRID_SPAN;
        let stale = |g: &TerrainGrid| (g.half - half).abs() > 1.0 || g.margin(x, z) < range * 1.1;
        if self.building.is_none() && self.grid.as_ref().map_or(true, stale) {
            self.building = Some(TerrainGrid::new(x, z, half, GRID_N));
        }
        if let Some(b) = self.building.as_mut() {
            // 量程在构建途中被改变：丢弃重来
            if (b.half - half).abs() > 1.0 {
                self.building = Some(TerrainGrid::new(x, z, half, GRID_N));
            }
        }
        if let Some(b) = self.building.as_mut() {
            if b.build_rows(height, ROWS_PER_FRAME) {
                self.grid = self.building.take();
            }
        }
    }

    /// 丢弃缓存（换世界/重新出发）。
    pub fn invalidate(&mut self) {
        self.grid = None;
        self.building = None;
        self.profile.clear();
        self.monitor.reset();
        self.alert = TawsAlert::None;
        self.status = TawsStatus::default();
        self.popup = 0.0;
    }
}

/// 按键：N 切换模式，`[` `]` 切换量程（仅飞行界面）。
pub fn radar_keys(keys: Res<ButtonInput<KeyCode>>, mut game: ResMut<GameState>, mut radar: ResMut<TerrainRadar>) {
    if game.screen != Screen::Flight {
        return;
    }
    if keys.just_pressed(KeyCode::KeyN) {
        radar.cycle_mode();
        let t = match radar.mode {
            RadarMode::Off => "TERRAIN RADAR OFF",
            RadarMode::Small => "TERRAIN RADAR",
            RadarMode::Large => "TERRAIN RADAR + PROFILE",
        };
        game.toast = Some((t.to_string(), 1.5));
    }
    if keys.just_pressed(KeyCode::BracketRight) {
        radar.range_up();
        game.toast = Some((format!("RADAR RANGE {} KM", radar.range() / 1000.0), 1.5));
    }
    if keys.just_pressed(KeyCode::BracketLeft) {
        radar.range_down();
        game.toast = Some((format!("RADAR RANGE {} KM", radar.range() / 1000.0), 1.5));
    }
}

/// 构造 TAWS 输入（供系统与测试共用）。
pub fn taws_input(game: &GameState) -> TawsInput {
    let s = &game.sim;
    let r = &s.state.rigid;
    let airport_dist = game.world.nearest_airport(r.pos.x, r.pos.z).map(|(_, d)| d).unwrap_or(f64::MAX);
    TawsInput {
        pos: r.pos,
        vel: r.vel,
        agl: (s.diag.agl - s.config.rest_cg_height).max(0.0),
        gear_down: s.state.gear_pos > 0.5,
        on_ground: s.state.on_ground,
        airport_dist,
        ignore_water: s.config.amphibious,
    }
}

/// 每帧：5 Hz 计算告警/剖面；雷达图可见时分帧构建高程网格。
pub fn terrain_system(game: Res<GameState>, mut radar: ResMut<TerrainRadar>, time: Res<Time>) {
    if radar.epoch != game.world_epoch {
        radar.epoch = game.world_epoch;
        radar.invalidate();
    }
    if game.screen != Screen::Flight || game.paused {
        return;
    }
    let dt = time.delta_secs_f64().min(0.1);
    let world = game.world.clone();
    let height = |x: f64, z: f64| world.height(x, z);
    let r = game.sim.state.rigid.pos;

    if radar.popup > 0.0 {
        radar.popup = (radar.popup - dt).max(0.0);
    }
    radar.timer += dt;
    if radar.timer >= 0.2 {
        let step = radar.timer;
        radar.timer = 0.0;
        let inp = taws_input(&game);
        let st = taws::evaluate(&height, &inp);
        radar.status = st;
        radar.alert = radar.monitor.update(st.alert, step);
        if radar.alert.level() >= 1 {
            radar.popup = POPUP_HOLD_S;
        }
        if radar.shown_mode() == RadarMode::Large {
            let hdg = game.sim.diag.heading;
            let range = radar.range();
            radar.profile = terrain_map::profile(&height, r.x, r.z, hdg, range, PROFILE_N);
        }
    }
    if radar.visible() {
        radar.update_grid(&height, r.x, r.z);
    }
}

// ---------------------------------------------------------------------------
// 绘制
// ---------------------------------------------------------------------------

/// 雷达图绘制所需的飞行数据。
pub struct RadarView {
    pub x: f64,
    pub z: f64,
    pub heading: f64,
    pub alt: f64,
    pub vs: f64,
    pub gs: f64,
    pub agl_ft: f64,
    pub time: f64,
}

pub fn band_fill(b: Band) -> Rgba {
    match b {
        Band::Water => [12, 34, 84, 255],
        Band::Far => [8, 38, 26, 255],
        Band::Near => [18, 96, 48, 255],
        Band::Caution => [136, 98, 0, 255],
        Band::Hazard => [140, 22, 22, 255],
    }
}

pub fn band_line(b: Band, index: bool) -> Rgba {
    if index && b != Band::Water {
        return [236, 236, 236, 255];
    }
    match b {
        Band::Water => [12, 34, 84, 255],
        Band::Far => [34, 104, 72, 255],
        Band::Near => [70, 206, 104, 255],
        Band::Caution => [255, 200, 48, 255],
        Band::Hazard => [255, 96, 96, 255],
    }
}

fn alert_color(a: TawsAlert, time: f64) -> Option<Rgba> {
    match a.level() {
        2 => Some(if (time * 6.0) as i64 % 2 == 0 { RED } else { [120, 20, 20, 255] }),
        1 => Some(if (time * 2.0) as i64 % 2 == 0 { AMBER } else { [120, 84, 0, 255] }),
        _ => None,
    }
}

fn ring(buf: &mut PixelBuffer, cx: i32, cy: i32, r: i32, c: Rgba, dotted: bool) {
    let steps = (r as f64 * 6.3 * 2.0) as i32;
    for a in 0..steps {
        if dotted && a % 3 != 0 {
            continue;
        }
        let t = a as f64 / steps as f64 * std::f64::consts::TAU;
        buf.set(cx + (t.cos() * r as f64).round() as i32, cy + (t.sin() * r as f64).round() as i32, c);
    }
}

/// 在 (cx, cy) 绘制半径 `r` 像素的地形圆图。
fn draw_circle_map(buf: &mut PixelBuffer, radar: &TerrainRadar, v: &RadarView, cx: i32, cy: i32, r: i32) {
    // 背板
    buf.disc(cx, cy, r + 1, BLACK);
    match &radar.grid {
        Some(g) => {
            let cells = terrain_map::render_cells(g, v.x, v.z, v.heading, v.alt, radar.range(), r);
            let d = 2 * r + 1;
            for py in 0..d {
                for px in 0..d {
                    if let Some(Cell { band, contour }) = cells[(py * d + px) as usize] {
                        let c = if contour > 0 { band_line(band, contour == 2) } else { band_fill(band) };
                        buf.set(cx - r + px, cy - r + py, c);
                    }
                }
            }
        }
        None => buf.text_center(cx, cy - 3, "TERRAIN...", CYAN, 1),
    }
    // 量程环 + 外圈（告警时变色）
    ring(buf, cx, cy, r / 2, [200, 200, 210, 120], true);
    let rc = alert_color(radar.alert, v.time).unwrap_or(GRAY);
    ring(buf, cx, cy, r + 1, rc, false);
    // 北向标记
    let h = v.heading.to_radians();
    let (nx, ny) = (-h.sin(), -h.cos());
    let (px, py) = (cx + (nx * (r - 5) as f64).round() as i32, cy + (ny * (r - 5) as f64).round() as i32);
    buf.rect(px - 2, py - 3, 5, 7, BLACK);
    buf.text(px - 1, py - 2, "N", CYAN, 1);
    // 航向刻线（顶部）
    buf.rect(cx, cy - r - 1, 1, 3, YELLOW);
    // 本机符号
    buf.line(cx, cy - 4, cx - 3, cy + 3, YELLOW);
    buf.line(cx, cy - 4, cx + 3, cy + 3, YELLOW);
    buf.line(cx - 3, cy + 3, cx + 3, cy + 3, YELLOW);
}

/// 前方地形纵剖面（距离 0..量程）。
fn draw_profile(buf: &mut PixelBuffer, radar: &TerrainRadar, v: &RadarView, x0: i32, y0: i32, w: i32, h: i32) {
    buf.rect(x0, y0, w, h, PANEL_DK);
    buf.frame(x0, y0, w, h, GRAY);
    buf.text(x0 + 3, y0 + 3, "PROFILE", GRAY, 1);
    let range = radar.range();
    let rlabel = format!("{}KM", range / 1000.0);
    buf.text(x0 + w - 3 - PixelBuffer::text_width(&rlabel, 1), y0 + 3, &rlabel, GRAY, 1);

    let (px0, py0, pw, ph) = (x0 + 2, y0 + 12, w - 4, h - 12 - 14);
    if radar.profile.len() < 2 || pw < 4 {
        return;
    }
    let tmax = radar.profile.iter().cloned().fold(0.0f32, f32::max) as f64;
    let tmin = radar.profile.iter().cloned().fold(f32::MAX, f32::min) as f64;
    let path_end = v.alt + if v.gs > 5.0 { v.vs / v.gs * range } else { 0.0 };
    let mut vmin = tmin.min(v.alt).min(path_end) - 60.0;
    let mut vmax = tmax.max(v.alt).max(path_end) + 120.0;
    if vmax - vmin < 600.0 {
        let mid = (vmax + vmin) / 2.0;
        vmin = mid - 300.0;
        vmax = mid + 300.0;
    }
    let to_y = |e: f64| py0 + ph - 1 - (((e - vmin) / (vmax - vmin)) * (ph - 1) as f64).round() as i32;
    let n = radar.profile.len();
    for col in 0..pw {
        let f = col as f64 / (pw - 1) as f64 * (n - 1) as f64;
        let i = (f.floor() as usize).min(n - 2);
        let t = f - i as f64;
        let e = radar.profile[i] as f64 * (1.0 - t) + radar.profile[i + 1] as f64 * t;
        let band = terrain_map::classify(e as f32, v.alt);
        let top = to_y(e.max(0.0));
        let c = match band {
            Band::Water => band_fill(Band::Water),
            b => band_fill(b),
        };
        for y in top..(py0 + ph) {
            buf.set(px0 + col, y, c);
        }
        buf.set(px0 + col, top, band_line(band, false));
    }
    // 本机高度线（黄）与外推航迹（青色虚线）
    let ya = to_y(v.alt);
    buf.rect(px0, ya, 6, 1, YELLOW);
    let y1 = to_y(path_end);
    for col in 0..pw {
        if col % 3 != 0 {
            continue;
        }
        let y = ya + ((y1 - ya) as f64 * col as f64 / (pw - 1) as f64).round() as i32;
        if y >= py0 && y < py0 + ph {
            buf.set(px0 + col, y, CYAN);
        }
    }
    // 读数：海拔 / 雷达高度
    buf.text(x0 + 3, y0 + h - 10, &format!("MSL{:5.0} RA{:5.0}", v.alt / FT, v.agl_ft.max(0.0)), WHITE, 1);
}

/// 绘制地形雷达（按当前模式）。`view_h` 为 3D 画面高度（驾驶舱 112，其余 160）。
pub fn draw_radar(buf: &mut PixelBuffer, radar: &TerrainRadar, v: &RadarView) {
    match radar.shown_mode() {
        RadarMode::Off => {}
        RadarMode::Small => {
            let (cx, cy, r) = (278, 47, 32);
            draw_circle_map(buf, radar, v, cx, cy, r);
            let lbl = format!("{}KM", radar.range() / 1000.0);
            let w = PixelBuffer::text_width(&lbl, 1);
            buf.rect(cx - r + 1, cy + r - 8, w + 2, 8, BLACK);
            buf.text(cx - r + 2, cy + r - 7, &lbl, WHITE, 1);
            let ra = format!("RA{:.0}", v.agl_ft.max(0.0));
            let w = PixelBuffer::text_width(&ra, 1);
            buf.rect(cx + r - w - 2, cy + r - 8, w + 2, 8, BLACK);
            buf.text(cx + r - w - 1, cy + r - 7, &ra, if v.agl_ft < 500.0 { AMBER } else { WHITE }, 1);
        }
        RadarMode::Large => {
            let (cx, cy, r) = (62, 58, 46);
            draw_circle_map(buf, radar, v, cx, cy, r);
            let lbl = format!("{}KM", radar.range() / 1000.0);
            buf.rect(cx - r + 1, cy + r - 8, PixelBuffer::text_width(&lbl, 1) + 2, 8, BLACK);
            buf.text(cx - r + 2, cy + r - 7, &lbl, WHITE, 1);
            draw_profile(buf, radar, v, 112, 14, 200, 92);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_cycles_and_range_wraps() {
        let mut r = TerrainRadar::default();
        assert!(!r.visible());
        r.cycle_mode();
        assert_eq!(r.mode, RadarMode::Small);
        r.cycle_mode();
        assert_eq!(r.mode, RadarMode::Large);
        r.cycle_mode();
        assert_eq!(r.mode, RadarMode::Off);
        r.range_idx = RANGES_M.len() - 1;
        r.range_up();
        assert_eq!(r.range_idx, 0);
        r.range_down();
        assert_eq!(r.range_idx, 0);
    }

    #[test]
    fn popup_shows_small_map_when_off() {
        let mut r = TerrainRadar::default();
        r.popup = 5.0;
        assert!(r.visible());
        assert_eq!(r.shown_mode(), RadarMode::Small);
    }

    #[test]
    fn grid_builds_incrementally_and_rebuilds_near_edge() {
        let mut r = TerrainRadar::default();
        r.mode = RadarMode::Small;
        let h = |x: f64, _z: f64| (x * 0.05).max(0.0);
        let mut frames = 0;
        while r.grid.is_none() {
            r.update_grid(&h, 0.0, 0.0);
            frames += 1;
            assert!(frames < 100);
        }
        assert!(frames > 1, "incremental");
        // 飞到网格边缘附近 → 开始重建，旧网格仍可用
        r.update_grid(&h, 9_000.0, 0.0);
        assert!(r.building.is_some() && r.grid.is_some());
        // 量程变化 → 网格过期
        r.range_up();
        for _ in 0..100 {
            r.update_grid(&h, 9_000.0, 0.0);
        }
        assert!((r.grid.as_ref().unwrap().half - r.range() * GRID_SPAN).abs() < 1.0);
    }

    #[test]
    fn draws_circle_map_and_profile_without_panicking_and_with_pixels() {
        let mut r = TerrainRadar::default();
        r.mode = RadarMode::Large;
        let h = |x: f64, _z: f64| (x * 0.1).max(0.0);
        while r.grid.is_none() {
            r.update_grid(&h, 0.0, 0.0);
        }
        r.profile = terrain_map::profile(&h, 0.0, 0.0, 90.0, r.range(), PROFILE_N);
        let v = RadarView { x: 0.0, z: 0.0, heading: 90.0, alt: 600.0, vs: -3.0, gs: 80.0, agl_ft: 1900.0, time: 0.0 };
        let mut buf = PixelBuffer::new();
        buf.clear(TRANSPARENT);
        draw_radar(&mut buf, &r, &v);
        let opaque = (0..160).flat_map(|y| (0..320).map(move |x| (x, y))).filter(|&(x, y)| buf.get(x, y)[3] == 255).count();
        assert!(opaque > 6000, "map + profile should cover pixels: {opaque}");
        // 等高线颜色应出现（计曲线为白色）
        let has_index = (0..160).flat_map(|y| (0..320).map(move |x| (x, y))).any(|(x, y)| buf.get(x, y) == [236, 236, 236, 255]);
        assert!(has_index);
        r.mode = RadarMode::Off;
        let mut empty = PixelBuffer::new();
        empty.clear(TRANSPARENT);
        draw_radar(&mut empty, &r, &v);
        assert_eq!(empty.get(278, 47)[3], 0, "nothing drawn when off");
    }
}
