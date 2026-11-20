//! Android / 触屏支持：虚拟摇杆 + 油门滑杆 + 触控按钮 + 菜单点选。
//!
//! 设计（见 docs/ANDROID_PORT_ASSESSMENT.md §2.2「虚拟键盘注入层」）：
//! * 本模块**不改动**旧的键盘逻辑：触控按钮在 `PreUpdate` 里向 `ButtonInput<KeyCode>` 注入
//!   `press()/release()`，`read_input` / `menu_system` / `map_system` 照常工作。
//! * 模拟量（摇杆俯仰/滚转、油门滑杆）通过 `TouchAxes` 资源并入 `read_input`。
//! * 渲染用 bevy_ui 覆盖层（独立 UI 摄像机，order = 1），不受 320×160 量化影响；
//!   命中检测自己用 `Touches` 的逻辑像素坐标做，支持多点触控。
//! * 桌面默认隐藏，Android 默认启用；桌面可加 `--touch` 或首次触摸后启用。

use crate::cockpit_ui::ViewMode;
use crate::pixel_pipeline::{H, W};
use crate::{GameState, Screen};
use bevy::audio::{AudioSink, AudioSinkPlayback};
use bevy::camera::visibility::RenderLayers;
use bevy::input::touch::Touches;
use bevy::input::InputSystems;
use bevy::prelude::*;
use bevy::ui::IsDefaultUiCamera;
use bevy::window::{AppLifecycle, PrimaryWindow};
use std::collections::{HashMap, HashSet};

/// 触控模拟量输出（`read_input` 消费）。
#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct TouchAxes {
    /// 俯仰：+1 = 拉杆（抬头）。向下拖动 = 拉杆。
    pub pitch: f64,
    /// 滚转：+1 = 右滚。
    pub roll: f64,
    /// 油门绝对值（仅当手指按在滑杆上时为 Some）。
    pub throttle: Option<f64>,
}

// ----------------------------------------------------------------------------- 布局表

const C_FLIGHT: u8 = 1;
const C_REPLAY: u8 = 2;
const C_MAP: u8 = 4;
const C_MENU: u8 = 8;
/// 仅当「MORE」面板展开时显示（飞行界面）。
const C_PANEL: u8 = 16;

#[derive(Clone, Copy, PartialEq)]
enum Act {
    /// 按住期间持续按下该键。
    Hold(KeyCode),
    /// 触碰瞬间按下一帧。
    Tap(KeyCode),
    /// 展开/收起 MORE 面板。
    More,
    /// 循环视角 1→2→3→4。
    View,
}

#[derive(Clone, Copy)]
enum Anchor {
    TL,
    TR,
    BL,
    BR,
}

struct Spec {
    label: &'static str,
    act: Act,
    ctx: u8,
    anchor: Anchor,
    dx: f32,
    dy: f32,
    w: f32,
    h: f32,
}

#[allow(clippy::too_many_arguments)]
const fn b(label: &'static str, act: Act, ctx: u8, anchor: Anchor, dx: f32, dy: f32, w: f32, h: f32) -> Spec {
    Spec { label, act, ctx, anchor, dx, dy, w, h }
}

use Act::*;
use Anchor::*;
use KeyCode as K;

#[rustfmt::skip]
const BTNS: &[Spec] = &[
    // ---- 飞行：顶部一行
    b("MENU",  Tap(K::Tab),    C_FLIGHT, TL,   6.0,  6.0, 46.0, 34.0),
    b("MAP",   Tap(K::KeyM),   C_FLIGHT, TL,  58.0,  6.0, 46.0, 34.0),
    b("VIEW",  View,           C_FLIGHT, TL, 110.0,  6.0, 46.0, 34.0),
    b("PAUSE", Tap(K::KeyP),   C_FLIGHT, TL, 162.0,  6.0, 46.0, 34.0),
    b("MORE",  More,           C_FLIGHT, TL, 214.0,  6.0, 46.0, 34.0),
    // ---- 飞行：右侧按钮列（油门滑杆在更靠边处）
    b("GEAR",  Tap(K::KeyG),   C_FLIGHT, TR,  56.0, 50.0, 46.0, 36.0),
    b("FLAP+", Tap(K::KeyF),   C_FLIGHT, TR,  56.0, 92.0, 46.0, 36.0),
    b("FLAP-", Tap(K::KeyV),   C_FLIGHT, TR,  56.0,134.0, 46.0, 36.0),
    b("BRAKE", Hold(K::Space), C_FLIGHT, TR,  56.0,176.0, 46.0, 36.0),
    // ---- 飞行：方向舵（右下）
    b("RUD<",  Hold(K::KeyQ),  C_FLIGHT, BR, 108.0,  8.0, 46.0, 40.0),
    b("RUD>",  Hold(K::KeyE),  C_FLIGHT, BR,  56.0,  8.0, 46.0, 40.0),
    // ---- 飞行：MORE 面板（4 列网格）
    b("PARK",  Tap(K::KeyB),       C_PANEL, TL,   6.0, 48.0, 46.0, 34.0),
    b("PITOT", Tap(K::KeyH),       C_PANEL, TL,  58.0, 48.0, 46.0, 34.0),
    b("DEICE", Tap(K::KeyI),       C_PANEL, TL, 110.0, 48.0, 46.0, 34.0),
    b("WX",    Tap(K::KeyC),       C_PANEL, TL, 162.0, 48.0, 46.0, 34.0),
    b("SPBRK", Hold(K::KeyJ),      C_PANEL, TL,   6.0, 88.0, 46.0, 34.0),
    b("REV",   Hold(K::KeyK),      C_PANEL, TL,  58.0, 88.0, 46.0, 34.0),
    b("TRIM+", Hold(K::KeyT),      C_PANEL, TL, 110.0, 88.0, 46.0, 34.0),
    b("TRIM-", Hold(K::KeyY),      C_PANEL, TL, 162.0, 88.0, 46.0, 34.0),
    b("REPLY", Tap(K::KeyR),       C_PANEL, TL,   6.0,128.0, 46.0, 34.0),
    b("RESET", Tap(K::Backspace),  C_PANEL, TL,  58.0,128.0, 46.0, 34.0),
    b("ENG-",  Tap(K::F9),         C_PANEL, TL, 110.0,128.0, 46.0, 34.0),
    b("FEATH", Tap(K::F10),        C_PANEL, TL, 162.0,128.0, 46.0, 34.0),
    b("FULL",  Tap(K::KeyZ),       C_PANEL, TL,   6.0,168.0, 46.0, 34.0),
    b("IDLE",  Tap(K::KeyX),       C_PANEL, TL,  58.0,168.0, 46.0, 34.0),
    b("CSV",   Tap(K::F5),         C_PANEL, TL, 110.0,168.0, 46.0, 34.0),
    b("TERR",  Tap(K::KeyN),       C_PANEL, TL, 162.0,168.0, 46.0, 34.0),
    b("RANGE", Tap(K::BracketRight), C_PANEL, TL, 214.0,168.0, 46.0, 34.0),
    // ---- 黑匣子回放
    b("EXIT",  Tap(K::KeyR),       C_REPLAY, TL,   6.0, 6.0, 46.0, 34.0),
    b("<<5S",  Tap(K::ArrowLeft),  C_REPLAY, TL,  58.0, 6.0, 46.0, 34.0),
    b("5S>>",  Tap(K::ArrowRight), C_REPLAY, TL, 110.0, 6.0, 46.0, 34.0),
    b("PAUSE", Tap(K::KeyP),       C_REPLAY, TL, 162.0, 6.0, 46.0, 34.0),
    // ---- 航图：日期/时间/天气
    b("DAY-",  Tap(K::BracketLeft),  C_MAP, TL,   6.0, 6.0, 46.0, 34.0),
    b("DAY+",  Tap(K::BracketRight), C_MAP, TL,  58.0, 6.0, 46.0, 34.0),
    b("HR-",   Tap(K::Comma),        C_MAP, TL, 110.0, 6.0, 46.0, 34.0),
    b("HR+",   Tap(K::Period),       C_MAP, TL, 162.0, 6.0, 46.0, 34.0),
    b("WX",    Tap(K::KeyC),         C_MAP, TL, 214.0, 6.0, 46.0, 34.0),
    // ---- 航图 / 菜单：方向键 + 确认 / 返回
    b("^",     Hold(K::ArrowUp),    C_MAP | C_MENU, BL,  58.0, 100.0, 46.0, 40.0),
    b("v",     Hold(K::ArrowDown),  C_MAP | C_MENU, BL,  58.0,  12.0, 46.0, 40.0),
    b("<",     Hold(K::ArrowLeft),  C_MAP | C_MENU, BL,   6.0,  56.0, 46.0, 40.0),
    b(">",     Hold(K::ArrowRight), C_MAP | C_MENU, BL, 110.0,  56.0, 46.0, 40.0),
    b("GO",    Tap(K::Enter),       C_MAP | C_MENU, BR,   6.0,  12.0, 90.0, 52.0),
    b("BACK",  Tap(K::Escape),      C_MAP | C_MENU, BR,   6.0,  72.0, 90.0, 40.0),
    b("BIG",   Hold(K::ShiftLeft),  C_MAP | C_MENU, BR,   6.0, 120.0, 90.0, 40.0),
];

/// 油门滑杆（右缘）。
const SLIDER_DX: f32 = 6.0;
const SLIDER_W: f32 = 44.0;
const SLIDER_TOP: f32 = 50.0;
const SLIDER_BOTTOM: f32 = 64.0;
/// 摇杆最大偏移半径（dp，乘 UI 缩放）。
const STICK_R: f32 = 62.0;
/// 刘海 / 圆角安全边距（dp）。
const SIDE_SAFE: f32 = 14.0;

#[derive(Clone, Copy)]
struct R {
    x: f32,
    y: f32,
    w: f32,
    h: f32,
}
impl R {
    fn hit(&self, p: Vec2) -> bool {
        p.x >= self.x && p.x <= self.x + self.w && p.y >= self.y && p.y <= self.y + self.h
    }
}

/// 按窗口逻辑尺寸计算 UI 缩放（以 360dp 高为 1.0）。
fn ui_scale(win: Vec2) -> f32 {
    (win.y / 360.0).clamp(0.8, 1.7)
}

fn spec_rect(s: &Spec, win: Vec2) -> R {
    let u = ui_scale(win);
    let (w, h) = (s.w * u, s.h * u);
    let sx = SIDE_SAFE * u;
    let x = match s.anchor {
        TL | BL => s.dx * u + sx,
        TR | BR => win.x - s.dx * u - w - sx,
    };
    let y = match s.anchor {
        TL | TR => s.dy * u,
        BL | BR => win.y - s.dy * u - h,
    };
    R { x, y, w, h }
}

fn slider_rect(win: Vec2) -> R {
    let u = ui_scale(win);
    let w = SLIDER_W * u;
    let top = SLIDER_TOP * u;
    R { x: win.x - SLIDER_DX * u - w - SIDE_SAFE * u, y: top, w, h: (win.y - top - SLIDER_BOTTOM * u).max(40.0) }
}

/// 320×160 渲染区在窗口中的逻辑矩形与「窗口逻辑像素 / 游戏像素」比例
/// （与 `pixel_pipeline::fit_quad_to_window` 同一算法）。
fn game_rect(win: &Window) -> (R, f32) {
    let (pw, ph) = (win.physical_width().max(1), win.physical_height().max(1));
    let s = (pw / W).min(ph / H).max(1) as f32;
    let sf = win.scale_factor().max(0.1);
    let (w, h) = (W as f32 * s / sf, H as f32 * s / sf);
    (R { x: (win.width() - w) / 2.0, y: (win.height() - h) / 2.0, w, h }, s / sf)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ctx {
    Flight,
    Replay,
    Map,
    Menu,
}
impl Ctx {
    fn bit(self) -> u8 {
        match self {
            Ctx::Flight => C_FLIGHT,
            Ctx::Replay => C_REPLAY,
            Ctx::Map => C_MAP,
            Ctx::Menu => C_MENU,
        }
    }
}

fn visible(spec: &Spec, ctx: Ctx, more: bool) -> bool {
    if spec.ctx & C_PANEL != 0 {
        return ctx == Ctx::Flight && more;
    }
    spec.ctx & ctx.bit() != 0
}

// ----------------------------------------------------------------------------- 状态

#[derive(Resource, Default)]
pub struct TouchUi {
    pub enabled: bool,
    more: bool,
    ctx: Option<Ctx>,
    stick_id: Option<u64>,
    stick_origin: Vec2,
    stick_pos: Vec2,
    slider_id: Option<u64>,
    btn_touch: HashMap<u64, usize>,
    held: Vec<bool>,
    ours: HashSet<KeyCode>,
    win: Vec2,
}

fn current_ctx(g: &GameState) -> Ctx {
    match g.screen {
        Screen::Menu => Ctx::Menu,
        Screen::Map => Ctx::Map,
        Screen::Flight => {
            if g.replay.is_some() {
                Ctx::Replay
            } else {
                Ctx::Flight
            }
        }
    }
}

/// 摇杆偏移 → [-1,1]，带死区与指数曲线（便于微调）。
fn stick_curve(v: f32) -> f64 {
    let dz = 0.07;
    let a = v.abs();
    if a < dz {
        return 0.0;
    }
    let n = ((a - dz) / (1.0 - dz)).clamp(0.0, 1.0);
    (v.signum() * (0.25 * n + 0.75 * n * n)) as f64
}

const FLY_ROW_TAP: usize = usize::MAX;

#[allow(clippy::too_many_arguments)]
fn touch_input_system(
    touches: Res<Touches>,
    windows: Query<&Window, With<PrimaryWindow>>,
    game: Res<GameState>,
    menu: Option<ResMut<crate::menu::MenuState>>,
    mut keys: ResMut<ButtonInput<KeyCode>>,
    mut ui: ResMut<TouchUi>,
    mut axes: ResMut<TouchAxes>,
) {
    let Ok(window) = windows.single() else { return };
    if !ui.enabled {
        if touches.iter().next().is_some() || touches.iter_just_pressed().next().is_some() {
            ui.enabled = true; // 桌面触屏：首次触摸后启用
        } else {
            return;
        }
    }
    let win = Vec2::new(window.width(), window.height());
    let ctx = current_ctx(&game);
    let ui = &mut *ui;
    ui.win = win;
    if ui.held.len() != BTNS.len() {
        ui.held = vec![false; BTNS.len()];
    }
    if ui.ctx != Some(ctx) {
        // 屏幕切换：释放所有控件，避免残留按住状态
        ui.ctx = Some(ctx);
        ui.btn_touch.clear();
        ui.stick_id = None;
        ui.slider_id = None;
    }
    let sl = slider_rect(win);
    let stick_enabled = matches!(ctx, Ctx::Flight | Ctx::Map);
    let (grect, gscale) = game_rect(window);

    let mut want: HashSet<KeyCode> = HashSet::new();
    let mut tapped: Vec<usize> = Vec::new();

    // 清理已抬起的触点
    let alive = |id: u64| touches.get_pressed(id).is_some();
    ui.btn_touch.retain(|id, _| alive(*id));
    if ui.stick_id.is_some_and(|id| !alive(id)) {
        ui.stick_id = None;
    }
    if ui.slider_id.is_some_and(|id| !alive(id)) {
        ui.slider_id = None;
    }

    // 新触点分配：按钮 > 滑杆 > 摇杆
    let mut new_holds: Vec<usize> = Vec::new();
    let mut menu_taps: Vec<Vec2> = Vec::new();
    for t in touches.iter_just_pressed() {
        let p = t.start_position();
        let mut taken = false;
        for (i, s) in BTNS.iter().enumerate() {
            if visible(s, ctx, ui.more) && spec_rect(s, win).hit(p) {
                taken = true;
                match s.act {
                    Hold(_) => {
                        ui.btn_touch.insert(t.id(), i);
                        new_holds.push(i);
                    }
                    _ => tapped.push(i),
                }
                break;
            }
        }
        if taken {
            continue;
        }
        if ctx == Ctx::Flight && sl.hit(p) && ui.slider_id.is_none() {
            ui.slider_id = Some(t.id());
            continue;
        }
        if ctx == Ctx::Menu {
            menu_taps.push(p);
            continue;
        }
        if stick_enabled && ui.stick_id.is_none() && p.x < win.x * 0.45 && p.y > win.y * 0.12 {
            ui.stick_id = Some(t.id());
            ui.stick_origin = p;
            ui.stick_pos = p;
        }
    }

    // 菜单：直接点选行 / 值（320×160 像素坐标系；行 y = 18 + 13i，值区 x≥60：左半减、右半加）
    if let Some(mut m) = menu {
        if ctx == Ctx::Menu {
            let big = ui.ours.contains(&KeyCode::ShiftLeft);
            let n = game.world.airports().len();
            for p in menu_taps {
                if !grect.hit(p) {
                    continue;
                }
                let (px, py) = ((p.x - grect.x) / gscale, (p.y - grect.y) / gscale);
                let row = ((py - 16.0) / 13.0).floor();
                if !(0.0..crate::menu::ROWS as f32).contains(&row) || px > 184.0 {
                    continue;
                }
                let row = row as usize;
                m.row = row;
                if row == crate::menu::ROWS - 1 {
                    tapped.push(FLY_ROW_TAP);
                } else if px >= 60.0 {
                    m.adjust(if px < 120.0 { -1 } else { 1 }, big, n);
                }
            }
        }
    }

    // 摇杆 / 滑杆
    axes.pitch = 0.0;
    axes.roll = 0.0;
    axes.throttle = None;
    if let Some(id) = ui.stick_id {
        if let Some(t) = touches.get_pressed(id) {
            ui.stick_pos = t.position();
        }
        let r = STICK_R * ui_scale(win);
        let d = (ui.stick_pos - ui.stick_origin) / r;
        let d = d.clamp_length_max(1.0);
        if ctx == Ctx::Flight {
            axes.roll = stick_curve(d.x);
            axes.pitch = stick_curve(d.y);
        } else {
            for (cond, k) in [(d.x < -0.35, KeyCode::ArrowLeft), (d.x > 0.35, KeyCode::ArrowRight), (d.y < -0.35, KeyCode::ArrowUp), (d.y > 0.35, KeyCode::ArrowDown)] {
                if cond {
                    want.insert(k);
                }
            }
        }
    }
    if ctx == Ctx::Flight {
        let level = |p: Vec2| Some((1.0 - ((p.y - sl.y) / sl.h) as f64).clamp(0.0, 1.0));
        if let Some(id) = ui.slider_id {
            if let Some(t) = touches.get_pressed(id) {
                axes.throttle = level(t.position());
            } else if let Some(t) = touches.iter_just_pressed().find(|t| t.id() == id) {
                axes.throttle = level(t.start_position());
            }
        }
    }

    // 按钮：按住 / 点按
    for h in ui.held.iter_mut() {
        *h = false;
    }
    let held_idx: Vec<usize> = ui.btn_touch.values().copied().chain(new_holds).collect();
    for i in held_idx {
        ui.held[i] = true;
        if let Hold(k) = BTNS[i].act {
            want.insert(k);
        }
    }
    for i in tapped {
        if i == FLY_ROW_TAP {
            want.insert(KeyCode::Enter);
            continue;
        }
        ui.held[i] = true;
        match BTNS[i].act {
            Tap(k) => {
                want.insert(k);
            }
            More => ui.more = !ui.more,
            View => {
                let next = match game.view {
                    ViewMode::Cockpit => KeyCode::Digit2,
                    ViewMode::Hud => KeyCode::Digit3,
                    ViewMode::Chase => KeyCode::Digit4,
                    ViewMode::Flyby => KeyCode::Digit1,
                };
                want.insert(next);
            }
            Hold(_) => {}
        }
    }
    if ctx != Ctx::Flight {
        ui.more = false;
    }

    // 注入键盘状态（只释放我们自己按下的键，不干扰物理键盘）
    for k in want.iter() {
        keys.press(*k);
    }
    let stale: Vec<KeyCode> = ui.ours.iter().filter(|k| !want.contains(k)).copied().collect();
    for k in stale {
        keys.release(k);
    }
    ui.ours = want;
}

// ----------------------------------------------------------------------------- 绘制

#[derive(Component)]
struct BtnNode(usize);
#[derive(Component)]
enum Deco {
    StickBase,
    StickKnob,
    SliderTrack,
    SliderFill,
    SliderKnob,
    SliderLabel,
}

const BG: Color = Color::srgba(0.04, 0.05, 0.09, 0.55);
const BG_ON: Color = Color::srgba(0.95, 0.65, 0.10, 0.80);
const AMBER: Color = Color::srgb(1.0, 0.78, 0.25);

fn base_node(radius: f32) -> Node {
    Node {
        position_type: PositionType::Absolute,
        display: Display::None,
        justify_content: JustifyContent::Center,
        align_items: AlignItems::Center,
        border_radius: BorderRadius::all(Val::Px(radius)),
        ..default()
    }
}

fn setup_touch_ui(mut commands: Commands, mut ui: ResMut<TouchUi>) {
    ui.enabled = cfg!(target_os = "android") || std::env::args().any(|a| a == "--touch");
    // 专用 UI 摄像机：叠加在像素后处理画面之上（order 1），不清屏。
    commands.spawn((Camera2d, Camera { order: 1, clear_color: ClearColorConfig::None, ..default() }, Msaa::Off, RenderLayers::layer(2), IsDefaultUiCamera));
    for (i, s) in BTNS.iter().enumerate() {
        commands.spawn((base_node(6.0), BackgroundColor(BG), BtnNode(i), GlobalZIndex(10))).with_children(|p| {
            p.spawn((Text::new(s.label), TextFont { font_size: 11.0, ..default() }, TextColor(AMBER)));
        });
    }
    commands.spawn((base_node(999.0), BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.10)), Deco::StickBase, GlobalZIndex(9)));
    commands.spawn((base_node(999.0), BackgroundColor(Color::srgba(1.0, 0.78, 0.25, 0.45)), Deco::StickKnob, GlobalZIndex(9)));
    commands.spawn((base_node(8.0), BackgroundColor(BG), Deco::SliderTrack, GlobalZIndex(9)));
    commands.spawn((base_node(8.0), BackgroundColor(Color::srgba(0.95, 0.65, 0.10, 0.45)), Deco::SliderFill, GlobalZIndex(9)));
    commands.spawn((base_node(6.0), BackgroundColor(Color::srgba(1.0, 0.9, 0.5, 0.9)), Deco::SliderKnob, GlobalZIndex(9)));
    commands.spawn((base_node(0.0), Deco::SliderLabel, GlobalZIndex(9))).with_children(|p| {
        p.spawn((Text::new("THR"), TextFont { font_size: 11.0, ..default() }, TextColor(AMBER)));
    });
}

fn set_rect(n: &mut Node, r: R) {
    n.display = Display::Flex;
    n.left = Val::Px(r.x);
    n.top = Val::Px(r.y);
    n.width = Val::Px(r.w);
    n.height = Val::Px(r.h);
}

#[allow(clippy::type_complexity)]
fn draw_touch_ui(
    ui: Res<TouchUi>,
    game: Res<GameState>,
    mut btns: Query<(&BtnNode, &mut Node, &mut BackgroundColor), Without<Deco>>,
    mut decos: Query<(&Deco, &mut Node), Without<BtnNode>>,
) {
    let on = ui.enabled && ui.ctx.is_some();
    let ctx = ui.ctx.unwrap_or(Ctx::Flight);
    let win = ui.win;
    for (b, mut n, mut bg) in &mut btns {
        let s = &BTNS[b.0];
        if on && visible(s, ctx, ui.more) {
            set_rect(&mut n, spec_rect(s, win));
            let lit = ui.held.get(b.0).copied().unwrap_or(false) || (matches!(s.act, More) && ui.more);
            bg.0 = if lit { BG_ON } else { BG };
        } else {
            n.display = Display::None;
        }
    }
    let u = ui_scale(win);
    let sl = slider_rect(win);
    let thr = game.sim.controls.throttle as f32;
    let rest = Vec2::new(win.x * 0.17 + SIDE_SAFE * u, win.y * 0.68);
    for (d, mut n) in &mut decos {
        n.display = Display::None;
        if !on {
            continue;
        }
        let flight = ctx == Ctx::Flight;
        match d {
            Deco::StickBase | Deco::StickKnob if matches!(ctx, Ctx::Flight | Ctx::Map) => {
                let r = STICK_R * u;
                let (origin, pos) = if ui.stick_id.is_some() { (ui.stick_origin, ui.stick_pos) } else { (rest, rest) };
                let off = (pos - origin).clamp_length_max(r);
                if matches!(d, Deco::StickBase) {
                    set_rect(&mut n, R { x: origin.x - r, y: origin.y - r, w: 2.0 * r, h: 2.0 * r });
                } else {
                    let k = r * 0.42;
                    set_rect(&mut n, R { x: origin.x + off.x - k, y: origin.y + off.y - k, w: 2.0 * k, h: 2.0 * k });
                }
            }
            Deco::SliderTrack if flight => set_rect(&mut n, sl),
            Deco::SliderFill if flight => set_rect(&mut n, R { x: sl.x, y: sl.y + sl.h * (1.0 - thr), w: sl.w, h: sl.h * thr }),
            Deco::SliderKnob if flight => set_rect(&mut n, R { x: sl.x - 2.0, y: sl.y + sl.h * (1.0 - thr) - 5.0 * u, w: sl.w + 4.0, h: 10.0 * u }),
            Deco::SliderLabel if flight => set_rect(&mut n, R { x: sl.x, y: sl.y - 18.0 * u, w: sl.w, h: 16.0 * u }),
            _ => {}
        }
    }
}

/// 切后台自动暂停并静音；回到前台恢复音频（保持暂停，由玩家点 PAUSE 继续）。
fn lifecycle_system(mut events: MessageReader<AppLifecycle>, mut game: ResMut<GameState>, sinks: Query<&AudioSink>) {
    for e in events.read() {
        match e {
            AppLifecycle::WillSuspend | AppLifecycle::Suspended => {
                game.paused = true;
                for s in &sinks {
                    s.pause();
                }
            }
            AppLifecycle::Running => {
                for s in &sinks {
                    s.play();
                }
            }
            _ => {}
        }
    }
}

pub struct TouchUiPlugin;

impl Plugin for TouchUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TouchAxes>()
            .init_resource::<TouchUi>()
            .add_systems(Startup, setup_touch_ui)
            .add_systems(PreUpdate, touch_input_system.after(InputSystems))
            .add_systems(Update, (draw_touch_ui, lifecycle_system));
    }
}

// ----------------------------------------------------------------------------- 测试（无头）

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::input::touch::{TouchInput, TouchPhase};
    use bevy::input::InputPlugin;

    const SIZES: [(f32, f32); 5] = [(800.0, 360.0), (915.0, 412.0), (640.0, 360.0), (960.0, 480.0), (2400.0, 1080.0)];

    fn overlap(a: R, b: R) -> bool {
        a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h
    }

    #[test]
    fn stick_curve_has_deadzone_and_is_monotonic() {
        assert_eq!(stick_curve(0.0), 0.0);
        assert_eq!(stick_curve(0.05), 0.0);
        assert!((stick_curve(1.0) - 1.0).abs() < 1e-6 && (stick_curve(-1.0) + 1.0).abs() < 1e-6);
        let mut last = 0.0;
        for i in 8..=100 {
            let v = stick_curve(i as f32 / 100.0);
            assert!(v >= last);
            last = v;
        }
        assert!(stick_curve(0.5) < 0.5, "指数曲线：中段灵敏度低于线性");
    }

    #[test]
    fn layout_fits_and_does_not_overlap() {
        for (w, h) in SIZES {
            let win = Vec2::new(w, h);
            let sl = slider_rect(win);
            assert!(sl.x > 0.0 && sl.x + sl.w < w && sl.y > 0.0 && sl.y + sl.h < h, "slider off-screen {w}x{h}");
            for ctx in [Ctx::Flight, Ctx::Replay, Ctx::Map, Ctx::Menu] {
                for more in [false, true] {
                    let vis: Vec<(usize, R)> = BTNS.iter().enumerate().filter(|(_, s)| visible(s, ctx, more)).map(|(i, s)| (i, spec_rect(s, win))).collect();
                    for (i, r) in &vis {
                        assert!(r.x >= 0.0 && r.y >= 0.0 && r.x + r.w <= w && r.y + r.h <= h, "{} off-screen at {w}x{h}", BTNS[*i].label);
                        if ctx == Ctx::Flight {
                            assert!(!overlap(*r, sl), "{} overlaps throttle slider at {w}x{h}", BTNS[*i].label);
                        }
                    }
                    for (a, (i, ra)) in vis.iter().enumerate() {
                        for (j, rb) in vis.iter().skip(a + 1) {
                            assert!(!overlap(*ra, *rb), "{} overlaps {} ({ctx:?}/{more}) at {w}x{h}", BTNS[*i].label, BTNS[*j].label, ctx = ctx as u8);
                        }
                    }
                }
            }
        }
    }

    fn app() -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, InputPlugin));
        let win = app.world_mut().spawn((Window { resolution: (800u32, 360u32).into(), ..default() }, PrimaryWindow)).id();
        app.insert_resource(GameState::new(32, None, 120.0, 10.0));
        app.init_resource::<TouchAxes>();
        app.insert_resource(TouchUi { enabled: true, ..default() });
        app.add_systems(PreUpdate, touch_input_system.after(InputSystems));
        app.add_systems(Update, crate::input::read_input);
        app.update();
        (app, win)
    }

    fn touch(app: &mut App, win: Entity, phase: TouchPhase, id: u64, p: Vec2) {
        app.world_mut().write_message(TouchInput { phase, position: p, window: win, force: None, id });
        app.update();
    }

    fn center(label: &str, ctx: Ctx) -> Vec2 {
        let s = BTNS.iter().find(|s| s.label == label && s.ctx & (ctx.bit() | C_PANEL) != 0).expect("button");
        let r = spec_rect(s, Vec2::new(800.0, 360.0));
        Vec2::new(r.x + r.w / 2.0, r.y + r.h / 2.0)
    }

    #[test]
    fn tap_gear_and_hold_brake_via_touch() {
        let (mut app, win) = app();
        let g0 = app.world().resource::<GameState>().sim.controls.gear_down;
        let p = center("GEAR", Ctx::Flight);
        touch(&mut app, win, TouchPhase::Started, 1, p);
        assert_ne!(app.world().resource::<GameState>().sim.controls.gear_down, g0, "GEAR 点按应切换起落架");
        touch(&mut app, win, TouchPhase::Ended, 1, p);
        app.update();
        assert_ne!(app.world().resource::<GameState>().sim.controls.gear_down, g0, "抬手后不应再次切换");

        let p = center("BRAKE", Ctx::Flight);
        touch(&mut app, win, TouchPhase::Started, 2, p);
        app.update();
        assert_eq!(app.world().resource::<GameState>().sim.controls.brake_left, 1.0);
        touch(&mut app, win, TouchPhase::Ended, 2, p);
        app.update();
        assert_eq!(app.world().resource::<GameState>().sim.controls.brake_left, 0.0);
    }

    #[test]
    fn throttle_slider_is_absolute() {
        let (mut app, win) = app();
        let sl = slider_rect(Vec2::new(800.0, 360.0));
        let x = sl.x + sl.w / 2.0;
        touch(&mut app, win, TouchPhase::Started, 1, Vec2::new(x, sl.y + 1.0));
        app.update();
        assert!(app.world().resource::<GameState>().sim.controls.throttle > 0.98);
        touch(&mut app, win, TouchPhase::Moved, 1, Vec2::new(x, sl.y + sl.h * 0.5));
        assert!((app.world().resource::<GameState>().sim.controls.throttle - 0.5).abs() < 0.03);
        touch(&mut app, win, TouchPhase::Ended, 1, Vec2::new(x, sl.y + sl.h * 0.5));
        app.update();
        assert!((app.world().resource::<GameState>().sim.controls.throttle - 0.5).abs() < 0.03, "抬手后油门保持");
    }

    #[test]
    fn floating_stick_drives_pitch_and_roll() {
        let (mut app, win) = app();
        let o = Vec2::new(150.0, 250.0);
        touch(&mut app, win, TouchPhase::Started, 1, o);
        touch(&mut app, win, TouchPhase::Moved, 1, o + Vec2::new(60.0, 60.0));
        app.update();
        let c = app.world().resource::<GameState>().sim.controls;
        assert!(c.roll > 0.5, "向右拖 = 右滚: {}", c.roll);
        assert!(c.pitch > 0.5, "向下拖 = 拉杆: {}", c.pitch);
        touch(&mut app, win, TouchPhase::Ended, 1, o);
        app.update();
        let c = app.world().resource::<GameState>().sim.controls;
        assert_eq!((c.roll, c.pitch), (0.0, 0.0), "抬手回中");
    }

    #[test]
    fn more_panel_toggles_and_menu_button_switches_screen() {
        let (mut app, win) = app();
        let p = center("MORE", Ctx::Flight);
        touch(&mut app, win, TouchPhase::Started, 1, p);
        touch(&mut app, win, TouchPhase::Ended, 1, p);
        assert!(app.world().resource::<TouchUi>().more);
        let pk = center("PARK", Ctx::Flight); // 面板内按钮
        let before = app.world().resource::<GameState>().sim.controls.parking_brake;
        touch(&mut app, win, TouchPhase::Started, 2, pk);
        touch(&mut app, win, TouchPhase::Ended, 2, pk);
        assert_ne!(app.world().resource::<GameState>().sim.controls.parking_brake, before);
        let m = center("MENU", Ctx::Flight);
        touch(&mut app, win, TouchPhase::Started, 3, m);
        assert_eq!(app.world().resource::<GameState>().screen, Screen::Menu);
    }
}
