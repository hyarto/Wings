//! M5 测试：皮托加热/除冰指示灯与提示、地形告警（TAWS）与等高线雷达图（真实生成的世界）。

use crate::cockpit_ui::*;
use crate::terrain_radar::{self, RadarMode, TerrainRadar};
use crate::{GameState, Screen};
use bevy::math::DVec3;
use bevy::prelude::*;
use sim_core::integrator::SystemState;
use sim_core::taws::{self, TawsAlert, TawsInput};
use sim_core::KT;
use std::sync::{Arc, OnceLock};
use world_gen::World;

const SEED: u64 = 32;

fn world() -> Arc<World> {
    static W: OnceLock<Arc<World>> = OnceLock::new();
    W.get_or_init(|| Arc::new(World::new(SEED))).clone()
}

fn base_hud() -> FlightHud {
    FlightHud { ias_kt: 90.0, alt_ft: 3000.0, heading: 90.0, code: "SKY-172".into(), engine_label: "RPM", rpm: 2300.0, time: 0.0, ..Default::default() }
}

fn lamp_state(h: &FlightHud, label: &str) -> Lamp {
    annunciators(h).into_iter().find(|(l, _)| *l == label).map(|(_, s)| s).unwrap()
}

fn render(h: &FlightHud, view: ViewMode) -> PixelBuffer {
    let mut b = PixelBuffer::new();
    draw_flight(&mut b, h, view);
    b
}

// ------------------------------------------------------------ 指示灯 ----

#[test]
fn pitot_lamp_states() {
    let mut h = base_hud();
    h.pitot = SystemState::Off;
    assert_eq!(lamp_state(&h, "PITOT"), Lamp::Off);
    h.pitot = SystemState::On;
    assert_eq!(lamp_state(&h, "PITOT"), Lamp::On(GREEN));
    h.pitot = SystemState::Off;
    h.icing_cond = true;
    assert_eq!(lamp_state(&h, "PITOT"), Lamp::Blink(AMBER), "reminder: icing conditions + heat off");
    h.pitot = SystemState::On;
    h.pitot_blocked = true;
    assert_eq!(lamp_state(&h, "PITOT"), Lamp::Blink(RED), "blocked pitot is a red warning");
    h.pitot = SystemState::NotFitted;
    h.pitot_blocked = false;
    assert_eq!(lamp_state(&h, "PITOT"), Lamp::Dim);
}

#[test]
fn deice_lamp_states() {
    let mut h = base_hud();
    h.deice = SystemState::NotFitted;
    assert_eq!(lamp_state(&h, "DEICE"), Lamp::Dim);
    h.deice = SystemState::Off;
    assert_eq!(lamp_state(&h, "DEICE"), Lamp::Off);
    h.ice = 0.3;
    assert_eq!(lamp_state(&h, "DEICE"), Lamp::Blink(AMBER), "ice accreting, de-ice off");
    h.deice = SystemState::On;
    assert_eq!(lamp_state(&h, "DEICE"), Lamp::On(GREEN));
    h.deice = SystemState::NoPower;
    assert_eq!(lamp_state(&h, "DEICE"), Lamp::Blink(AMBER));
}

#[test]
fn ice_lamp_and_terrain_lamp() {
    let mut h = base_hud();
    assert_eq!(lamp_state(&h, "ICE"), Lamp::Off);
    h.icing_cond = true;
    assert_eq!(lamp_state(&h, "ICE"), Lamp::On(AMBER));
    h.ice = 0.6;
    assert_eq!(lamp_state(&h, "ICE"), Lamp::Blink(AMBER));
    assert_eq!(lamp_state(&h, "TERR"), Lamp::Off);
    h.taws = TawsAlert::Terrain;
    assert_eq!(lamp_state(&h, "TERR"), Lamp::Blink(AMBER));
    h.taws = TawsAlert::PullUp;
    assert_eq!(lamp_state(&h, "TERR"), Lamp::Blink(RED));
}

#[test]
fn lamps_render_visibly_in_every_view() {
    for view in [ViewMode::Cockpit, ViewMode::Hud, ViewMode::Chase, ViewMode::Flyby] {
        let calm = render(&base_hud(), view);
        for (name, h) in [
            ("pitot on", FlightHud { pitot: SystemState::On, ..base_hud() }),
            ("deice on", FlightHud { deice: SystemState::On, ..base_hud() }),
            ("pitot blocked", FlightHud { pitot_blocked: true, pitot: SystemState::On, ..base_hud() }),
            ("terrain", FlightHud { taws: TawsAlert::Terrain, ..base_hud() }),
            ("pull up", FlightHud { taws: TawsAlert::PullUp, ..base_hud() }),
        ] {
            assert_ne!(render(&h, view).get(0, 0), [1, 2, 3, 4]);
            let mut changed = 0;
            for y in 0..160 {
                for x in 0..320 {
                    if calm.get(x, y) != render(&h, view).get(x, y) {
                        changed += 1;
                    }
                }
            }
            assert!(changed > 0, "{name} invisible in {view:?}");
        }
    }
}

#[test]
fn lit_deice_lamp_is_green_pixels_and_dim_lamp_is_struck_through() {
    // 在驾驶舱视图里：DEICE 灯亮时出现绿色像素块
    let on = render(&FlightHud { deice: SystemState::On, ..base_hud() }, ViewMode::Cockpit);
    let mut green = 0;
    for y in 4..14 {
        for x in 0..320 {
            if on.get(x, y) == GREEN {
                green += 1;
            }
        }
    }
    assert!(green > 20, "lit lamp should be a solid green chip: {green}");
    // 未装备：同一位置没有绿色，但有划线（浅灰）
    let na = render(&FlightHud { deice: SystemState::NotFitted, ..base_hud() }, ViewMode::Cockpit);
    let mut strike = 0;
    for y in 4..14 {
        for x in 0..320 {
            assert_ne!(na.get(x, y), GREEN);
            if na.get(x, y) == [150, 150, 160, 255] {
                strike += 1;
            }
        }
    }
    assert!(strike >= 25, "N/A lamp must be struck through: {strike}");
}

#[test]
fn toast_is_drawn_and_pull_up_text_is_big() {
    let calm = render(&base_hud(), ViewMode::Cockpit);
    let t = render(&FlightHud { toast: Some("NO DE-ICE SYSTEM ON THIS AIRCRAFT".into()), ..base_hud() }, ViewMode::Cockpit);
    let mut diff = 0;
    for y in 54..68 {
        for x in 0..320 {
            if calm.get(x, y) != t.get(x, y) {
                diff += 1;
            }
        }
    }
    assert!(diff > 200, "toast banner: {diff}");
    let p = render(&FlightHud { taws: TawsAlert::PullUp, time: 0.0, ..base_hud() }, ViewMode::Cockpit);
    let mut red = 0;
    for y in 28..44 {
        for x in 0..320 {
            if p.get(x, y) == RED {
                red += 1;
            }
        }
    }
    assert!(red > 60, "PULL UP text: {red}");
}

#[test]
fn frozen_pitot_marks_airspeed_instrument() {
    let a = render(&base_hud(), ViewMode::Cockpit);
    let b = render(&FlightHud { pitot_blocked: true, ..base_hud() }, ViewMode::Cockpit);
    let mut diff = 0;
    for y in 112..160 {
        for x in 60..140 {
            if a.get(x, y) != b.get(x, y) {
                diff += 1;
            }
        }
    }
    assert!(diff > 30, "IAS box should show frozen state: {diff}");
}

// ------------------------------------------------------------ 按键提示 ----

fn input_app(code: &str) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(ButtonInput::<KeyCode>::default());
    app.insert_resource(GameState::new_with(SEED, None, 120.0, 10.0, code));
    app.add_systems(Update, crate::input::read_input);
    app
}

fn tap(app: &mut App, k: KeyCode) {
    app.world_mut().resource_mut::<ButtonInput<KeyCode>>().press(k);
    app.update();
    let mut i = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
    i.release(k);
    i.clear();
}

fn toast(app: &App) -> Option<String> {
    app.world().resource::<GameState>().toast.as_ref().map(|t| t.0.clone())
}

#[test]
fn de_ice_key_on_unequipped_aircraft_tells_the_pilot() {
    for code in ["SKY-172", "JET-39"] {
        let mut app = input_app(code);
        tap(&mut app, KeyCode::KeyI);
        let t = toast(&app).expect("a message must be shown");
        assert!(t.contains("NO DE-ICE"), "{code}: {t}");
        assert_eq!(app.world().resource::<GameState>().sim.deice_state(), SystemState::NotFitted);
    }
}

#[test]
fn de_ice_and_pitot_keys_toggle_with_feedback_on_equipped_aircraft() {
    let mut app = input_app("TWIN-6");
    tap(&mut app, KeyCode::KeyI);
    assert_eq!(toast(&app).as_deref(), Some("DE-ICE ON"));
    assert_eq!(app.world().resource::<GameState>().sim.deice_state(), SystemState::On);
    tap(&mut app, KeyCode::KeyI);
    assert_eq!(toast(&app).as_deref(), Some("DE-ICE OFF"));
    assert_eq!(app.world().resource::<GameState>().sim.deice_state(), SystemState::Off);

    tap(&mut app, KeyCode::KeyH);
    assert_eq!(toast(&app).as_deref(), Some("PITOT HEAT ON"));
    assert_eq!(app.world().resource::<GameState>().sim.pitot_heat_state(), SystemState::On);
    tap(&mut app, KeyCode::KeyH);
    assert_eq!(toast(&app).as_deref(), Some("PITOT HEAT OFF"));
}

#[test]
fn de_ice_without_engine_power_warns() {
    let mut app = input_app("TWIN-6");
    for e in &mut app.world_mut().resource_mut::<GameState>().sim.state.engines {
        e.running = false;
    }
    tap(&mut app, KeyCode::KeyI);
    assert!(toast(&app).unwrap().contains("NO ENGINE POWER"));
    assert_eq!(app.world().resource::<GameState>().sim.deice_state(), SystemState::NoPower);
}

// ------------------------------------------------------------ 地形告警 ----

/// 在生成的世界里找一座山：返回 (起点 x, z, 航向°, 飞行高度, 山顶位置)。
/// 飞机从起点水平飞向山顶，高度比山顶低 250 m，且起点地面比飞行高度低 ≥ 350 m。
fn find_mountain_approach(world: &World) -> ((f64, f64), f64, f64, (f64, f64)) {
    let (cx, cz) = world.airports()[0].center;
    let mut best: Option<(f64, ((f64, f64), f64, f64, (f64, f64)))> = None;
    for gi in -60..=60 {
        for gj in -60..=60 {
            let (px, pz) = (cx + gi as f64 * 4000.0, cz + gj as f64 * 4000.0);
            let h = world.height(px, pz);
            if !(1400.0..4000.0).contains(&h) {
                continue;
            }
            let alt = h - 250.0;
            for k in 0..8 {
                let th = k as f64 / 8.0 * std::f64::consts::TAU;
                let (sx, sz) = (px - th.sin() * 7000.0, pz + th.cos() * 7000.0);
                // 路径：起点附近必须明显低于飞行高度；沿途不得先撞到别的山（< alt-60）
                let ok = (0..=34).all(|i| {
                    let f = i as f64 / 35.0;
                    world.height(sx + (px - sx) * f, sz + (pz - sz) * f) < alt - 60.0 || f > 0.9
                });
                if ok && world.height(sx, sz) < alt - 350.0 {
                    let hdg = th.to_degrees().rem_euclid(360.0);
                    let score = h;
                    if best.as_ref().map_or(true, |b| score > b.0) {
                        best = Some((score, ((sx, sz), hdg, alt, (px, pz))));
                    }
                }
            }
        }
    }
    best.expect("seed 32 world must contain a mountain approach").1
}

#[test]
fn taws_warns_before_flying_into_a_real_mountain() {
    let w = world();
    let ((sx, sz), hdg, alt, p) = find_mountain_approach(&w);
    let height = |x: f64, z: f64| w.height(x, z);
    println!("mountain at {p:?} h={:.0}, start ({sx:.0},{sz:.0}) hdg {hdg:.0} alt {alt:.0}", w.height(p.0, p.1));
    let dir = DVec3::new(hdg.to_radians().sin(), 0.0, -hdg.to_radians().cos());
    let speed = 100.0 * KT;
    let mut first_caution = None;
    let mut first_warning = None;
    let mut impact = None;
    let mut ok_time = 0.0;
    // 逐秒推进：直线平飞撞山（解析推进，避免与飞行动力学耦合）
    for t in 0..400 {
        let t = t as f64;
        let pos = DVec3::new(sx, alt, sz) + dir * speed * t;
        let ground = height(pos.x, pos.z);
        if ground >= alt - 5.0 {
            impact = Some(t);
            break;
        }
        let inp = TawsInput { pos, vel: dir * speed, agl: alt - ground, gear_down: false, on_ground: false, airport_dist: f64::MAX, ignore_water: false };
        let s = taws::evaluate(&height, &inp);
        if s.alert == TawsAlert::None && first_caution.is_none() {
            ok_time = t;
        }
        if s.alert.level() >= 1 && first_caution.is_none() {
            first_caution = Some(t);
        }
        if s.alert == TawsAlert::PullUp && first_warning.is_none() {
            first_warning = Some(t);
        }
    }
    let impact = impact.expect("the straight flight path must hit the mountain");
    let caution = first_caution.expect("a caution must have been raised");
    let warning = first_warning.expect("a PULL UP must have been raised");
    println!("quiet until {ok_time}s, caution at {caution}s, PULL UP at {warning}s, impact at {impact}s");
    assert!(impact - caution >= 10.0, "caution lead time {}", impact - caution);
    assert!(impact - warning >= 4.0, "PULL UP lead time {}", impact - warning);
    assert!(warning >= caution);
    assert!(caution > 0.0, "should not already be alarming at the start (7 km away)");

    // 同一路线、高度高过山顶 300 m：全程无告警
    let high = w.height(p.0, p.1) + 300.0 + 400.0;
    for t in 0..120 {
        let pos = DVec3::new(sx, high, sz) + dir * speed * t as f64;
        let ground = height(pos.x, pos.z);
        let inp = TawsInput { pos, vel: dir * speed, agl: high - ground, gear_down: false, on_ground: false, airport_dist: f64::MAX, ignore_water: false };
        let s = taws::evaluate(&height, &inp);
        assert_eq!(s.alert, TawsAlert::None, "t={t} alt {high} ground {ground}");
    }
}

#[test]
fn taws_is_silent_for_the_normal_departure_roll_and_climb_out() {
    let g = GameState::new_with(SEED, None, 120.0, 10.0, "SKY-172");
    let inp = terrain_radar::taws_input(&g);
    assert!(inp.on_ground);
    let w = g.world.clone();
    assert_eq!(taws::evaluate(&|x, z| w.height(x, z), &inp).alert, TawsAlert::None);
    // 离场爬升：离地 60 m，距机场 1 km，爬升率 +600 fpm
    let ap = &w.airports()[g.airport];
    let (dx, dz) = (ap.center.0, ap.center.1);
    let inp = TawsInput {
        pos: DVec3::new(dx, w.height(dx, dz) + 60.0, dz),
        vel: DVec3::new(0.0, 3.0, -35.0),
        agl: 60.0,
        gear_down: true,
        on_ground: false,
        airport_dist: 1000.0,
        ignore_water: false,
    };
    assert_eq!(taws::evaluate(&|x, z| w.height(x, z), &inp).alert, TawsAlert::None);
}

#[test]
fn taws_sampling_is_cheap_enough_for_5hz_and_grid_build_is_incremental() {
    let w = world();
    let height = |x: f64, z: f64| w.height(x, z);
    let (cx, cz) = w.airports()[0].center;
    let t0 = std::time::Instant::now();
    let inp = TawsInput { pos: DVec3::new(cx, 3000.0, cz), vel: DVec3::new(80.0, 0.0, 0.0), agl: 2000.0, gear_down: false, on_ground: false, airport_dist: f64::MAX, ignore_water: false };
    for _ in 0..50 {
        taws::evaluate(&height, &inp);
    }
    let per = t0.elapsed().as_secs_f64() / 50.0;
    println!("taws evaluate: {:.2} ms", per * 1e3);
    assert!(per < 0.05, "evaluate too slow: {per}");
    let t1 = std::time::Instant::now();
    let mut g = sim_core::terrain_map::TerrainGrid::new(cx, cz, 15_000.0, 160);
    g.build_rows(&height, 24);
    let per_frame = t1.elapsed().as_secs_f64();
    println!("grid rows/frame (24 rows): {:.1} ms", per_frame * 1e3);
    assert!(per_frame < 0.25, "per-frame grid cost too high: {per_frame}");
}

// ------------------------------------------------------------ 雷达系统 ----

fn radar_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(ButtonInput::<KeyCode>::default());
    app.insert_resource(GameState::new(SEED, None, 120.0, 10.0));
    app.init_resource::<TerrainRadar>();
    app.add_systems(Update, (terrain_radar::radar_keys, terrain_radar::terrain_system).chain());
    app
}

fn run_frames(app: &mut App, n: usize) {
    for _ in 0..n {
        std::thread::sleep(std::time::Duration::from_millis(60));
        app.update();
    }
}

#[test]
fn n_key_cycles_radar_and_brackets_change_range() {
    let mut app = radar_app();
    assert_eq!(app.world().resource::<TerrainRadar>().mode, RadarMode::Off);
    tap(&mut app, KeyCode::KeyN);
    assert_eq!(app.world().resource::<TerrainRadar>().mode, RadarMode::Small);
    tap(&mut app, KeyCode::KeyN);
    assert_eq!(app.world().resource::<TerrainRadar>().mode, RadarMode::Large);
    let r0 = app.world().resource::<TerrainRadar>().range();
    tap(&mut app, KeyCode::BracketRight);
    assert!(app.world().resource::<TerrainRadar>().range() > r0);
    tap(&mut app, KeyCode::BracketLeft);
    assert_eq!(app.world().resource::<TerrainRadar>().range(), r0);
    assert!(app.world().resource::<GameState>().toast.is_some());
    tap(&mut app, KeyCode::KeyN);
    assert_eq!(app.world().resource::<TerrainRadar>().mode, RadarMode::Off);
    // 菜单界面里这些键无效
    app.world_mut().resource_mut::<GameState>().screen = Screen::Menu;
    tap(&mut app, KeyCode::KeyN);
    assert_eq!(app.world().resource::<TerrainRadar>().mode, RadarMode::Off);
}

#[test]
fn radar_builds_real_terrain_grid_and_auto_pops_up_on_terrain_alert() {
    let w = world();
    let ((sx, sz), hdg, alt, p) = find_mountain_approach(&w);
    let mut app = radar_app();
    {
        let mut g = app.world_mut().resource_mut::<GameState>();
        // 距山顶 ≈ 2.2 km 处平飞（约 21 s 后撞山）
        let dir = DVec3::new(hdg.to_radians().sin(), 0.0, -hdg.to_radians().cos());
        let pos = DVec3::new(p.0, alt, p.1) - dir * 2200.0;
        let _ = (sx, sz);
        g.sim.state.rigid.pos = pos;
        g.sim.state.rigid.vel = dir * 100.0 * KT;
        g.sim.state.on_ground = false;
        g.sim.diag.agl = (alt - w.height(pos.x, pos.z)).max(50.0);
        g.sim.diag.heading = hdg;
        g.sim.state.gear_pos = 0.0;
    }
    run_frames(&mut app, 14);
    let r = app.world().resource::<TerrainRadar>();
    assert!(r.alert.level() >= 1, "alert {:?} status {:?}", r.alert, r.status);
    assert!(r.popup > 0.0, "radar should pop up automatically");
    assert_eq!(r.mode, RadarMode::Off);
    assert_eq!(r.shown_mode(), RadarMode::Small);
    let grid = r.grid.as_ref().expect("grid built while visible");
    assert!(grid.is_complete());
    // 山顶位置在网格内，且高程接近真实值
    let e = grid.sample(p.0, p.1).expect("summit inside grid") as f64;
    assert!((e - w.height(p.0, p.1)).abs() < 150.0, "grid {e} vs world {}", w.height(p.0, p.1));

    // 渲染：前方（上方）应能看到红色/琥珀色危险区与等高线
    let v = terrain_radar::RadarView { x: app.world().resource::<GameState>().sim.state.rigid.pos.x, z: app.world().resource::<GameState>().sim.state.rigid.pos.z, heading: hdg, alt, vs: 0.0, gs: 51.4, agl_ft: 800.0, time: 0.0 };
    let mut buf = PixelBuffer::new();
    buf.clear(TRANSPARENT);
    terrain_radar::draw_radar(&mut buf, r, &v);
    let (mut hazard, mut lines) = (0, 0);
    for y in 14..80 {
        for x in 246..310 {
            let c = buf.get(x, y);
            if c == terrain_radar::band_fill(sim_core::terrain_map::Band::Hazard) || c == terrain_radar::band_line(sim_core::terrain_map::Band::Hazard, false) {
                hazard += 1;
            }
            if c[3] == 255 && c != [0, 0, 0, 255] && (c == [236, 236, 236, 255] || c == terrain_radar::band_line(sim_core::terrain_map::Band::Caution, false) || c == terrain_radar::band_line(sim_core::terrain_map::Band::Near, false) || c == terrain_radar::band_line(sim_core::terrain_map::Band::Far, false)) {
                lines += 1;
            }
        }
    }
    println!("hazard px {hazard}, contour px {lines}");
    assert!(hazard > 20, "terrain above the aircraft must show as red");
    assert!(lines > 20, "contour lines must be drawn");
}
