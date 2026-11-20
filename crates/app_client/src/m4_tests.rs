//! M4 客户端无头测试：5 机型程序化模型、开始菜单、机型/机场切换、水面出发、黑匣子回放、输入扩展，
//! 以及在真实生成世界的跑道上起飞。

use crate::aircraft_mesh::model_parts;
use crate::cockpit_ui::{PixelBuffer, BLACK};
use crate::menu::{draw_menu, draw_topview, MenuState, ROWS};
use crate::pixel_pipeline::{H, W};
use crate::{GameState, Screen};
use bevy::prelude::*;
use sim_core::aircraft::{AircraftConfig, CODES};
use sim_core::autopilot::*;
use sim_core::{FIXED_DT, KT};

const SEED: u64 = 32;

fn extents(m: &Mesh) -> (Vec3, Vec3) {
    let pos = m.attribute(Mesh::ATTRIBUTE_POSITION).and_then(|a| a.as_float3()).expect("positions");
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for p in pos {
        lo = lo.min(Vec3::from(*p));
        hi = hi.max(Vec3::from(*p));
    }
    (lo, hi)
}

#[test]
fn five_procedural_models_match_their_aircraft() {
    let mut counts = vec![];
    for code in CODES {
        let cfg = AircraftConfig::by_code(code).unwrap();
        let parts = model_parts(&cfg);
        let n = parts.body.count_vertices();
        let (lo, hi) = extents(&parts.body);
        println!("{code}: {n} verts, x {:.1}..{:.1}, z {:.1}..{:.1}, props {}", lo.x, hi.x, lo.z, hi.z, parts.props.len());
        assert!(n > 150, "{code} too simple");
        // 翼展与配置一致（后掠翼尖略向内）
        let semi = cfg.wing_span as f32 * 0.5;
        assert!(hi.x > semi * 0.8 && hi.x < semi * 1.2, "{code}: half span {} vs {}", hi.x, semi);
        assert!((lo.x + hi.x).abs() < 0.3, "{code} asymmetric");
        // 机身长度合理（8–40 m）
        assert!((hi.z - lo.z) > 7.0 && (hi.z - lo.z) < 45.0, "{code} length {}", hi.z - lo.z);
        assert_eq!(parts.surfaces.len(), 4, "{code} control surfaces");
        // 螺旋桨数量 = 螺旋桨发动机数量
        let want = cfg.engines.iter().filter(|e| !matches!(e.kind, sim_core::aircraft::EngineKind::Turbofan { .. })).count();
        assert_eq!(parts.props.len(), want, "{code} props");
        if code != "SKY-172" {
            assert!(parts.gear.count_vertices() >= 24, "{code} gear");
        }
        counts.push(n);
    }
    // 各机型模型互不相同
    let set: std::collections::HashSet<_> = counts.iter().collect();
    assert_eq!(set.len(), 5, "models should differ: {counts:?}");
    // 水上机：船体 + 浮筒使网格更复杂，且底部低于机轮收起线
    let sea = model_parts(&AircraftConfig::by_code("SEA-415").unwrap());
    let twin = model_parts(&AircraftConfig::by_code("TWIN-6").unwrap());
    assert!(sea.body.count_vertices() > twin.body.count_vertices());
    // 客机最大、喷气教练机最小的翼展
    let span = |c: &str| { let (lo, hi) = extents(&model_parts(&AircraftConfig::by_code(c).unwrap()).body); hi.x - lo.x };
    assert!(span("LINER-73") > span("SEA-415") * 0.9 && span("SEA-415") > span("TWIN-6") && span("TWIN-6") > span("JET-39"));
}

#[test]
fn menu_selection_applies_aircraft_airport_weather_and_season() {
    let mut game = GameState::new(SEED, None, 120.0, 10.0);
    assert_eq!(game.sim.config.code, "SKY-172");
    let mut m = MenuState::from_game(&game);
    assert_eq!(m.row, 7);
    // 行导航（循环）
    m.move_row(1);
    assert_eq!(m.row, 0);
    m.move_row(-1);
    assert_eq!(m.row, 7);
    // 机型：向右两次 → JET-39
    m.row = 0;
    m.adjust(1, false, 10);
    m.adjust(1, false, 10);
    assert_eq!(CODES[m.aircraft], "JET-39");
    // 机场
    let n = game.world.airports().len();
    m.row = 1;
    let a0 = m.airport;
    m.adjust(1, false, n);
    assert_eq!(m.airport, (a0 + 1) % n);
    // 起飞方式：非两栖机忽略
    m.row = 2;
    m.adjust(1, false, n);
    assert!(!m.water);
    // 天气 / 日期 / 时间（循环取模）
    m.row = 3;
    m.adjust(1, false, n);
    assert_eq!(m.preset, 1);
    m.row = 4;
    m.day = 362.0;
    m.adjust(1, false, n);
    assert!((m.day - 2.0).abs() < 1e-9, "day wraps: {}", m.day);
    m.adjust(-1, true, n);
    assert!((m.day - 337.0).abs() < 1e-9);
    m.row = 5;
    m.hour = 23.0;
    m.adjust(1, false, n);
    assert_eq!(m.hour, 0.0);
    m.apply(&mut game);
    assert_eq!(game.sim.config.code, "JET-39");
    assert_eq!(game.airport, (a0 + 1) % n);
    assert_eq!(game.preset, 1);
    assert!((game.day - 337.0).abs() < 1e-9 && game.hour == 0.0);
    assert_eq!(game.screen, Screen::Flight);
    assert!(game.sim.controls.parking_brake);
    // 出发位置在所选机场跑道上
    let ap = &game.world.airports()[game.airport];
    let p = game.sim.state.rigid.pos;
    assert!(world_gen::coords::wrapped_distance(ap.center, (p.x, p.z)) < ap.length, "not at selected airport");
}

#[test]
fn menu_seed_change_regenerates_the_planet() {
    let mut game = GameState::new(SEED, None, 120.0, 10.0);
    let before: Vec<String> = game.world.airports().iter().take(8).map(|a| a.icao.clone()).collect();
    let mut m = MenuState::from_game(&game);
    m.row = 6;
    m.adjust(1, true, 0);
    assert_eq!(m.seed, SEED + 100);
    // 种子改变后机场行不再可调（序号失效）
    m.row = 1;
    let a = m.airport;
    m.adjust(1, false, 100);
    assert_eq!(m.airport, a);
    let epoch = game.world_epoch;
    m.apply(&mut game);
    assert_eq!(game.seed, SEED + 100);
    assert_eq!(game.world_epoch, epoch + 1);
    let after: Vec<String> = game.world.airports().iter().take(8).map(|a| a.icao.clone()).collect();
    assert_ne!(before, after, "different seed must give a different airport network");
    assert_eq!(game.screen, Screen::Flight);
    // 同一种子可复现
    let w2 = world_gen::World::new(SEED + 100);
    assert_eq!(w2.airports()[0].icao, game.world.airports()[0].icao);
}

#[test]
fn every_aircraft_spawns_stably_on_a_generated_runway() {
    let mut game = GameState::new(SEED, None, 120.0, 10.0);
    for (i, code) in CODES.iter().enumerate() {
        game.set_aircraft(i);
        assert_eq!(game.sim.config.code, *code);
        let y0 = game.sim.state.rigid.pos.y;
        for _ in 0..(120 * 4) {
            game.sim.step();
        }
        assert!(!game.sim.state.damage.any(), "{code} damaged on spawn");
        assert!(game.sim.state.on_ground, "{code} not on ground");
        assert!(game.sim.diag.ground_speed < 0.5, "{code} rolling with parking brake: {}", game.sim.diag.ground_speed);
        assert!((game.sim.state.rigid.pos.y - y0).abs() < 0.4, "{code} sank/jumped {} -> {}", y0, game.sim.state.rigid.pos.y);
    }
}

#[test]
fn amphibian_starts_on_water_when_requested() {
    let mut game = GameState::new_with(SEED, None, 120.0, 10.0, "SEA-415");
    game.water_start = true;
    let n = game.world.airports().len();
    let mut found = None;
    for i in 0..n.min(60) {
        let hdg = game.world.airports()[i].heading0;
        if let Some(w) = game.find_water_near(i, hdg) {
            found = Some((i, w));
            break;
        }
    }
    let (idx, (x, z)) = found.expect("an airport with open water nearby");
    assert!(game.world.height(x, z) < -4.0);
    game.depart(idx);
    let p = game.sim.state.rigid.pos;
    assert!(game.world.height(p.x, p.z) <= 0.0, "must start over water");
    assert!(game.sim.state.gear_pos < 0.05, "gear up on water");
    for _ in 0..(120 * 8) {
        game.sim.step();
    }
    assert!(game.sim.diag.gear.float_contact && !game.sim.state.damage.any());
    assert!(game.sim.state.rigid.pos.y < 3.0 && game.sim.state.rigid.pos.y > 0.3);
    // 非两栖机忽略水面选项
    let mut g2 = GameState::new_with(SEED, None, 120.0, 10.0, "LINER-73");
    g2.water_start = true;
    g2.depart(idx);
    let p2 = g2.sim.state.rigid.pos;
    assert!(g2.world.height(p2.x, p2.z) > 0.0 || g2.world.airports()[idx].elevation >= 0.0);
    assert!(!g2.sim.config.amphibious);
}

#[test]
fn blackbox_records_in_game_and_replays_then_restores() {
    let mut game = GameState::new(SEED, None, 120.0, 10.0);
    assert!(!game.start_replay(), "nothing recorded yet");
    game.sim.controls.parking_brake = false;
    game.sim.controls.throttle = 1.0;
    for _ in 0..(60 * 12) {
        game.sim.advance(1.0 / 60.0);
        let g = &mut game;
        g.blackbox.record(&g.sim);
    }
    assert!(game.blackbox.len() > 100, "len {}", game.blackbox.len());
    let live_pos = game.sim.state.rigid.pos;
    let live_t = game.sim.state.time;
    assert!(game.start_replay());
    // 回放起点 = 记录起点（飞机尚在起点附近）
    let first = game.blackbox.samples[0];
    assert!((game.sim.state.rigid.pos.z - first.pos[2]).abs() < 1e-6);
    assert!((game.sim.state.rigid.pos - live_pos).length() > 20.0, "replay should rewind the aircraft");
    game.replay_tick(5.0);
    assert!(game.replay.is_some());
    let mid = game.sim.state.rigid.pos;
    assert!((mid - live_pos).length() > 1.0);
    game.replay_tick(-100.0); // 倒带钳制到开头
    assert!((game.sim.state.rigid.pos.z - first.pos[2]).abs() < 1e-6);
    game.replay_tick(1000.0); // 播放结束自动退出并恢复实时状态
    assert!(game.replay.is_none());
    assert_eq!(game.sim.state.rigid.pos, live_pos);
    assert_eq!(game.sim.state.time, live_t);
    // CSV 往返
    let csv = game.blackbox.to_csv();
    let back = sim_core::blackbox::BlackBox::from_csv(&csv).unwrap();
    assert_eq!(back.len(), game.blackbox.len());
    // 重新出发会清空黑匣子
    game.respawn();
    assert!(game.blackbox.is_empty());
}

#[test]
fn menu_screen_renders_and_topviews_differ() {
    let game = GameState::new(SEED, None, 120.0, 10.0);
    let mut buf = PixelBuffer::new();
    let mut m = MenuState::from_game(&game);
    let mut sigs = vec![];
    for i in 0..CODES.len() {
        m.aircraft = i;
        m.row = i % ROWS;
        draw_menu(&mut buf, &m, &game, 0.0);
        let mut lit = 0;
        for y in 0..H as i32 {
            for x in 0..W as i32 {
                if buf.get(x, y) != [0, 0, 0, 255] {
                    lit += 1;
                }
            }
        }
        assert!(lit > 1500, "menu too empty: {lit}");
        // 剪影
        let mut tv = PixelBuffer::new();
        tv.clear(BLACK);
        draw_topview(&mut tv, &AircraftConfig::by_code(CODES[i]).unwrap(), 160, 80, 40.0, [255, 255, 255, 255]);
        let mut sig = vec![];
        for y in 0..H as i32 {
            for x in 0..W as i32 {
                if tv.get(x, y) == [255, 255, 255, 255] {
                    sig.push((x, y));
                }
            }
        }
        assert!(sig.len() > 60, "{} silhouette too small", CODES[i]);
        sigs.push(sig);
    }
    for i in 0..sigs.len() {
        for j in i + 1..sigs.len() {
            assert_ne!(sigs[i], sigs[j], "top views {i} and {j} identical");
        }
    }
}

fn input_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(ButtonInput::<KeyCode>::default());
    app.insert_resource(GameState::new_with(SEED, None, 120.0, 10.0, "TWIN-6"));
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

#[test]
fn tab_opens_menu_and_new_keys_drive_engine_failure_and_replay() {
    let mut app = input_app();
    tap(&mut app, KeyCode::Tab);
    assert_eq!(app.world().resource::<GameState>().screen, Screen::Menu);
    // 菜单中 M 键不切换航图
    tap(&mut app, KeyCode::KeyM);
    assert_eq!(app.world().resource::<GameState>().screen, Screen::Menu);
    tap(&mut app, KeyCode::Tab);
    assert_eq!(app.world().resource::<GameState>().screen, Screen::Flight);
    // F9 失效发动机、F10 顺桨
    tap(&mut app, KeyCode::F9);
    assert!(app.world().resource::<GameState>().sim.state.engines[0].failed);
    assert!(!app.world().resource::<GameState>().sim.state.engines[1].failed);
    tap(&mut app, KeyCode::F9);
    assert!(app.world().resource::<GameState>().sim.state.engines[1].failed);
    tap(&mut app, KeyCode::F10);
    assert!(app.world().resource::<GameState>().sim.controls.feather);
    // 无记录时 R 不启动回放
    tap(&mut app, KeyCode::KeyR);
    assert!(app.world().resource::<GameState>().replay.is_none());
    // 有记录后 R 开始/退出回放
    {
        let mut g = app.world_mut().resource_mut::<GameState>();
        for _ in 0..600 {
            g.sim.advance(1.0 / 60.0);
            let gg = &mut *g;
            gg.blackbox.record(&gg.sim);
        }
    }
    tap(&mut app, KeyCode::KeyR);
    assert!(app.world().resource::<GameState>().replay.is_some());
    tap(&mut app, KeyCode::ArrowRight);
    tap(&mut app, KeyCode::KeyR);
    assert!(app.world().resource::<GameState>().replay.is_none());
}

/// 在真实生成世界的跑道上，每种陆地机型都能完成起飞滑跑并离地爬升。
#[test]
fn every_land_aircraft_takes_off_from_a_generated_runway() {
    for code in ["TWIN-6", "JET-39", "LINER-73", "SEA-415"] {
        let mut game = GameState::new_with(SEED, None, 120.0, 10.0, code);
        let hdg0 = game.sim.diag.heading;
        let ap_len = game.world.airports()[game.airport].length;
        let cfg = game.sim.config.clone();
        let sim = &mut game.sim;
        sim.controls.parking_brake = false;
        sim.controls.throttle = 1.0;
        sim.controls.flaps = match code {
            "TWIN-6" => 0.27,
            "JET-39" => 0.5,
            "LINER-73" => 5.0 / 40.0,
            _ => 0.33,
        };
        sim.state.surfaces.flaps = sim.controls.flaps;
        let mut ph = PitchHold::default();
        let p0 = sim.state.rigid.pos;
        let mut lifted = false;
        let mut climbed = false;
        for _ in 0..(120 * 90) {
            let ias = sim.diag.ias_true / KT;
            sim.controls.yaw = heading_to_rudder(sim, hdg0);
            let t = if ias >= cfg.v_rotate / KT { 9.0 } else { 0.0 };
            sim.controls.pitch = ph.control(sim, t, FIXED_DT);
            sim.step();
            if !lifted && !sim.state.on_ground && sim.state.air_time > 0.3 {
                lifted = true;
                let run = ((sim.state.rigid.pos.x - p0.x).powi(2) + (sim.state.rigid.pos.z - p0.z).powi(2)).sqrt();
                println!("{code}: lift-off after {run:.0} m of a {ap_len:.0} m runway");
                assert!(run < ap_len + 50.0, "{code}: ran off the runway end ({run} > {ap_len})");
            }
            if lifted && sim.diag.agl > 40.0 {
                climbed = true;
                break;
            }
            assert!(!sim.state.damage.crashed, "{code} crashed on takeoff");
        }
        assert!(lifted && climbed, "{code} never got airborne");
        assert!(!sim.state.damage.any(), "{code} damaged: {:?}", sim.state.damage);
    }
}
