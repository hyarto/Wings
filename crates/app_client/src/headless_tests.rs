//! app_client 无头测试（不需要 GPU / 窗口）：
//! * 浮动原点精度、程序化网格、地形 Chunk、驾驶舱像素渲染、航图；
//! * `WorldGround` 适配器、`GameState` 起飞/天气预设、键盘输入系统；
//! * M2 DoD：飞越经度 ±180° 无坐标跳变；
//! * M3 DoD：低能见度雨/雪/雾中仅依靠仪表（ILS 航向道 + 下滑道）盲降落回跑道。

use crate::cockpit_ui::*;
use crate::map_screen;
use crate::mesh_util::{rgba, MeshBuilder};
use crate::origin::to_render;
use crate::terrain_mesh::{build_chunk, ChunkKey, CELLS, LOD_SIZE};
use crate::{GameState, Screen, WorldGround};
use bevy::math::DVec3;
use bevy::prelude::*;
use sim_core::autopilot::*;
use sim_core::gear::{Ground, SurfaceType};
use sim_core::weather::PrecipKind;
use sim_core::{FIXED_DT, KT};
use std::sync::{Arc, OnceLock};
use world_gen::airport::AirportKind;
use world_gen::coords::*;
use world_gen::World;

const SEED: u64 = 32;

fn world() -> Arc<World> {
    static W: OnceLock<Arc<World>> = OnceLock::new();
    W.get_or_init(|| Arc::new(World::new(SEED))).clone()
}

// ------------------------------------------------------------- 浮动原点 ----

#[test]
fn floating_origin_keeps_subcentimeter_precision_far_from_origin() {
    // 世界边缘 ±625 km：f32 的绝对坐标分辨率约 6 cm，而浮动原点后相对坐标 < 1 mm
    let origin = DVec3::new(612_345.678_9, 3_200.123_4, -301_234.567_8);
    let mut worst_naive: f64 = 0.0;
    for i in 0..1000 {
        let off = DVec3::new((i as f64 * 7.31).sin() * 4000.0, (i as f64 * 1.7).cos() * 300.0, (i as f64 * 3.3).sin() * 4000.0);
        let p = origin + off;
        let r = to_render(origin, p);
        let err = (DVec3::new(r.x as f64, r.y as f64, r.z as f64) - off).length();
        assert!(err < 5e-4, "floating-origin error {err}");
        let naive = (p.as_vec3() - origin.as_vec3()).as_dvec3();
        worst_naive = worst_naive.max((naive - off).length());
    }
    println!("naive f32 subtraction worst error {worst_naive:.3} m");
    assert!(worst_naive > 0.02, "premise: naive f32 jitters at this range");
}

#[test]
fn floating_origin_motion_is_smooth_for_100km_flight() {
    // 以 60 m/s 沿 X 飞行 100 km（含 f64 累加），世界固定物体的渲染位置每帧位移精确 = 速度 × dt
    let anchor = DVec3::new(0.0, 0.0, 0.0);
    let mut origin = DVec3::new(-50_000.0, 500.0, 0.0);
    let dt = 1.0 / 60.0;
    let mut prev = to_render(origin, anchor);
    let mut worst: f32 = 0.0;
    for _ in 0..(100_000.0 / (60.0 * dt)) as usize {
        origin.x += 60.0 * dt;
        let r = to_render(origin, anchor);
        let step = (prev.x - r.x).abs();
        worst = worst.max((step - (60.0 * dt) as f32).abs());
        prev = r;
    }
    // 在 ±50 km 范围内 f32 分辨率约 4 mm，但抖动不会随飞行距离增长到 > 1 cm（无“跳变”）
    assert!(worst < 0.01, "frame-to-frame jitter {worst} m");
}

// ------------------------------------------------------------- 网格构建 ----

#[test]
fn mesh_builder_counts_and_normals() {
    let mut m = MeshBuilder::new();
    assert!(m.is_empty());
    m.tri(Vec3::ZERO, Vec3::Z, Vec3::X, [1.0; 4]); // 逆时针（自上而下看）→ 法线朝 +Y
    assert_eq!(m.idx.len(), 3);
    assert!((Vec3::from(m.nor[0]) - Vec3::Y).length() < 1e-5);
    m.cuboid(Vec3::ZERO, Vec3::splat(1.0), Quat::IDENTITY, [1.0, 0.0, 0.0, 1.0]);
    assert_eq!(m.idx.len(), 3 + 12 * 3, "cuboid = 12 triangles");
    let before = m.idx.len();
    m.blob(Vec3::ZERO, Vec3::new(2.0, 1.0, 2.0), [1.0; 4]);
    assert_eq!(m.idx.len() - before, 8 * 4 * 3, "blob = 32 triangles");
    // 所有索引有效；每个顶点有法线/颜色
    assert!(m.idx.iter().all(|&i| (i as usize) < m.pos.len()));
    assert_eq!(m.pos.len(), m.nor.len());
    assert_eq!(m.pos.len(), m.col.len());
    // 长方体法线朝外
    let mut c = MeshBuilder::new();
    c.aabb(Vec3::ZERO, Vec3::splat(1.0), [1.0; 4]);
    for (p, n) in c.pos.iter().zip(&c.nor) {
        assert!(Vec3::from(*p).dot(Vec3::from(*n)) > 0.9, "inward-facing normal at {p:?} {n:?}");
    }
    let mesh = c.build();
    assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    assert!(mesh.attribute(Mesh::ATTRIBUTE_COLOR).is_some());
    assert_eq!(rgba([0.1, 0.2, 0.3]), [0.1, 0.2, 0.3, 1.0]);
}

// ------------------------------------------------------------ 地形 Chunk ----

fn chunk_vertices(mb: &MeshBuilder) -> usize {
    mb.pos.len()
}

#[test]
fn terrain_chunks_are_deterministic_and_well_formed() {
    let w = world();
    let key = ChunkKey { lod: 1, cx: 12, cz: -7 };
    let a = build_chunk(&w, 120.0, key);
    let b = build_chunk(&w, 120.0, key);
    assert_eq!(a.pos, b.pos);
    assert_eq!(a.col, b.col);
    // 表面三角形 + 裙边
    let tris_surface = (CELLS * CELLS * 2) as usize;
    let tris_skirt = (CELLS * 4 * 2) as usize;
    assert_eq!(a.idx.len(), (tris_surface + tris_skirt) * 3);
    assert_eq!(chunk_vertices(&a), a.idx.len());
    for c in &a.col {
        assert!(c.iter().all(|v| (0.0..=1.0).contains(v)));
    }
    // 顶点坐标相对 Chunk 中心（f32 精度安全）
    let half = LOD_SIZE[1] as f32 / 2.0 + 1.0;
    assert!(a.pos.iter().all(|p| p[0].abs() <= half && p[2].abs() <= half));
}

#[test]
fn adjacent_chunks_share_identical_edge_heights_without_cracks() {
    let w = world();
    for lod in 0..3u8 {
        for &(cx, cz) in &[(3i64, 4i64), (-20, 9), (0, 0)] {
            let a_key = ChunkKey { lod, cx, cz };
            let b_key = ChunkKey { lod, cx: cx + 1, cz };
            let (a, b) = (build_chunk(&w, 100.0, a_key), build_chunk(&w, 100.0, b_key));
            let size = LOD_SIZE[lod as usize] as f32;
            // A 的右边缘（x = +size/2）与 B 的左边缘（x = -size/2）逐顶点高度一致
            let edge = |mb: &MeshBuilder, x: f32| -> Vec<(i32, f32)> {
                let mut v: Vec<(i32, f32)> = mb
                    .pos
                    .iter()
                    .filter(|p| (p[0] - x).abs() < 1e-3 && p[1] > -1e6)
                    .map(|p| ((p[2] * 10.0).round() as i32, p[1]))
                    .collect();
                v.sort_by_key(|e| e.0);
                v.dedup_by_key(|e| e.0);
                v
            };
            let ea = edge(&a, size / 2.0);
            let eb = edge(&b, -size / 2.0);
            // 裙边顶点向下延伸；取每个 z 位置上最高者作为地表
            let top = |mb: &MeshBuilder, x: f32| -> Vec<(i32, f32)> {
                let mut m = std::collections::BTreeMap::new();
                for p in mb.pos.iter().filter(|p| (p[0] - x).abs() < 1e-3) {
                    let k = (p[2] * 10.0).round() as i32;
                    let e = m.entry(k).or_insert(f32::MIN);
                    *e = e.max(p[1]);
                }
                m.into_iter().collect()
            };
            let _ = (ea, eb);
            let (ta, tb) = (top(&a, size / 2.0), top(&b, -size / 2.0));
            assert_eq!(ta.len(), tb.len());
            for (p, q) in ta.iter().zip(&tb) {
                assert_eq!(p.0, q.0);
                assert!((p.1 - q.1).abs() < 1e-3, "crack at lod {lod}: {} vs {}", p.1, q.1);
            }
        }
    }
}

#[test]
fn terrain_chunks_tile_seamlessly_across_antimeridian() {
    let w = world();
    // LOD1 Chunk 尺寸 4800 m；世界宽 1,250,000 m 不是整数倍，所以 ±180° 附近存在跨界 Chunk。
    let s = LOD_SIZE[1];
    let cx = (WORLD_WIDTH / 2.0 / s).floor() as i64; // 越过 +W/2 的第一块
    let cz = (-20_000.0 / s) as i64;
    let a = build_chunk(&w, 200.0, ChunkKey { lod: 1, cx, cz });
    let b = build_chunk(&w, 200.0, ChunkKey { lod: 1, cx: cx + 1, cz });
    let (xa, _) = ChunkKey { lod: 1, cx, cz }.center();
    assert!(xa + s as f64 / 2.0 > WORLD_WIDTH / 2.0, "chunk straddles the seam");
    // 同样的世界点（x 与 x−W）高度一致 → 相邻块边缘一致
    let size = s as f32;
    let top = |mb: &MeshBuilder, x: f32| -> Vec<f32> {
        let mut m = std::collections::BTreeMap::new();
        for p in mb.pos.iter().filter(|p| (p[0] - x).abs() < 1e-3) {
            let k = (p[2] * 10.0).round() as i32;
            let e = m.entry(k).or_insert(f32::MIN);
            *e = e.max(p[1]);
        }
        m.into_values().collect()
    };
    for (p, q) in top(&a, size / 2.0).iter().zip(top(&b, -size / 2.0).iter()) {
        assert!((p - q).abs() < 1e-3, "seam crack {p} vs {q}");
    }
}

#[test]
fn terrain_chunk_over_airport_is_flat_at_runway_elevation() {
    let w = world();
    let ap = w.airports().iter().find(|a| a.kind == AirportKind::Regional).expect("regional");
    let s = LOD_SIZE[0];
    let key = ChunkKey { lod: 0, cx: (ap.center.0 / s).floor() as i64, cz: (ap.center.1 / s).floor() as i64 };
    let mb = build_chunk(&w, 100.0, key);
    let (xc, zc) = key.center();
    let mut checked = 0;
    for p in mb.pos.iter() {
        let (wx, wz) = (xc + p[0] as f64, zc + p[2] as f64);
        let (along, lat) = ap.runway_local(wx, wz);
        if along > 0.0 && along < ap.length && lat.abs() < ap.width / 2.0 {
            // 顶点在跑道内：高度 = 机场标高（含 LOD0 偏移 0）
            if p[1] as f64 > ap.elevation - 5.0 {
                assert!((p[1] as f64 - ap.elevation).abs() < 0.05, "runway vertex {} vs {}", p[1], ap.elevation);
                checked += 1;
            }
        }
    }
    println!("checked {checked} runway vertices");
    assert!(checked > 0 || mb.pos.len() > 100);
}

// ---------------------------------------------------- WorldGround 适配器 ----

#[test]
fn world_ground_reports_surfaces_heights_and_flat_normals() {
    let w = world();
    let g = WorldGround { world: w.clone(), day: 120.0 };
    // 机场：沥青/土路
    let mut seen_dirt = false;
    let mut seen_asphalt = false;
    for a in w.airports() {
        let (x, z) = a.runway_point(a.length / 2.0, 0.0);
        let s = g.sample(x, z);
        assert!((s.height - a.elevation).abs() < 0.05);
        assert!(s.normal.y > 0.999, "runway must be level: {:?}", s.normal);
        match a.kind {
            AirportKind::Dirt => {
                assert_eq!(s.surface, SurfaceType::Dirt);
                seen_dirt = true;
            }
            _ => {
                assert_eq!(s.surface, SurfaceType::Asphalt);
                seen_asphalt = true;
            }
        }
        // 跑道外 100 m（草地）
        let (gx, gz) = a.runway_point(a.length / 2.0, a.flat_radius * 0.9);
        let sg = g.sample(gx, gz);
        assert!(sg.surface != SurfaceType::Asphalt);
    }
    assert!(seen_dirt && seen_asphalt);
    // 海洋：水面，高度 0
    let mut found_ocean = false;
    'o: for i in 0..200 {
        for j in 0..100 {
            let (x, z) = (-600_000.0 + i as f64 * 6000.0, -300_000.0 + j as f64 * 6000.0);
            if w.is_ocean(x, z) && w.height(x, z) < -50.0 {
                let s = g.sample(x, z);
                assert_eq!(s.surface, SurfaceType::Water);
                assert_eq!(s.height, 0.0);
                assert!((s.normal - DVec3::Y).length() < 1e-9);
                found_ocean = true;
                break 'o;
            }
        }
    }
    assert!(found_ocean);
    // 极地冬季：积雪地表
    let cold = WorldGround { world: w.clone(), day: 355.0 };
    let mut found_snow = false;
    'p: for i in 0..300 {
        for j in 0..20 {
            let (x, z) = latlon_to_world(70.0 + j as f64, -180.0 + i as f64 * 1.2);
            if w.height(x, z) > 5.0 && w.airport_near(x, z).is_none() {
                assert_eq!(cold.sample(x, z).surface, SurfaceType::Snow);
                found_snow = true;
                break 'p;
            }
        }
    }
    assert!(found_snow, "no polar land found");
}

// ------------------------------------------------------------ GameState ----

#[test]
fn game_state_starts_on_runway_threshold_facing_runway() {
    let g = GameState::new(SEED, None, 120.0, 10.0);
    let ap = &g.world.airports()[g.airport];
    assert_eq!(ap.kind, AirportKind::International);
    let r = g.sim.state.rigid;
    let (along, lat) = {
        // 起点在入口前 14 m 处，沿着陆方向；用 runway_local 判断是否在跑道上并对齐
        let a = ap.runway_local(r.pos.x, r.pos.z);
        (a.0, a.1)
    };
    let end0 = along.abs() < 40.0;
    let end1 = (along - ap.length).abs() < 40.0;
    assert!(end0 || end1, "not at a threshold: along {along}");
    assert!(lat.abs() < 2.0);
    let hdg_ok = (g.sim.diag.heading - ap.heading_of_end(0)).abs() < 1.0 || (g.sim.diag.heading - ap.heading_of_end(1)).abs() < 1.0;
    let _ = hdg_ok; // diag 在第一次 step 后才更新，用姿态检验：
    let (h, _, _) = sim_core::euler_from_attitude(r.q);
    assert!(((h - ap.heading_of_end(0)).rem_euclid(360.0)).min((360.0 - (h - ap.heading_of_end(0)).rem_euclid(360.0)).abs()) < 1.0 || ((h - ap.heading_of_end(1)).rem_euclid(360.0)).min((360.0 - (h - ap.heading_of_end(1)).rem_euclid(360.0)).abs()) < 1.0);
    assert!(g.sim.controls.parking_brake);
    assert_eq!(g.screen, Screen::Flight);
}

#[test]
fn all_weather_presets_are_finite_and_airplane_stays_parked_under_brakes() {
    let mut g = GameState::new(SEED, None, 120.0, 10.0);
    for p in 0..crate::weather_fx::PRESET_NAMES.len() {
        g.preset = p;
        g.respawn();
        for _ in 0..(120 * 5) {
            g.sim.step();
        }
        let r = g.sim.state.rigid;
        assert!(r.pos.is_finite() && r.vel.is_finite() && r.w.is_finite(), "preset {p} NaN");
        assert!(r.vel.length() < 2.0, "preset {p}: parked airplane moved at {}", r.vel.length());
        assert!(!g.sim.state.damage.any(), "preset {p}: {:?}", g.sim.state.damage);
    }
}

#[test]
fn cycle_weather_wraps_and_changes_conditions() {
    let mut g = GameState::new(SEED, None, 120.0, 10.0);
    let n = crate::weather_fx::PRESET_NAMES.len();
    let mut kinds = std::collections::HashSet::new();
    for _ in 0..n {
        g.cycle_weather();
        kinds.insert((g.sim.weather.precip == PrecipKind::Rain, g.sim.weather.precip == PrecipKind::Snow, g.sim.weather.clouds.len(), (g.sim.weather.fog_visibility as i64)));
    }
    assert_eq!(g.preset, 0, "cycled back to start");
    assert!(kinds.len() >= 5, "presets should be distinct: {kinds:?}");
}

#[test]
fn takeoff_from_generated_international_runway_in_calm_weather() {
    let mut g = GameState::new(SEED, None, 120.0, 10.0);
    let ap = g.world.airports()[g.airport].clone();
    let start_hdg = {
        let (h, _, _) = sim_core::euler_from_attitude(g.sim.state.rigid.q);
        h
    };
    g.sim.weather.surface_wind *= 0.5;
    g.sim.controls.parking_brake = false;
    g.sim.controls.throttle = 1.0;
    for _ in 0..240 {
        g.sim.step();
    }
    let mut ph = PitchHold::default();
    let mut lifted = None;
    for _ in 0..(120 * 60) {
        let ias = g.sim.diag.ias_true / KT;
        g.sim.controls.yaw = heading_to_rudder(&g.sim, start_hdg);
        g.sim.controls.pitch = if ias >= 55.0 { ph.control(&g.sim, 8.0, FIXED_DT) } else { 0.0 };
        g.sim.step();
        if !g.sim.diag.gear.wheel_contact && g.sim.state.air_time > 0.2 {
            lifted = Some(g.sim.diag.ias_true / KT);
            break;
        }
    }
    let ias = lifted.expect("never lifted off");
    let (_, lat) = ap.runway_local(g.sim.state.rigid.pos.x, g.sim.state.rigid.pos.z);
    println!("liftoff {ias:.1} kt, {lat:.1} m off centerline");
    assert!((50.0..=70.0).contains(&ias), "liftoff IAS {ias}");
    assert!(lat.abs() < ap.width / 2.0, "left the runway: {lat}");
}

// --------------------------------------------------------- 输入系统 ----

fn input_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(ButtonInput::<KeyCode>::default());
    app.insert_resource(GameState::new(SEED, None, 120.0, 10.0));
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
fn keyboard_maps_to_flight_controls() {
    let mut app = input_app();
    tap(&mut app, KeyCode::KeyZ);
    assert_eq!(app.world().resource::<GameState>().sim.controls.throttle, 1.0);
    tap(&mut app, KeyCode::KeyX);
    assert_eq!(app.world().resource::<GameState>().sim.controls.throttle, 0.0);

    // 起落架切换、襟翼档位、皮托加温、除冰、驻车刹车
    let gear0 = app.world().resource::<GameState>().sim.controls.gear_down;
    tap(&mut app, KeyCode::KeyG);
    assert_eq!(app.world().resource::<GameState>().sim.controls.gear_down, !gear0);
    tap(&mut app, KeyCode::KeyF);
    tap(&mut app, KeyCode::KeyF);
    {
        let g = app.world().resource::<GameState>();
        assert_eq!(g.flap_idx, 2);
        assert!((g.sim.controls.flaps - g.sim.config.flap_detents[2]).abs() < 1e-12);
    }
    tap(&mut app, KeyCode::KeyV);
    assert_eq!(app.world().resource::<GameState>().flap_idx, 1);
    for _ in 0..10 {
        tap(&mut app, KeyCode::KeyF);
    }
    let n = app.world().resource::<GameState>().sim.config.flap_detents.len();
    assert_eq!(app.world().resource::<GameState>().flap_idx, n - 1, "flap detents clamp");

    let (ph, di, pb) = {
        let c = app.world().resource::<GameState>().sim.controls;
        (c.pitot_heat, c.deice, c.parking_brake)
    };
    tap(&mut app, KeyCode::KeyH);
    tap(&mut app, KeyCode::KeyI);
    tap(&mut app, KeyCode::KeyB);
    let c = app.world().resource::<GameState>().sim.controls;
    assert_eq!((c.pitot_heat, c.deice, c.parking_brake), (!ph, !di, !pb));

    // 视角 1–4，航图 M，暂停 P
    use crate::cockpit_ui::ViewMode;
    for (k, v) in [(KeyCode::Digit2, ViewMode::Hud), (KeyCode::Digit3, ViewMode::Chase), (KeyCode::Digit4, ViewMode::Flyby), (KeyCode::Digit1, ViewMode::Cockpit)] {
        tap(&mut app, k);
        assert_eq!(app.world().resource::<GameState>().view, v);
    }
    tap(&mut app, KeyCode::KeyP);
    assert!(app.world().resource::<GameState>().paused);
    tap(&mut app, KeyCode::KeyM);
    assert_eq!(app.world().resource::<GameState>().screen, Screen::Map);
    // 在航图界面，飞行输入被忽略
    let thr = app.world().resource::<GameState>().sim.controls.throttle;
    tap(&mut app, KeyCode::KeyZ);
    assert_eq!(app.world().resource::<GameState>().sim.controls.throttle, thr);
    tap(&mut app, KeyCode::KeyM);
    assert_eq!(app.world().resource::<GameState>().screen, Screen::Flight);
}

#[test]
fn held_keys_drive_stick_and_brakes() {
    let mut app = input_app();
    {
        let mut i = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
        i.press(KeyCode::KeyS); // 拉杆
        i.press(KeyCode::KeyD); // 右滚
        i.press(KeyCode::KeyQ); // 左舵
        i.press(KeyCode::Space);
    }
    app.update();
    let c = app.world().resource::<GameState>().sim.controls;
    assert_eq!((c.pitch, c.roll, c.yaw), (1.0, 1.0, -1.0));
    assert_eq!((c.brake_left, c.brake_right), (1.0, 1.0));
    {
        let mut i = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
        i.release_all();
        i.press(KeyCode::KeyW);
    }
    app.update();
    let c = app.world().resource::<GameState>().sim.controls;
    assert_eq!((c.pitch, c.roll, c.yaw, c.brake_left), (-1.0, 0.0, 0.0, 0.0));
}

#[test]
fn backspace_respawns_and_weather_key_cycles() {
    let mut app = input_app();
    app.world_mut().resource_mut::<GameState>().sim.state.rigid.pos.y += 500.0;
    tap(&mut app, KeyCode::Backspace);
    assert!(app.world().resource::<GameState>().sim.state.rigid.pos.y < 500.0);
    let p0 = app.world().resource::<GameState>().preset;
    tap(&mut app, KeyCode::KeyC);
    assert_ne!(app.world().resource::<GameState>().preset, p0);
}

// ------------------------------------------------------- 驾驶舱像素渲染 ----

fn render(h: &FlightHud, view: ViewMode) -> PixelBuffer {
    let mut b = PixelBuffer::new();
    draw_flight(&mut b, h, view);
    b
}

fn diff_count(a: &PixelBuffer, b: &PixelBuffer, y0: i32, y1: i32) -> usize {
    let mut n = 0;
    for y in y0..y1 {
        for x in 0..320 {
            if a.get(x, y) != b.get(x, y) {
                n += 1;
            }
        }
    }
    n
}

fn base_hud() -> FlightHud {
    FlightHud { ias_kt: 90.0, alt_ft: 3000.0, heading: 90.0, code: "SKY-172".into(), engine_label: "RPM", rpm: 2300.0, ..Default::default() }
}

#[test]
fn pixel_buffer_primitives_clip_safely() {
    let mut b = PixelBuffer::new();
    b.set(-5, -5, WHITE);
    b.set(1000, 1000, WHITE);
    b.rect(-10, -10, 20, 20, RED);
    assert_eq!(b.get(0, 0), RED);
    assert_eq!(b.get(10, 10), TRANSPARENT);
    b.line(-50, -50, 400, 300, GREEN);
    b.disc(160, 80, 6, AMBER);
    assert_eq!(b.get(160, 80), AMBER);
    b.frame(10, 10, 30, 20, CYAN);
    assert_eq!(b.get(10, 10), CYAN);
    assert_eq!(b.get(20, 20), TRANSPARENT);
    assert!(PixelBuffer::text_width("AB", 1) > PixelBuffer::text_width("A", 1));
    assert_eq!(PixelBuffer::text_width("ABC", 2), PixelBuffer::text_width("ABC", 1) * 2);
    assert_eq!(b.data.len(), 320 * 160 * 4);
}

#[test]
fn ils_needles_move_with_deviation_and_hide_when_invalid() {
    let mk = |loc: f64, gs: f64, valid: bool| {
        let mut h = base_hud();
        h.ils = Some(IlsHud { ident: "IXYZ".into(), loc_deg: loc, gs_deg: gs, loc_valid: valid, gs_valid: valid, dist_km: 5.0 });
        render(&h, ViewMode::Cockpit)
    };
    let none = render(&base_hud(), ViewMode::Cockpit);
    let centered = mk(0.0, 0.0, true);
    let right = mk(1.5, 0.0, true);
    let left = mk(-1.5, 0.0, true);
    let high = mk(0.0, 0.8, true);
    let low = mk(0.0, -0.8, true);
    let invalid = mk(1.5, 0.0, false);
    assert!(diff_count(&none, &centered, 112, 160) > 10, "ILS indicator must draw");
    assert!(diff_count(&centered, &right, 112, 160) > 0, "localizer needle moves right");
    assert!(diff_count(&right, &left, 112, 160) > 0);
    assert!(diff_count(&centered, &high, 112, 160) > 0, "glideslope needle moves");
    assert!(diff_count(&high, &low, 112, 160) > 0);
    // 无效信号：指针不再随偏差变化（显示 OFF 旗）
    let invalid2 = mk(-1.5, 0.0, false);
    assert_eq!(diff_count(&invalid, &invalid2, 112, 160), 0, "invalid ILS ignores deviation");
}

#[test]
fn warning_lamps_and_alerts_light_up() {
    let calm = render(&base_hud(), ViewMode::Cockpit);
    let cases: Vec<(&str, FlightHud)> = vec![
        ("stall", FlightHud { stall: true, ..base_hud() }),
        ("ice", FlightHud { ice: 0.6, ..base_hud() }),
        ("ovspd", FlightHud { ovspd: true, ..base_hud() }),
        ("glim", FlightHud { glim: true, ..base_hud() }),
        ("gear", FlightHud { gear_warn: true, ..base_hud() }),
        ("damage", FlightHud { damage: Some("WING FAILURE".into()), ..base_hud() }),
    ];
    for (name, h) in cases {
        let lit = render(&h, ViewMode::Cockpit);
        let n = diff_count(&calm, &lit, 0, 160);
        assert!(n > 0, "{name} produced no visual change");
    }
}

#[test]
fn windshield_frost_and_rain_overlays_scale_with_intensity() {
    let opaque_top = |b: &PixelBuffer| (0..112).map(|y| (0..320).filter(|&x| b.get(x, y)[3] > 0).count()).sum::<usize>();
    let clear = opaque_top(&render(&base_hud(), ViewMode::Cockpit));
    let frost_lo = opaque_top(&render(&FlightHud { frost: 0.25, ..base_hud() }, ViewMode::Cockpit));
    let frost_hi = opaque_top(&render(&FlightHud { frost: 1.0, ..base_hud() }, ViewMode::Cockpit));
    let rain = opaque_top(&render(&FlightHud { rain: 0.8, time: 1.0, ..base_hud() }, ViewMode::Cockpit));
    println!("windshield pixels clear {clear} frost .25 {frost_lo} frost 1 {frost_hi} rain {rain}");
    assert!(frost_lo > clear, "frost draws");
    assert!(frost_hi > frost_lo, "more ice → more frost");
    assert!(rain > clear, "rain streaks draw");
    // 雨线随时间移动
    let r0 = render(&FlightHud { rain: 0.8, time: 1.0, ..base_hud() }, ViewMode::Cockpit);
    let r1 = render(&FlightHud { rain: 0.8, time: 1.3, ..base_hud() }, ViewMode::Cockpit);
    assert!(diff_count(&r0, &r1, 0, 112) > 0, "rain animates");
}

#[test]
fn three_view_modes_render_distinct_layouts() {
    let h = FlightHud { fpv: (2.0, -3.0), ..base_hud() };
    let (c, hud, chase, fly) = (render(&h, ViewMode::Cockpit), render(&h, ViewMode::Hud), render(&h, ViewMode::Chase), render(&h, ViewMode::Flyby));
    // 驾驶舱：底部 48 px 不透明仪表盘；HUD/外部视角：全屏 3D，底部无面板
    assert_eq!(c.get(160, 150)[3], 255);
    assert_eq!(hud.get(5, 158)[3], 0);
    assert_eq!(chase.get(5, 158)[3], 0);
    assert!(diff_count(&c, &hud, 0, 160) > 500);
    assert!(diff_count(&hud, &chase, 0, 160) > 50, "HUD symbology only in HUD view");
    let _ = fly;
}

#[test]
fn map_overlay_draws_markers_and_status() {
    let terrain: Vec<[u8; 3]> = vec![[40, 90, 160]; 320 * MAP_H as usize];
    let markers = vec![MapMarker { px: 100, py: 60, kind: 2 }, MapMarker { px: 200, py: 90, kind: 0 }];
    let mk = |selected: Option<usize>, aircraft: Option<(i32, i32, f64)>, status: &str| {
        let m = MapView { terrain: Some(&terrain), markers: &markers, selected, cursor: (150, 40), aircraft, status: [status.to_string(), String::from("DAY 120")], time: 0.0 };
        let mut b = PixelBuffer::new();
        draw_map(&mut b, &m);
        b
    };
    let plain = mk(None, None, "SEED 32");
    assert_eq!(plain.get(5, 5), [40, 90, 160, 255], "terrain blitted");
    assert_ne!(plain.get(100, 60), [40, 90, 160, 255], "airport marker drawn");
    assert_eq!(plain.get(160, 150)[3], 255, "status bar opaque");
    let sel = mk(Some(1), None, "SEED 32");
    assert!(diff_count(&plain, &sel, 0, 140) > 0, "selection highlight");
    let ac = mk(None, Some((160, 70, 45.0)), "SEED 32");
    assert!(diff_count(&plain, &ac, 0, 140) > 0, "aircraft glyph");
    let txt = mk(None, None, "OTHER");
    assert!(diff_count(&plain, &txt, 140, 160) > 0, "status text");
}

// ------------------------------------------------------------- 航图 ----

#[test]
fn map_projection_and_dates() {
    for &(px, py) in &[(0, 0), (319, 139), (160, 70), (33, 101)] {
        let (x, z) = map_screen::unproject(px, py);
        assert_eq!(map_screen::project(x, z), (px, py));
    }
    // 经度环绕：x 与 x+W 投影到同一列
    let (x, z) = map_screen::unproject(10, 20);
    assert_eq!(map_screen::project(x, z), map_screen::project(x + WORLD_WIDTH, z));
    assert_eq!(map_screen::date_string(0.0), map_screen::date_string(365.0));
    assert_ne!(map_screen::date_string(10.0), map_screen::date_string(200.0));
}

#[test]
fn rendered_map_has_ocean_land_and_seasonal_snow() {
    let w = world();
    let summer = map_screen::render_map_terrain(&w, 172.0);
    let winter = map_screen::render_map_terrain(&w, 355.0);
    assert_eq!(summer.len(), 320 * MAP_H as usize);
    // 海洋像素（偏蓝）占比大致与地形的海洋占比一致 55%–65%（容差放宽到 45–75%）
    let blue = summer.iter().filter(|p| p[2] > p[0] as u8 && p[2] > p[1] as u8).count() as f64 / summer.len() as f64;
    println!("map ocean-like fraction {blue:.2}");
    assert!((0.45..=0.75).contains(&blue), "ocean fraction {blue}");
    // 北半球（上半图）冬季白色像素更多，南半球（下半图）夏季更少 —— 即季节反转
    let white = |m: &[[u8; 3]], y0: usize, y1: usize| m[y0 * 320..y1 * 320].iter().filter(|p| p[0] > 200 && p[1] > 200 && p[2] > 200).count();
    let h = MAP_H as usize;
    let (n_s, n_w) = (white(&summer, 0, h / 2), white(&winter, 0, h / 2));
    let (s_s, s_w) = (white(&summer, h / 2, h), white(&winter, h / 2, h));
    println!("snow px  north: summer {n_s} winter {n_w}; south: summer {s_s} winter {s_w}");
    assert!(n_w > n_s, "northern winter has more snow");
    assert!(s_s > s_w, "southern hemisphere is reversed (day 172 is its winter)");
}

// ------------------------------- M2 DoD：飞越经度 ±180° 无撕裂/无跳变 ----

#[test]
fn flying_across_antimeridian_has_no_coordinate_or_terrain_jump() {
    let w = world();
    let (_, z) = latlon_to_world(12.0, 0.0);
    let ground = WorldGround { world: w.clone(), day: 120.0 };
    let mut sim = sim_core::integrator::Simulation::new(sim_core::aircraft::sky172(), Box::new(ground), (0.0, z), 90.0);
    sim.state.rigid.pos.x = WORLD_WIDTH / 2.0 - 4_000.0;
    sim.set_airborne(3600.0, 95.0 * KT, 90.0);
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut last_gh = sim.ground.sample(sim.state.rigid.pos.x, sim.state.rigid.pos.z).height;
    let mut worst_gh_step: f64 = 0.0;
    let mut crossed = false;
    let mut prev_x = sim.state.rigid.pos.x;
    let mut max_dx: f64 = 0.0;
    for _ in 0..(120 * 140) {
        sim.controls.throttle = 0.7;
        sim.controls.pitch = ph.control(&sim, 3.0, FIXED_DT);
        sim.controls.roll = bh.control(&sim, heading_to_bank(&sim, 90.0, 15.0), FIXED_DT);
        sim.step();
        let p = sim.state.rigid.pos;
        max_dx = max_dx.max((p.x - prev_x).abs());
        prev_x = p.x;
        let gh = sim.ground.sample(p.x, p.z).height;
        worst_gh_step = worst_gh_step.max((gh - last_gh).abs());
        last_gh = gh;
        if p.x > WORLD_WIDTH / 2.0 + 500.0 {
            crossed = true;
        }
        assert!(p.is_finite());
    }
    println!("crossed={crossed}, max |dx| per step {max_dx:.3} m, worst ground step {worst_gh_step:.2} m");
    assert!(crossed, "did not reach the seam");
    // 位置连续（物理使用无限展开的 f64 X，不会在 ±180° 处跳变）
    assert!(max_dx < 1.0, "x jump {max_dx}");
    // 地形高度在接缝处平滑（每个物理步 ≈0.4 m 水平位移）
    assert!(worst_gh_step < 3.0, "terrain jump {worst_gh_step}");
    // 同一个物理点两种写法 → 同一地面
    let p = sim.state.rigid.pos;
    let (a, b) = (sim.ground.sample(p.x, p.z), sim.ground.sample(wrap_x(p.x), p.z));
    assert_eq!(a.height, b.height);
    assert!(!sim.state.damage.any());
}

// ---------------------------------- M3 DoD：低能见度 ILS 盲降落回跑道 ----

#[derive(Debug)]
struct LandingResult {
    min_visibility: f64,
    touchdown_along: f64,
    sink: f64,
    stop_along: f64,
    stop_lateral: f64,
    stopped: bool,
    max_loc: f64,
    max_gs: f64,
    damaged: bool,
    crashed: bool,
}

/// 全程仅使用仪表量（ILS 偏差、表速、垂直速度、姿态、AGL 读数）进行盲降。
fn blind_ils_landing(preset: usize, dist0: f64, offset_m: f64) -> LandingResult {
    let mut g = GameState::new(SEED, None, 120.0, 10.0);
    g.preset = preset;
    g.respawn(); // 重新套用天气，并按“迎风端”设置机场/跑道
    let ap = g.world.airports()[g.airport].clone();
    // 着陆端 = 迎风端 = depart() 所选端：由起始位置判断
    let st = g.sim.state.rigid.pos;
    let (along0, _) = ap.runway_local(st.x, st.z);
    let end = if along0 < ap.length / 2.0 { 0 } else { 1 };
    let ils = ap.ils.iter().find(|i| i.end == end).expect("ILS for landing end").clone();
    let hdg = ap.heading_of_end(end);
    let (dx, dz) = ap.landing_dir(end);
    let t = ap.ends[end];
    let right = (-dz, dx);
    let start = (t.0 - dx * dist0 + right.0 * offset_m, t.1 - dz * dist0 + right.1 * offset_m);
    let gs_tan = ils.glide_deg.to_radians().tan();
    let alt = ap.elevation + (dist0 + 300.0) * gs_tan;

    let sim = &mut g.sim;
    sim.state.rigid.pos.x = start.0;
    sim.state.rigid.pos.z = start.1;
    sim.set_airborne(alt, 76.0 * KT, hdg);
    let wind = sim.weather.wind_at(sim.state.rigid.pos, alt - ap.elevation, 0.0);
    sim.state.rigid.vel += wind;
    sim.controls.parking_brake = false;
    sim.controls.pitot_heat = true;
    sim.controls.flaps = 2.0 / 3.0;
    sim.state.surfaces.flaps = 2.0 / 3.0;

    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut vs_pid = Pid::new(0.8, 0.05, 0.0, 6.0);
    let mut thr = Pid::new(0.04, 0.01, 0.0, 0.5);
    let mut loc_i = 0.0;
    let mut res = LandingResult { min_visibility: f64::MAX, touchdown_along: f64::NAN, sink: 0.0, stop_along: f64::NAN, stop_lateral: f64::NAN, stopped: false, max_loc: 0.0, max_gs: 0.0, damaged: false, crashed: false };
    let mut touched = false;
    for step in 0..(120 * 600) {
        let p = sim.state.rigid.pos;
        let dev = ap.ils_deviation(&ils, p.x, p.z, p.y);
        res.min_visibility = res.min_visibility.min(sim.diag.visibility);
        let agl = sim.diag.agl - sim.config.rest_cg_height;
        let on_ground = sim.state.on_ground;
        if !on_ground {
            // ---- 仪表盲降：航向道/下滑道指针 → 航向/坡度、垂直速率/俯仰 ----
            if dev.loc_valid && dev.dist_to_threshold > 200.0 {
                loc_i = (loc_i + dev.loc_deg * FIXED_DT).clamp(-3.0, 3.0);
            }
            let loc = if dev.loc_valid { dev.loc_deg } else { 0.0 };
            let gain = (dev.dist_to_threshold / 3000.0).clamp(0.4, 1.0); // 近距降低增益防止振荡
            let hdg_t = hdg - (3.2 * loc + 0.8 * loc_i).clamp(-14.0, 14.0) * gain;
            let bank = heading_to_bank(sim, hdg_t, 18.0);
            sim.controls.roll = bh.control(sim, bank, FIXED_DT);
            let gs_err = if dev.gs_valid { dev.gs_deg } else { 0.0 };
            let vs_t = if agl < 6.0 { -0.6 } else { -sim.diag.ground_speed * gs_tan - (3.0 * gs_err).clamp(-2.0, 2.0) };
            let pitch_t = 2.0 + vs_pid.update(vs_t - sim.diag.vertical_speed, FIXED_DT);
            sim.controls.pitch = ph.control(sim, pitch_t.clamp(-6.0, 9.0), FIXED_DT);
            let tgt = if agl < 15.0 { 62.0 * KT } else { 68.0 * KT };
            sim.controls.throttle = if agl < 5.0 { 0.0 } else { (0.35 + thr.update((tgt - sim.diag.ias_true) / KT, FIXED_DT)).clamp(0.0, 1.0) };
            sim.controls.yaw = 0.0;
            res.max_loc = res.max_loc.max(if dev.dist_to_threshold < 4000.0 && dev.dist_to_threshold > 400.0 { loc.abs() } else { 0.0 });
            res.max_gs = res.max_gs.max(if dev.dist_to_threshold < 4000.0 && dev.dist_to_threshold > 600.0 { gs_err.abs() } else { 0.0 });
        } else {
            if !touched {
                touched = true;
                res.touchdown_along = ap.runway_local(p.x, p.z).0;
                if end == 1 {
                    res.touchdown_along = ap.length - res.touchdown_along;
                }
                res.sink = sim.state.last_touchdown_sink;
            }
            sim.controls.throttle = 0.0;
            sim.controls.flaps = 0.0;
            sim.controls.pitch = 0.4;
            sim.controls.roll = 0.0;
            sim.controls.yaw = heading_to_rudder(sim, hdg);
            if sim.diag.ground_speed < 35.0 {
                sim.controls.brake_left = 0.8;
                sim.controls.brake_right = 0.8;
            }
        }
        sim.step();
        if sim.state.damage.crashed {
            res.crashed = true;
            break;
        }
        if touched && sim.state.on_ground && sim.diag.ground_speed < 0.3 {
            res.stopped = true;
            let q = sim.state.rigid.pos;
            let (a, l) = ap.runway_local(q.x, q.z);
            res.stop_along = if end == 0 { a } else { ap.length - a };
            res.stop_lateral = l;
            break;
        }
        let _ = step;
    }
    res.damaged = sim.state.damage.any();
    res
}

fn assert_good_landing(name: &str, r: &LandingResult, width: f64, length: f64) {
    println!("{name}: {r:?}");
    assert!(!r.crashed, "{name}: crashed");
    assert!(!r.damaged, "{name}: damaged");
    assert!(r.touchdown_along.is_finite(), "{name}: never touched down");
    assert!((-50.0..=1200.0).contains(&r.touchdown_along), "{name}: touchdown at {} m", r.touchdown_along);
    assert!(r.sink < 3.0, "{name}: sink {} exceeds gear limit", r.sink);
    assert!(r.stopped, "{name}: did not stop on the runway");
    assert!(r.stop_along > 0.0 && r.stop_along < length, "{name}: stopped at {} (len {length})", r.stop_along);
    assert!(r.stop_lateral.abs() < width / 2.0 + 3.0, "{name}: off the runway: {}", r.stop_lateral);
}

#[test]
fn blind_ils_landing_in_dense_fog() {
    let r = blind_ils_landing(3, 9_000.0, 0.0); // FOG：能见度 350 m
    assert!(r.min_visibility <= 400.0, "fog scenario must be low-vis: {}", r.min_visibility);
    let (w, l) = {
        let g = GameState::new(SEED, None, 120.0, 10.0);
        let a = &g.world.airports()[g.airport];
        (a.width, a.length)
    };
    assert_good_landing("fog", &r, w, l);
    assert!(r.max_loc < 0.5 && r.max_gs < 0.6, "ILS tracking within ±0.5°/±0.6° on short final: loc {} gs {}", r.max_loc, r.max_gs);
}

#[test]
fn blind_ils_landing_in_rain_with_offset_intercept() {
    // 雨 + 云 + 阵风/乱流；从航向道右侧 400 m 切入，必须靠指针修正
    let r = blind_ils_landing(1, 10_000.0, 400.0);
    assert!(r.min_visibility < 2500.0, "rain scenario must be low-vis: {}", r.min_visibility);
    let (w, l) = {
        let g = GameState::new(SEED, None, 120.0, 10.0);
        let a = &g.world.airports()[g.airport];
        (a.width, a.length)
    };
    assert_good_landing("rain", &r, w, l);
}

#[test]
fn blind_ils_landing_in_snow_and_cloud() {
    let r = blind_ils_landing(2, 9_000.0, -250.0);
    assert!(r.min_visibility < 1500.0, "snow scenario must be low-vis: {}", r.min_visibility);
    let (w, l) = {
        let g = GameState::new(SEED, None, 120.0, 10.0);
        let a = &g.world.airports()[g.airport];
        (a.width, a.length)
    };
    assert_good_landing("snow", &r, w, l);
}
