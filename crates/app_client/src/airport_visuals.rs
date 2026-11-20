//! 程序化机场：跑道标线、边灯、PAPI 灯、风向袋、低模塔台/机库。
//! 机场实体锚定在 f64 世界坐标，由 Floating Origin 同步到渲染坐标。

use crate::mesh_util::{rgba, MeshBuilder};
use crate::origin::WorldAnchor;
use crate::GameState;
use bevy::math::DVec3;
use bevy::prelude::*;
use std::collections::HashMap;
use world_gen::airport::Airport;
use world_gen::coords::wrapped_dx;

const SPAWN_RADIUS: f64 = 38_000.0;
const DESPAWN_RADIUS: f64 = 55_000.0;

#[derive(Component)]
pub struct AirportRoot {
    pub index: usize,
    /// 机场锚点（f64 世界坐标，已展开经度环绕）。
    pub anchor: DVec3,
}

#[derive(Component)]
pub struct PapiLight {
    pub index: usize,
    pub end: usize,
    pub slot: usize,
    pub local: Vec3,
    pub anchor: DVec3,
}

#[derive(Component)]
pub struct Windsock;

#[derive(Resource, Default)]
pub struct AirportVisuals {
    spawned: HashMap<usize, Entity>,
}

#[derive(Resource)]
pub struct AirportAssets {
    lit: Handle<StandardMaterial>,
    unlit: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
    red: Handle<StandardMaterial>,
    cube: Handle<Mesh>,
    sock: Handle<Mesh>,
}

fn setup_assets(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let lit = mats.add(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 1.0, reflectance: 0.05, ..default() });
    let unlit = mats.add(StandardMaterial { base_color: Color::WHITE, unlit: true, ..default() });
    let white = mats.add(StandardMaterial { base_color: Color::srgb(1.0, 1.0, 0.95), unlit: true, ..default() });
    let red = mats.add(StandardMaterial { base_color: Color::srgb(1.0, 0.1, 0.05), unlit: true, ..default() });
    let cube = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    // 风向袋：沿局部 -Z 方向的锥
    let mut sb = MeshBuilder::new();
    let (len, r) = (3.5f32, 0.7f32);
    let n = 6;
    for i in 0..n {
        let (a0, a1) = (i as f32 / n as f32 * std::f32::consts::TAU, (i + 1) as f32 / n as f32 * std::f32::consts::TAU);
        let c = if i % 2 == 0 { [1.0, 0.45, 0.05, 1.0] } else { [1.0, 1.0, 1.0, 1.0] };
        let (p0, p1) = (Vec3::new(a0.cos() * r, a0.sin() * r, 0.0), Vec3::new(a1.cos() * r, a1.sin() * r, 0.0));
        let tip = Vec3::new(0.0, 0.0, -len);
        let small = 0.25;
        let (q0, q1) = (Vec3::new(a0.cos() * small, a0.sin() * small, -len), Vec3::new(a1.cos() * small, a1.sin() * small, -len));
        let _ = tip;
        sb.quad(p0, q0, q1, p1, c);
        sb.quad(p1, q1, q0, p0, c);
    }
    let sock = meshes.add(sb.build());
    commands.insert_resource(AirportAssets { lit, unlit, white, red, cube, sock });
}

/// 跑道局部点（相对机场中心；x 东，z 南），沿跑道距离 `along`（自端 0），横向 `lat`（右为正）。
fn lp(ap: &Airport, along: f64, lat: f64, y: f32) -> Vec3 {
    let (dx, dz) = ap.dir0();
    let a = along - ap.length / 2.0;
    Vec3::new((dx * a - dz * lat) as f32, y, (dz * a + dx * lat) as f32)
}

fn flat_quad(mb: &mut MeshBuilder, ap: &Airport, a0: f64, a1: f64, l0: f64, l1: f64, y: f32, col: [f32; 4]) {
    // 顺序保证法线朝上（+Y）
    let (p00, p10, p11, p01) = (lp(ap, a0, l0, y), lp(ap, a1, l0, y), lp(ap, a1, l1, y), lp(ap, a0, l1, y));
    let n = (p10 - p00).cross(p01 - p00);
    if n.y >= 0.0 {
        mb.quad(p00, p10, p11, p01, col);
    } else {
        mb.quad(p00, p01, p11, p10, col);
    }
}

fn build_airport_meshes(ap: &Airport) -> (MeshBuilder, MeshBuilder) {
    let mut ground = MeshBuilder::new();
    let mut lights = MeshBuilder::new();
    let hw = ap.width / 2.0;
    let l = ap.length;
    let surf = if ap.asphalt { [0.15, 0.15, 0.17, 1.0] } else { [0.45, 0.35, 0.22, 1.0] };
    // 路面
    flat_quad(&mut ground, ap, 0.0, l, -hw, hw, 0.12, surf);
    // 土路跑道两侧压实带
    if !ap.asphalt {
        flat_quad(&mut ground, ap, -20.0, l + 20.0, -hw - 4.0, hw + 4.0, 0.08, [0.38, 0.45, 0.2, 1.0]);
    }
    let white = [0.95, 0.95, 0.95, 1.0];
    if ap.asphalt {
        // 中线虚线
        let mut a = 60.0;
        while a + 24.0 < l - 60.0 {
            flat_quad(&mut ground, ap, a, a + 24.0, -0.5, 0.5, 0.16, white);
            a += 48.0;
        }
        // 入口标线 & 接地区标线 & 边线
        for end in 0..2 {
            let s = if end == 0 { 1.0 } else { -1.0 };
            let base = if end == 0 { 0.0 } else { l };
            for k in 0..8 {
                let lat = -hw * 0.8 + k as f64 * (hw * 1.6 / 7.0);
                flat_quad(&mut ground, ap, base + s * 8.0, base + s * 38.0, lat - 0.8, lat + 0.8, 0.16, white);
            }
            for dist in [150.0, 300.0, 450.0] {
                for side in [-1.0, 1.0] {
                    flat_quad(&mut ground, ap, base + s * dist, base + s * (dist + 22.0), side * hw * 0.35 - 1.2, side * hw * 0.35 + 1.2, 0.16, white);
                }
            }
        }
        flat_quad(&mut ground, ap, 0.0, l, -hw + 0.5, -hw + 1.0, 0.16, white);
        flat_quad(&mut ground, ap, 0.0, l, hw - 1.0, hw - 0.5, 0.16, white);
        // 滑行道与停机坪
        let apron = [0.22, 0.22, 0.24, 1.0];
        flat_quad(&mut ground, ap, l * 0.5 - 90.0, l * 0.5 + 90.0, hw + 40.0, hw + 100.0, 0.1, apron);
        flat_quad(&mut ground, ap, l * 0.5 - 12.0, l * 0.5 + 12.0, hw, hw + 40.0, 0.1, apron);
    } else {
        flat_quad(&mut ground, ap, l * 0.5 - 40.0, l * 0.5 + 40.0, hw + 30.0, hw + 60.0, 0.1, [0.5, 0.4, 0.28, 1.0]);
    }
    // 塔台与机库
    let (tx, tz) = {
        let p = lp(ap, l * 0.5, hw + 110.0, 0.0);
        (p.x, p.z)
    };
    if ap.asphalt {
        ground.aabb(Vec3::new(tx, 7.0, tz), Vec3::new(2.5, 7.0, 2.5), [0.75, 0.75, 0.78, 1.0]);
        ground.aabb(Vec3::new(tx, 15.5, tz), Vec3::new(4.0, 1.8, 4.0), [0.3, 0.45, 0.55, 1.0]);
        ground.aabb(Vec3::new(tx, 17.8, tz), Vec3::new(4.6, 0.4, 4.6), [0.5, 0.5, 0.52, 1.0]);
        for k in 0..3 {
            let p = lp(ap, l * 0.5 - 60.0 + k as f64 * 60.0, hw + 150.0, 0.0);
            ground.aabb(Vec3::new(p.x, 5.0, p.z), Vec3::new(18.0, 5.0, 14.0), [0.55, 0.58, 0.6, 1.0]);
        }
    } else {
        let p = lp(ap, l * 0.5, hw + 45.0, 0.0);
        ground.aabb(Vec3::new(p.x, 3.0, p.z), Vec3::new(6.0, 3.0, 4.0), [0.5, 0.25, 0.2, 1.0]);
    }
    // 风向袋杆
    let sp = lp(ap, 250.0, hw + 40.0, 0.0);
    ground.aabb(Vec3::new(sp.x, 3.0, sp.z), Vec3::new(0.12, 3.0, 0.12), [0.9, 0.9, 0.9, 1.0]);

    // 边灯（60 m 间隔）、末端灯（红/绿）
    for (a, b) in ap.edge_lights().into_iter().zip(std::iter::repeat(())) {
        let _ = b;
        let p = Vec3::new(((a.0 - ap.center.0) as f32 + 0.0) * 0.0, 0.0, 0.0);
        let _ = p;
    }
    let n = (l / 60.0).floor() as i32;
    for i in 0..=n {
        let a = i as f64 * l / n as f64;
        let amber = a > l - 600.0 || a < 0.5;
        let c = if amber { [1.0, 0.8, 0.2, 1.0] } else { [1.0, 1.0, 0.95, 1.0] };
        for side in [-1.0, 1.0] {
            let p = lp(ap, a, side * (hw + 2.0), 0.55);
            lights.aabb(p, Vec3::splat(0.35), c);
        }
    }
    for k in -4..=4 {
        let lat = k as f64 * (ap.width / 8.0);
        lights.aabb(lp(ap, -1.0, lat, 0.4), Vec3::new(0.3, 0.3, 0.3), [0.1, 1.0, 0.2, 1.0]);
        lights.aabb(lp(ap, l + 1.0, lat, 0.4), Vec3::new(0.3, 0.3, 0.3), [1.0, 0.1, 0.1, 1.0]);
    }
    let _ = rgba;
    (ground, lights)
}

fn manage_airports(
    mut commands: Commands,
    game: Res<GameState>,
    assets: Option<Res<AirportAssets>>,
    mut vis: ResMut<AirportVisuals>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Some(assets) = assets else { return };
    let ac = game.sim.state.rigid.pos;
    let aps = game.world.airports();
    // 卸载
    let far: Vec<usize> = vis
        .spawned
        .keys()
        .copied()
        .filter(|&i| {
            let a = &aps[i];
            let dx = wrapped_dx(ac.x, a.center.0);
            dx.hypot(a.center.1 - ac.z) > DESPAWN_RADIUS
        })
        .collect();
    for i in far {
        if let Some(e) = vis.spawned.remove(&i) {
            commands.entity(e).despawn();
        }
    }
    // 加载
    for (i, a) in aps.iter().enumerate() {
        if vis.spawned.contains_key(&i) {
            continue;
        }
        let dx = wrapped_dx(ac.x, a.center.0);
        if dx.hypot(a.center.1 - ac.z) > SPAWN_RADIUS {
            continue;
        }
        let anchor = DVec3::new(ac.x + dx, a.elevation, a.center.1);
        let (g, l) = build_airport_meshes(a);
        let root = commands
            .spawn((Transform::default(), Visibility::default(), WorldAnchor(anchor), AirportRoot { index: i, anchor }))
            .id();
        let g_ent = commands.spawn((Mesh3d(meshes.add(g.build())), MeshMaterial3d(assets.lit.clone()), Transform::default())).id();
        let l_ent = commands.spawn((Mesh3d(meshes.add(l.build())), MeshMaterial3d(assets.unlit.clone()), Transform::default())).id();
        commands.entity(root).add_children(&[g_ent, l_ent]);
        // 风向袋
        let hw = a.width / 2.0;
        let sp = lp(a, 250.0, hw + 40.0, 6.2);
        let sock = commands.spawn((Mesh3d(assets.sock.clone()), MeshMaterial3d(assets.lit.clone()), Transform::from_translation(sp), Windsock)).id();
        commands.entity(root).add_children(&[sock]);
        // PAPI（两端各 4 盏）
        for end in 0..2usize {
            let papi = a.papi(end);
            for (slot, p) in papi.positions.iter().enumerate() {
                let (rx, rz) = (wrapped_dx(a.center.0, p.0), p.1 - a.center.1);
                let local = Vec3::new(rx as f32, 0.7, rz as f32);
                let e = commands
                    .spawn((
                        Mesh3d(assets.cube.clone()),
                        MeshMaterial3d(assets.white.clone()),
                        Transform::from_translation(local).with_scale(Vec3::new(1.4, 0.8, 1.4)),
                        PapiLight { index: i, end, slot, local, anchor },
                    ))
                    .id();
                commands.entity(root).add_children(&[e]);
            }
        }
        vis.spawned.insert(i, root);
    }
}

/// PAPI：以飞机相对灯位的仰角决定红/白（2.5°/2.83°/3.17°/3.5° 阈值，3° 下滑 = 2 红 2 白）。
fn update_papi(game: Res<GameState>, assets: Option<Res<AirportAssets>>, mut q: Query<(&PapiLight, &mut MeshMaterial3d<StandardMaterial>)>) {
    let Some(assets) = assets else { return };
    let ac = game.sim.state.rigid.pos;
    let aps = game.world.airports();
    for (p, mut m) in &mut q {
        let ap = &aps[p.index];
        let rel = ac - p.anchor;
        let v = Vec3::new(rel.x as f32, rel.y as f32, rel.z as f32) - p.local;
        let (ddx, ddz) = ap.landing_dir(p.end);
        let behind = -(v.x as f64 * ddx + v.z as f64 * ddz);
        let ang = if behind > 20.0 { (v.y as f64 - 0.7).atan2(behind).to_degrees() } else { 90.0 };
        let white = ang > 2.5 + 0.333 * p.slot as f64;
        let want = if white { assets.white.clone() } else { assets.red.clone() };
        if m.0 != want {
            m.0 = want;
        }
    }
}

fn update_windsock(game: Res<GameState>, mut q: Query<&mut Transform, With<Windsock>>) {
    let w = game.sim.weather.wind_at(game.sim.state.rigid.pos, 10.0, game.sim.state.time);
    let sp = (w.x * w.x + w.z * w.z).sqrt();
    let dir = if sp > 0.6 { Vec3::new(w.x as f32, -(0.5 - (sp as f32 / 12.0).min(0.5)) * 1.2, w.z as f32).normalize() } else { Vec3::NEG_Y };
    for mut t in &mut q {
        t.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, dir);
    }
}

pub struct AirportVisualsPlugin;
impl Plugin for AirportVisualsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AirportVisuals>()
            .add_systems(Startup, setup_assets)
            .add_systems(Update, (manage_airports, update_papi, update_windsock));
    }
}

/// 清空所有机场实体（切换世界/种子时使用）。
pub fn reset_airports(commands: &mut Commands, vis: &mut AirportVisuals) {
    for (_, e) in vis.spawned.drain() {
        commands.entity(e).despawn();
    }
}
