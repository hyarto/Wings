//! 5 种机型的程序化低多边形 3D 模型（外部视角可见）：数据驱动（由 `AircraftConfig` 的几何生成），
//! 含舵面偏转、螺旋桨旋转、起落架收放、涡扇短舱、水上机船体与浮筒。SKY-172 保留手工造型。

use crate::mesh_util::MeshBuilder;
use crate::origin::FloatingOrigin;
use crate::GameState;
use bevy::math::DVec3;
use bevy::prelude::*;
use sim_core::aircraft::{AircraftConfig, ContactConfig, ContactKind, EngineKind, SurfaceConfig, SurfaceKind};

#[derive(Component)]
pub struct AircraftRoot;
/// 螺旋桨（对应第 N 台发动机，用其转速驱动旋转）。
#[derive(Component)]
pub struct Propeller(pub usize);
/// 可收放起落架部件（收起时隐藏）。
#[derive(Component)]
pub struct GearPart;
/// 当前已生成的机型代号（切换机型时重建模型）。
#[derive(Resource, Default)]
pub struct CurrentModel(pub String);
#[derive(Component)]
pub enum ControlSurface {
    AileronL,
    AileronR,
    Elevator,
    Rudder,
}

const WHITE: [f32; 4] = [0.93, 0.93, 0.9, 1.0];
const RED: [f32; 4] = [0.78, 0.1, 0.1, 1.0];
const DARK: [f32; 4] = [0.12, 0.12, 0.14, 1.0];
const GLASS: [f32; 4] = [0.25, 0.4, 0.55, 1.0];

fn body_mesh() -> Mesh {
    let mut m = MeshBuilder::new();
    // 机身：驾驶舱 + 机头 + 尾锥
    m.aabb(Vec3::new(0.0, 0.0, 0.2), Vec3::new(0.55, 0.62, 1.5), WHITE);
    m.aabb(Vec3::new(0.0, 0.7, -0.3), Vec3::new(0.5, 0.18, 0.7), GLASS);
    m.aabb(Vec3::new(0.0, -0.05, -1.9), Vec3::new(0.46, 0.5, 0.8), WHITE);
    m.aabb(Vec3::new(0.0, -0.0, -2.75), Vec3::new(0.3, 0.3, 0.1), DARK);
    m.aabb(Vec3::new(0.0, 0.08, 2.45), Vec3::new(0.38, 0.42, 1.0), WHITE);
    m.aabb(Vec3::new(0.0, 0.2, 4.0), Vec3::new(0.2, 0.25, 1.1), WHITE);
    m.aabb(Vec3::new(0.0, 0.1, 1.2), Vec3::new(0.56, 0.1, 3.5), RED); // 腰线
    // 主翼（高翼）+ 撑杆
    m.aabb(Vec3::new(0.0, 0.82, 0.1), Vec3::new(5.5, 0.07, 0.75), WHITE);
    for s in [-1.0f32, 1.0] {
        m.cuboid(Vec3::new(s * 2.3, 0.35, 0.1), Vec3::new(0.05, 0.62, 0.05), Quat::from_rotation_z(-s * 0.9), DARK);
        m.aabb(Vec3::new(s * 5.45, 0.82, 0.1), Vec3::new(0.12, 0.08, 0.7), RED);
    }
    // 水平/垂直尾翼（固定部分）
    m.aabb(Vec3::new(0.0, 0.42, 4.55), Vec3::new(1.8, 0.04, 0.35), WHITE);
    m.aabb(Vec3::new(0.0, 0.95, 4.6), Vec3::new(0.04, 0.6, 0.4), WHITE);
    m.aabb(Vec3::new(0.0, 1.5, 4.7), Vec3::new(0.05, 0.1, 0.3), RED);
    // 起落架（固定）
    m.aabb(Vec3::new(0.0, -0.75, -1.35), Vec3::new(0.04, 0.3, 0.04), DARK);
    m.aabb(Vec3::new(0.0, -1.0, -1.35), Vec3::new(0.06, 0.16, 0.16), DARK);
    for s in [-1.0f32, 1.0] {
        m.cuboid(Vec3::new(s * 0.95, -0.72, 0.35), Vec3::new(0.04, 0.55, 0.04), Quat::from_rotation_z(s * 0.7), WHITE);
        m.aabb(Vec3::new(s * 1.25, -0.92, 0.35), Vec3::new(0.07, 0.17, 0.17), DARK);
    }
    m.build()
}

fn surface_mesh(half: Vec3, offset: Vec3, col: [f32; 4]) -> Mesh {
    let mut m = MeshBuilder::new();
    m.aabb(offset, half, col);
    m.build()
}


// ============================================================ 通用数据驱动模型

const YELLOW: [f32; 4] = [0.95, 0.78, 0.12, 1.0];
const BLUE: [f32; 4] = [0.12, 0.25, 0.6, 1.0];
const GREY: [f32; 4] = [0.55, 0.58, 0.6, 1.0];
const OLIVE: [f32; 4] = [0.33, 0.4, 0.27, 1.0];

struct Style {
    body: [f32; 4],
    accent: [f32; 4],
    /// 机身半宽/半高。
    half: Vec2,
    high_wing: bool,
    ttail: bool,
    /// 机翼后掠（rad）。
    sweep: f32,
}

fn style(code: &str) -> Style {
    match code {
        "TWIN-6" => Style { body: YELLOW, accent: WHITE, half: Vec2::new(1.0, 1.15), high_wing: true, ttail: true, sweep: 0.0 },
        "JET-39" => Style { body: GREY, accent: OLIVE, half: Vec2::new(0.55, 0.6), high_wing: false, ttail: false, sweep: 0.05 },
        "LINER-73" => Style { body: WHITE, accent: BLUE, half: Vec2::new(1.85, 1.95), high_wing: false, ttail: false, sweep: 0.42 },
        "SEA-415" => Style { body: YELLOW, accent: RED, half: Vec2::new(1.5, 1.4), high_wing: true, ttail: true, sweep: 0.0 },
        _ => Style { body: WHITE, accent: RED, half: Vec2::new(0.55, 0.6), high_wing: true, ttail: false, sweep: 0.0 },
    }
}

fn surf<F: Fn(SurfaceKind) -> bool>(cfg: &AircraftConfig, f: F) -> &SurfaceConfig {
    cfg.surfaces.iter().find(|s| f(s.kind)).expect("surface")
}

/// 模型各部分（网格 + 局部变换）。
pub struct ModelParts {
    pub body: Mesh,
    pub gear: Mesh,
    /// (发动机序号, 位置, 桨盘半径)
    pub props: Vec<(usize, Vec3, f32)>,
    /// (舵面种类, 铰链位置, 半尺寸, 网格偏移, 颜色)
    pub surfaces: Vec<(ControlSurface, Vec3, Vec3, Vec3, [f32; 4])>,
}

pub fn model_parts(cfg: &AircraftConfig) -> ModelParts {
    if cfg.code == "SKY-172" {
        return sky172_parts();
    }
    let st = style(cfg.code);
    let v = |d: DVec3| Vec3::new(d.x as f32, d.y as f32, d.z as f32);
    let wing_in = surf(cfg, |k| k == SurfaceKind::WingRightInner);
    let wing_out = surf(cfg, |k| k == SurfaceKind::WingRightOuter);
    let htail = surf(cfg, |k| k == SurfaceKind::HorizontalStab);
    let vtail = surf(cfg, |k| k == SurfaceKind::VerticalStab);
    let span = cfg.wing_span as f32;
    let chord = cfg.mean_chord as f32;
    let (wy, wz) = (wing_in.pos.y as f32, wing_in.pos.z as f32);
    let nose_z = cfg.contacts.first().map_or(-2.0, |c| c.pos.z as f32);
    let z_front = nose_z - st.half.x * 0.9 - 1.0;
    let z_back = vtail.pos.z as f32 + 1.4;
    let len = z_back - z_front;
    let zc = 0.5 * (z_front + z_back);

    let mut b = MeshBuilder::new();
    // ---- 机身：主段 + 圆润机头 + 渐缩尾锥 + 座舱盖
    let main_len = len * 0.62;
    let main_zc = z_front + 0.12 * len + main_len * 0.5;
    b.aabb(Vec3::new(0.0, 0.0, main_zc), Vec3::new(st.half.x, st.half.y, main_len * 0.5), st.body);
    b.blob(Vec3::new(0.0, -0.02, z_front + 0.12 * len), Vec3::new(st.half.x, st.half.y * 0.95, 0.13 * len), st.body);
    let tail_len = z_back - (main_zc + main_len * 0.5);
    b.aabb(Vec3::new(0.0, st.half.y * 0.25, main_zc + main_len * 0.5 + tail_len * 0.25), Vec3::new(st.half.x * 0.7, st.half.y * 0.7, tail_len * 0.25), st.body);
    b.aabb(Vec3::new(0.0, st.half.y * 0.5, main_zc + main_len * 0.5 + tail_len * 0.75), Vec3::new(st.half.x * 0.4, st.half.y * 0.45, tail_len * 0.25), st.body);
    b.blob(Vec3::new(0.0, st.half.y * 0.85, z_front + 0.22 * len), Vec3::new(st.half.x * 0.7, st.half.y * 0.38, 0.07 * len), GLASS);
    b.aabb(Vec3::new(0.0, 0.0, zc), Vec3::new(st.half.x + 0.01, st.half.y * 0.08, len * 0.45), st.accent); // 腰线

    // ---- 主翼：左右各一段（后掠 + 上反）
    let semi = span * 0.5;
    let t = (0.04 * chord).max(0.07);
    for s in [-1.0f32, 1.0] {
        let dih = (wing_in.normal.x as f32).abs().asin();
        let rot = Quat::from_rotation_y(-s * st.sweep) * Quat::from_rotation_z(-s * dih);
        let cx = s * (semi * 0.5) * st.sweep.cos();
        let cz = wz + (semi * 0.5) * st.sweep.sin() * 1.0;
        b.cuboid(Vec3::new(cx, wy, cz), Vec3::new(semi * 0.5, t, chord * 0.5), rot, st.body);
        // 翼尖
        b.aabb(Vec3::new(s * (semi * st.sweep.cos() + 0.05), wy + dih.sin() * semi, wz + semi * st.sweep.sin()), Vec3::new(0.12, 0.09, chord * 0.42), st.accent);
    }
    if cfg.code == "JET-39" {
        // L-39 标志性的翼尖油箱
        for s in [-1.0f32, 1.0] {
            b.blob(Vec3::new(s * (semi * st.sweep.cos() + 0.35), wy + 0.1, wz + semi * st.sweep.sin() - 0.2), Vec3::new(0.32, 0.32, 1.7), st.accent);
        }
    }
    if !st.high_wing {
        // 下单翼：翼根整流
        b.aabb(Vec3::new(0.0, wy + 0.1, wz), Vec3::new(st.half.x + 0.3, 0.12, chord * 0.55), st.body);
    }

    // ---- 尾翼
    let hspan = (htail.area * htail.aspect_ratio).sqrt() as f32;
    let hch = (htail.area as f32 / hspan).max(0.4);
    let vh = (vtail.area * vtail.aspect_ratio).sqrt() as f32;
    let vch = (vtail.area as f32 / vh).max(0.5);
    let (hy, hz) = (htail.pos.y as f32, htail.pos.z as f32);
    let (vy, vz) = (vtail.pos.y as f32, vtail.pos.z as f32);
    b.aabb(Vec3::new(0.0, hy, hz - hch * 0.15), Vec3::new(hspan * 0.5, 0.05, hch * 0.35), st.body);
    let fin_center = if st.ttail { 0.5 * (hy + st.half.y * 0.4) } else { vy };
    let fin_half = if st.ttail { 0.5 * (hy - st.half.y * 0.4 + 0.1) } else { vh * 0.5 };
    b.aabb(Vec3::new(0.0, fin_center, vz - vch * 0.15), Vec3::new(0.06, fin_half, vch * 0.35), st.body);
    b.aabb(Vec3::new(0.0, fin_center + fin_half, vz - vch * 0.15), Vec3::new(0.07, 0.12, vch * 0.3), st.accent);

    // ---- 发动机
    let mut props = Vec::new();
    for (i, e) in cfg.engines.iter().enumerate() {
        let p = v(e.pos);
        match e.kind {
            EngineKind::Piston { .. } => {
                props.push((i, Vec3::new(p.x, p.y, z_front - 0.15), e.prop_diameter as f32 * 0.5));
            }
            EngineKind::Turboprop { .. } => {
                let r = e.prop_diameter as f32 * 0.5;
                b.aabb(Vec3::new(p.x, p.y - 0.1, p.z + 1.0), Vec3::new(r * 0.3, r * 0.33, 1.9), st.body);
                b.aabb(Vec3::new(p.x, p.y - 0.1, p.z - 0.9), Vec3::new(r * 0.22, r * 0.25, 0.2), DARK);
                props.push((i, Vec3::new(p.x, p.y - 0.1, p.z - 1.15), r));
            }
            EngineKind::Turbofan { .. } => {
                if cfg.engines.len() == 1 {
                    // 单发：两侧进气道 + 尾喷口
                    for s in [-1.0f32, 1.0] {
                        b.aabb(Vec3::new(s * (st.half.x + 0.15), 0.0, z_front + 0.3 * len), Vec3::new(0.2, st.half.y * 0.55, 1.1), DARK);
                    }
                    b.aabb(Vec3::new(0.0, 0.0, z_back + 0.1), Vec3::new(0.3, 0.3, 0.2), DARK);
                } else {
                    let r = (cfg.wing_span as f32 * 0.03).max(0.5);
                    b.aabb(Vec3::new(p.x, p.y - 0.05, p.z + 0.4), Vec3::new(r, r, 2.0), st.body);
                    b.aabb(Vec3::new(p.x, p.y - 0.05, p.z - 1.65), Vec3::new(r * 0.8, r * 0.8, 0.12), DARK);
                    b.aabb(Vec3::new(p.x, p.y - 0.05, p.z + 2.5), Vec3::new(r * 0.5, r * 0.5, 0.2), DARK);
                }
            }
        }
    }

    // ---- 水上机：船体 + 翼尖浮筒
    let floats: Vec<&ContactConfig> = cfg.contacts.iter().filter(|c| c.kind == ContactKind::Float).collect();
    if !floats.is_empty() {
        let hull: Vec<&&ContactConfig> = floats.iter().filter(|c| c.pos.x.abs() < 3.0).collect();
        let zmin = hull.iter().map(|c| c.pos.z as f32).fold(f32::MAX, f32::min);
        let zmax = hull.iter().map(|c| c.pos.z as f32).fold(f32::MIN, f32::max);
        let keel = hull.iter().map(|c| c.pos.y as f32).fold(f32::MAX, f32::min);
        b.blob(Vec3::new(0.0, keel * 0.5, 0.5 * (zmin + zmax)), Vec3::new(st.half.x * 0.95, (-keel + st.half.y) * 0.5, (zmax - zmin) * 0.55), st.body);
        b.aabb(Vec3::new(0.0, keel + 0.12, 0.5 * (zmin + zmax)), Vec3::new(st.half.x * 0.5, 0.1, (zmax - zmin) * 0.5), st.accent);
        for c in floats.iter().filter(|c| c.pos.x.abs() > 3.0) {
            let (fx, fy, fz) = (c.pos.x as f32, c.pos.y as f32, c.pos.z as f32);
            b.blob(Vec3::new(fx, fy - 0.1, fz), Vec3::new(0.3, 0.3, 1.6), st.accent);
            b.aabb(Vec3::new(fx * 0.995, fy + (wy - fy) * 0.5 + 0.2, fz), Vec3::new(0.05, (wy - fy) * 0.5, 0.08), DARK);
        }
    }

    // ---- 起落架（可收放）
    let mut g = MeshBuilder::new();
    for c in cfg.contacts.iter().filter(|c| c.kind == ContactKind::Wheel) {
        let p = v(c.pos);
        let leg = c.rest_len as f32;
        let r = c.radius as f32;
        g.aabb(Vec3::new(p.x, p.y - leg * 0.5, p.z), Vec3::new(0.05 + r * 0.12, leg * 0.5, 0.05 + r * 0.12), DARK);
        g.aabb(Vec3::new(p.x, p.y - leg - r * 0.15, p.z), Vec3::new(r * 0.45, r, r), DARK);
    }

    // ---- 舵面
    let out_x = wing_out.pos.x.abs() as f32;
    let ail_half = Vec3::new(semi * 0.2, 0.035, chord * 0.12);
    let ail_hinge_z = wz + chord * 0.32 + (out_x * st.sweep.tan());
    let ail_y = wy;
    let surfaces = vec![
        (ControlSurface::AileronL, Vec3::new(-out_x, ail_y, ail_hinge_z), ail_half, Vec3::new(0.0, 0.0, ail_half.z), st.accent),
        (ControlSurface::AileronR, Vec3::new(out_x, ail_y, ail_hinge_z), ail_half, Vec3::new(0.0, 0.0, ail_half.z), st.accent),
        (ControlSurface::Elevator, Vec3::new(0.0, hy, hz + hch * 0.2), Vec3::new(hspan * 0.48, 0.04, hch * 0.17), Vec3::new(0.0, 0.0, hch * 0.17), st.body),
        (ControlSurface::Rudder, Vec3::new(0.0, fin_center + fin_half * 0.0 + if st.ttail { 0.0 } else { 0.0 }, vz + vch * 0.2), Vec3::new(0.04, fin_half * 0.98, vch * 0.17), Vec3::new(0.0, 0.0, vch * 0.17), st.body),
    ];
    ModelParts { body: b.build(), gear: g.build(), props, surfaces }
}

fn sky172_parts() -> ModelParts {
    let mut surfaces = vec![];
    let ail = |x: f32, c: ControlSurface| (c, Vec3::new(x, 0.82, 0.8), Vec3::new(1.4, 0.04, 0.2), Vec3::new(0.0, 0.0, 0.2), RED);
    surfaces.push(ail(-3.9, ControlSurface::AileronL));
    surfaces.push(ail(3.9, ControlSurface::AileronR));
    surfaces.push((ControlSurface::Elevator, Vec3::new(0.0, 0.42, 4.88), Vec3::new(1.75, 0.035, 0.22), Vec3::new(0.0, 0.0, 0.22), WHITE));
    surfaces.push((ControlSurface::Rudder, Vec3::new(0.0, 0.95, 5.0), Vec3::new(0.035, 0.55, 0.2), Vec3::new(0.0, 0.0, 0.2), WHITE));
    ModelParts { body: body_mesh(), gear: { let mut m = MeshBuilder::new(); m.aabb(Vec3::ZERO, Vec3::splat(0.001), DARK); m.build() }, props: vec![(0, Vec3::new(0.0, 0.0, -2.9), 0.95)], surfaces }
}

/// 生成并挂接一个机型模型，返回根实体。
pub fn spawn_model(commands: &mut Commands, meshes: &mut Assets<Mesh>, mat: Handle<StandardMaterial>, cfg: &AircraftConfig) -> Entity {
    let parts = model_parts(cfg);
    let root = commands.spawn((Transform::default(), Visibility::Hidden, AircraftRoot)).id();
    let mut kids = vec![];
    kids.push(commands.spawn((Mesh3d(meshes.add(parts.body)), MeshMaterial3d(mat.clone()), Transform::default())).id());
    if cfg.code != "SKY-172" {
        kids.push(commands.spawn((Mesh3d(meshes.add(parts.gear)), MeshMaterial3d(mat.clone()), Transform::default(), GearPart)).id());
    }
    for (i, pos, r) in parts.props {
        let mut m = MeshBuilder::new();
        m.aabb(Vec3::ZERO, Vec3::new(0.07 * r.max(0.5), r, 0.025), DARK);
        m.aabb(Vec3::ZERO, Vec3::new(r, 0.07 * r.max(0.5), 0.025), DARK);
        m.aabb(Vec3::new(0.0, 0.0, -0.05), Vec3::new(0.12 * r.max(0.6), 0.12 * r.max(0.6), 0.12), RED);
        kids.push(commands.spawn((Mesh3d(meshes.add(m.build())), MeshMaterial3d(mat.clone()), Transform::from_translation(pos), Propeller(i))).id());
    }
    for (c, hinge, half, off, col) in parts.surfaces {
        kids.push(commands.spawn((Mesh3d(meshes.add(surface_mesh(half, off, col))), MeshMaterial3d(mat.clone()), Transform::from_translation(hinge), c)).id());
    }
    commands.entity(root).add_children(&kids);
    root
}

fn model_material(mats: &mut Assets<StandardMaterial>) -> Handle<StandardMaterial> {
    mats.add(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 0.8, reflectance: 0.2, ..default() })
}

pub fn spawn_aircraft(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>, game: Res<GameState>, mut cur: ResMut<CurrentModel>) {
    let mat = model_material(&mut mats);
    spawn_model(&mut commands, &mut meshes, mat, &game.sim.config);
    cur.0 = game.sim.config.code.to_string();
}

/// 机型切换时重建模型。
pub fn sync_model(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    game: Res<GameState>,
    mut cur: ResMut<CurrentModel>,
    roots: Query<Entity, With<AircraftRoot>>,
) {
    if cur.0 == game.sim.config.code {
        return;
    }
    for e in &roots {
        commands.entity(e).despawn();
    }
    let mat = model_material(&mut mats);
    spawn_model(&mut commands, &mut meshes, mat, &game.sim.config);
    cur.0 = game.sim.config.code.to_string();
}

#[allow(clippy::type_complexity)]
pub fn update_aircraft(
    game: Res<GameState>,
    origin: Res<FloatingOrigin>,
    mut root: Query<(&mut Transform, &mut Visibility), (With<AircraftRoot>, Without<Propeller>, Without<ControlSurface>, Without<GearPart>)>,
    mut prop: Query<(&Propeller, &mut Transform), (Without<ControlSurface>, Without<AircraftRoot>, Without<GearPart>)>,
    mut surf: Query<(&ControlSurface, &mut Transform), (Without<Propeller>, Without<AircraftRoot>, Without<GearPart>)>,
    mut gear: Query<&mut Visibility, (With<GearPart>, Without<AircraftRoot>)>,
    time: Res<Time>,
) {
    let s = &game.sim.state;
    if let Ok((mut t, mut v)) = root.single_mut() {
        let p = s.rigid.pos;
        t.translation = Vec3::new((p.x - origin.0.x) as f32, (p.y - origin.0.y) as f32, (p.z - origin.0.z) as f32);
        let q = s.rigid.q;
        t.rotation = Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32);
        *v = if matches!(game.view, crate::cockpit_ui::ViewMode::Chase | crate::cockpit_ui::ViewMode::Flyby) && game.screen == crate::Screen::Flight {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    for (p, mut t) in &mut prop {
        let rpm = s.engines.get(p.0).map(|e| e.rpm).unwrap_or(0.0) as f32;
        // 转速过高时以 0.5 弧度/帧限制频闪
        let w = (rpm / 60.0 * std::f32::consts::TAU).min(80.0);
        t.rotate_local_z(w * time.delta_secs());
    }
    let sf = s.surfaces;
    for (c, mut t) in &mut surf {
        t.rotation = match c {
            ControlSurface::AileronL => Quat::from_rotation_x(-(sf.aileron as f32) * 0.35),
            ControlSurface::AileronR => Quat::from_rotation_x((sf.aileron as f32) * 0.35),
            ControlSurface::Elevator => Quat::from_rotation_x((sf.elevator as f32) * 0.4),
            ControlSurface::Rudder => Quat::from_rotation_y((sf.rudder as f32) * 0.4),
        };
    }
    for mut v in &mut gear {
        *v = if s.gear_pos > 0.5 { Visibility::Inherited } else { Visibility::Hidden };
    }
}

pub struct AircraftMeshPlugin;
impl Plugin for AircraftMeshPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CurrentModel>().add_systems(Startup, spawn_aircraft).add_systems(Update, (sync_model, update_aircraft));
    }
}
