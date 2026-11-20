//! SKY-172 程序化低多边形 3D 模型（外部视角可见），含舵面/螺旋桨动画与起落架。

use crate::mesh_util::MeshBuilder;
use crate::origin::FloatingOrigin;
use crate::GameState;
use bevy::prelude::*;

#[derive(Component)]
pub struct AircraftRoot;
#[derive(Component)]
pub struct Propeller;
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

fn prop_mesh() -> Mesh {
    let mut m = MeshBuilder::new();
    m.aabb(Vec3::ZERO, Vec3::new(0.07, 0.95, 0.025), DARK);
    m.aabb(Vec3::ZERO, Vec3::new(0.95, 0.07, 0.025), DARK);
    m.aabb(Vec3::new(0.0, 0.0, -0.05), Vec3::new(0.12, 0.12, 0.12), RED);
    m.build()
}

fn surface_mesh(half: Vec3, offset: Vec3, col: [f32; 4]) -> Mesh {
    let mut m = MeshBuilder::new();
    m.aabb(offset, half, col);
    m.build()
}

pub fn spawn_aircraft(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut mats: ResMut<Assets<StandardMaterial>>) {
    let mat = mats.add(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 0.8, reflectance: 0.2, ..default() });
    let root = commands.spawn((Transform::default(), Visibility::Hidden, AircraftRoot)).id();
    let body = commands.spawn((Mesh3d(meshes.add(body_mesh())), MeshMaterial3d(mat.clone()), Transform::default())).id();
    let prop = commands.spawn((Mesh3d(meshes.add(prop_mesh())), MeshMaterial3d(mat.clone()), Transform::from_xyz(0.0, 0.0, -2.9), Propeller)).id();
    // 舵面：铰链在前缘，网格向后偏移
    let ail_l = commands
        .spawn((Mesh3d(meshes.add(surface_mesh(Vec3::new(1.4, 0.04, 0.2), Vec3::new(0.0, 0.0, 0.2), RED))), MeshMaterial3d(mat.clone()), Transform::from_xyz(-3.9, 0.82, 0.8), ControlSurface::AileronL))
        .id();
    let ail_r = commands
        .spawn((Mesh3d(meshes.add(surface_mesh(Vec3::new(1.4, 0.04, 0.2), Vec3::new(0.0, 0.0, 0.2), RED))), MeshMaterial3d(mat.clone()), Transform::from_xyz(3.9, 0.82, 0.8), ControlSurface::AileronR))
        .id();
    let elev = commands
        .spawn((Mesh3d(meshes.add(surface_mesh(Vec3::new(1.75, 0.035, 0.22), Vec3::new(0.0, 0.0, 0.22), WHITE))), MeshMaterial3d(mat.clone()), Transform::from_xyz(0.0, 0.42, 4.88), ControlSurface::Elevator))
        .id();
    let rud = commands
        .spawn((Mesh3d(meshes.add(surface_mesh(Vec3::new(0.035, 0.55, 0.2), Vec3::new(0.0, 0.0, 0.2), WHITE))), MeshMaterial3d(mat.clone()), Transform::from_xyz(0.0, 0.95, 5.0), ControlSurface::Rudder))
        .id();
    commands.entity(root).add_children(&[body, prop, ail_l, ail_r, elev, rud]);
}

pub fn update_aircraft(
    game: Res<GameState>,
    origin: Res<FloatingOrigin>,
    mut root: Query<(&mut Transform, &mut Visibility), (With<AircraftRoot>, Without<Propeller>, Without<ControlSurface>)>,
    mut prop: Query<&mut Transform, (With<Propeller>, Without<ControlSurface>, Without<AircraftRoot>)>,
    mut surf: Query<(&ControlSurface, &mut Transform), (Without<Propeller>, Without<AircraftRoot>)>,
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
    let rpm = s.engines.first().map(|e| e.rpm).unwrap_or(0.0) as f32;
    for mut t in &mut prop {
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
}

pub struct AircraftMeshPlugin;
impl Plugin for AircraftMeshPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_aircraft).add_systems(Update, update_aircraft);
    }
}
