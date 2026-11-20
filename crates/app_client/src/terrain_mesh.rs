//! 多级 LOD 地形 Chunk 异步生成与卸载（AsyncComputeTaskPool）。
//! 地形采样使用 f64 世界坐标；Chunk 顶点相对 Chunk 中心（f32），实体位置由 Floating Origin 同步。

use crate::mesh_util::MeshBuilder;
use crate::origin::WorldAnchor;
use crate::GameState;
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy::tasks::futures_lite::future;
use bevy::tasks::{AsyncComputeTaskPool, Task};
use std::collections::HashMap;
use std::sync::Arc;
use world_gen::climate;
use world_gen::coords::world_to_latlon;
use world_gen::World;

/// 每级 LOD 的 Chunk 边长（m）与每边网格数。
pub const LOD_SIZE: [f64; 4] = [1_200.0, 4_800.0, 19_200.0, 76_800.0];
pub const CELLS: i32 = 24;
const RADIUS: i64 = 3;
/// 低 LOD 下沉（m），避免与高 LOD 重叠处 Z 冲突。
const LOD_BIAS: [f32; 4] = [0.0, -4.0, -18.0, -70.0];

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct ChunkKey {
    pub lod: u8,
    pub cx: i64,
    pub cz: i64,
}

impl ChunkKey {
    pub fn center(&self) -> (f64, f64) {
        let s = LOD_SIZE[self.lod as usize];
        ((self.cx as f64 + 0.5) * s, (self.cz as f64 + 0.5) * s)
    }
}

#[derive(Component)]
pub struct TerrainChunk(pub ChunkKey);

#[derive(Resource, Default)]
pub struct ChunkManager {
    pub loaded: HashMap<ChunkKey, Entity>,
    pub pending: HashMap<ChunkKey, Task<MeshBuilder>>,
    pub day_built: f64,
}

#[derive(Resource)]
pub struct TerrainMaterial(pub Handle<StandardMaterial>);

/// 生成一个 Chunk 的网格数据（纯函数，可在线程池运行）。
pub fn build_chunk(world: &World, day: f64, key: ChunkKey) -> MeshBuilder {
    let size = LOD_SIZE[key.lod as usize];
    let step = size / CELLS as f64;
    let (xc, zc) = key.center();
    let (x0, z0) = (xc - size / 2.0, zc - size / 2.0);
    let n = (CELLS + 3) as usize; // 含 1 圈边界用于坡度
    let mut hs = vec![0.0f64; n * n];
    for j in 0..n {
        for i in 0..n {
            hs[j * n + i] = world.height(x0 + (i as f64 - 1.0) * step, z0 + (j as f64 - 1.0) * step);
        }
    }
    let at = |i: i32, j: i32| hs[((j + 1) as usize) * n + (i + 1) as usize];
    let verts = (CELLS + 1) as usize;
    let mut pos = vec![Vec3::ZERO; verts * verts];
    let mut col = vec![[0.0f32; 4]; verts * verts];
    let bias = LOD_BIAS[key.lod as usize];
    for j in 0..=CELLS {
        for i in 0..=CELLS {
            let h = at(i, j);
            let dx = (at(i + 1, j) - at(i - 1, j)) / (2.0 * step);
            let dz = (at(i, j + 1) - at(i, j - 1)) / (2.0 * step);
            let slope = dx.hypot(dz).atan();
            let (wx, wz) = (x0 + i as f64 * step, z0 + j as f64 * step);
            let (lat, _) = world_to_latlon(wx, wz);
            let ocean = h <= 0.0;
            let t_sl = climate::sea_level_temp(lat, day, ocean);
            let hum = climate::humidity(lat, world.terrain.humidity_noise(wx, wz));
            let b = climate::classify(h, slope, t_sl, hum, climate::snow_line(lat, day));
            let mut c = climate::biome_color(b);
            // 基于高度的细微明暗，增强低模层次感
            let shade = 1.0 + ((h * 0.004).sin() as f32) * 0.05;
            for v in &mut c {
                *v = (*v * shade).clamp(0.0, 1.0);
            }
            let y = if ocean { 0.0 } else { h } as f32 + bias;
            pos[(j as usize) * verts + i as usize] = Vec3::new((i as f64 * step - size / 2.0) as f32, y, (j as f64 * step - size / 2.0) as f32);
            col[(j as usize) * verts + i as usize] = [c[0], c[1], c[2], 1.0];
        }
    }
    let mut mb = MeshBuilder::new();
    let skirt = 40.0 * (key.lod as f32 + 1.0);
    for j in 0..CELLS as usize {
        for i in 0..CELLS as usize {
            let idx = |a: usize, b: usize| b * verts + a;
            let (p00, p10, p01, p11) = (idx(i, j), idx(i + 1, j), idx(i, j + 1), idx(i + 1, j + 1));
            // 朝上的绕序：(x,z) 平面内 p00 → p01 → p10
            let avg = |a: usize, b: usize, c: usize| {
                let (ca, cb, cc) = (col[a], col[b], col[c]);
                [(ca[0] + cb[0] + cc[0]) / 3.0, (ca[1] + cb[1] + cc[1]) / 3.0, (ca[2] + cb[2] + cc[2]) / 3.0, 1.0]
            };
            mb.tri(pos[p00], pos[p01], pos[p10], avg(p00, p01, p10));
            mb.tri(pos[p10], pos[p01], pos[p11], avg(p10, p01, p11));
        }
    }
    // 裙边：向下延伸，遮住不同 LOD 之间的裂缝
    let mut edge = |a: usize, b: usize, flip: bool| {
        let (pa, pb) = (pos[a], pos[b]);
        let (da, db) = (pa - Vec3::Y * skirt, pb - Vec3::Y * skirt);
        let c = col[a];
        if flip {
            mb.quad(pa, pb, db, da, c);
        } else {
            mb.quad(pb, pa, da, db, c);
        }
    };
    let idx = |a: usize, b: usize| b * verts + a;
    for i in 0..CELLS as usize {
        edge(idx(i, 0), idx(i + 1, 0), false);
        edge(idx(i, CELLS as usize), idx(i + 1, CELLS as usize), true);
        edge(idx(0, i), idx(0, i + 1), true);
        edge(idx(CELLS as usize, i), idx(CELLS as usize, i + 1), false);
    }
    mb
}

/// 每帧：决定需要的 Chunk、发起异步任务、收取结果、卸载远处 Chunk。
pub fn stream_chunks(
    mut commands: Commands,
    game: Res<GameState>,
    mut mgr: ResMut<ChunkManager>,
    mat: Option<Res<TerrainMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Some(mat) = mat else { return };
    let r = &game.sim.state.rigid;
    let (ax, az) = (r.pos.x, r.pos.z);

    // 日期变化（积雪线移动）→ 全部重建
    if (game.day - mgr.day_built).abs() >= 3.0 {
        let ents: Vec<Entity> = mgr.loaded.drain().map(|(_, e)| e).collect();
        for e in ents {
            commands.entity(e).despawn();
        }
        mgr.pending.clear();
        mgr.day_built = game.day;
    }

    // 需要的集合（高 LOD 覆盖区域内的低 LOD Chunk 被跳过）
    let mut wanted: Vec<ChunkKey> = Vec::new();
    for lod in 0..4usize {
        let s = LOD_SIZE[lod];
        let (acx, acz) = ((ax / s).floor() as i64, (az / s).floor() as i64);
        for dz in -RADIUS..=RADIUS {
            for dx in -RADIUS..=RADIUS {
                let key = ChunkKey { lod: lod as u8, cx: acx + dx, cz: acz + dz };
                if lod > 0 {
                    let ps = LOD_SIZE[lod - 1];
                    let (pcx, pcz) = ((ax / ps).floor() as i64, (az / ps).floor() as i64);
                    let (min_x, max_x) = ((pcx - RADIUS) as f64 * ps, (pcx + RADIUS + 1) as f64 * ps);
                    let (min_z, max_z) = ((pcz - RADIUS) as f64 * ps, (pcz + RADIUS + 1) as f64 * ps);
                    let (kx0, kz0) = (key.cx as f64 * s, key.cz as f64 * s);
                    if kx0 >= min_x && kx0 + s <= max_x && kz0 >= min_z && kz0 + s <= max_z {
                        continue;
                    }
                }
                wanted.push(key);
            }
        }
    }
    // 近处优先
    wanted.sort_by(|a, b| {
        let da = (a.center().0 - ax).hypot(a.center().1 - az);
        let db = (b.center().0 - ax).hypot(b.center().1 - az);
        da.partial_cmp(&db).unwrap()
    });
    let wanted_set: std::collections::HashSet<ChunkKey> = wanted.iter().copied().collect();

    // 卸载
    let stale: Vec<ChunkKey> = mgr.loaded.keys().filter(|k| !wanted_set.contains(k)).copied().collect();
    for k in stale {
        if let Some(e) = mgr.loaded.remove(&k) {
            commands.entity(e).despawn();
        }
    }
    mgr.pending.retain(|k, _| wanted_set.contains(k));

    // 收取完成的任务
    let done: Vec<ChunkKey> = mgr.pending.keys().copied().collect();
    for k in done {
        let finished = {
            let task = mgr.pending.get_mut(&k).unwrap();
            future::block_on(future::poll_once(task))
        };
        if let Some(mb) = finished {
            mgr.pending.remove(&k);
            let (xc, zc) = k.center();
            let e = commands
                .spawn((
                    Mesh3d(meshes.add(mb.build())),
                    MeshMaterial3d(mat.0.clone()),
                    Transform::default(),
                    WorldAnchor(DVec3::new(xc, 0.0, zc)),
                    TerrainChunk(k),
                ))
                .id();
            mgr.loaded.insert(k, e);
        }
    }

    // 发起新任务（每帧最多 8 个，避免线程池拥塞）
    let pool = AsyncComputeTaskPool::get();
    let mut started = 0;
    for k in wanted {
        if started >= 8 || mgr.pending.len() >= 24 {
            break;
        }
        if mgr.loaded.contains_key(&k) || mgr.pending.contains_key(&k) {
            continue;
        }
        let world: Arc<World> = game.world.clone();
        let day = game.day;
        mgr.pending.insert(k, pool.spawn(async move { build_chunk(&world, day, k) }));
        started += 1;
    }
}

pub fn setup_terrain_material(mut commands: Commands, mut mats: ResMut<Assets<StandardMaterial>>) {
    let h = mats.add(StandardMaterial { base_color: Color::WHITE, perceptual_roughness: 1.0, reflectance: 0.05, metallic: 0.0, ..default() });
    commands.insert_resource(TerrainMaterial(h));
}

pub struct TerrainPlugin;
impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ChunkManager>().add_systems(Startup, setup_terrain_material).add_systems(Update, stream_chunks);
    }
}
