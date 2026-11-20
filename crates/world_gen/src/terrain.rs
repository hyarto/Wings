//! 确定性地形：大陆/海洋掩码 + 构造山脉(Ridged Multifractal) + 丘陵细节，机场压平混合。
//!
//! 全部噪声在单位球面 3D 坐标上采样 → X 方向无缝环绕、极点无畸变。

use crate::coords::*;
use crate::noise::*;
use crate::World;
use glam::DVec3;

/// 陆地最大海拔（m），垂直 1:1。
pub const MAX_LAND_HEIGHT: f64 = 5500.0;
/// 目标海洋占比。
pub const OCEAN_FRACTION: f64 = 0.60;

#[derive(Clone, Debug)]
pub struct TerrainGen {
    pub seed: u64,
    /// 大陆场阈值（经校准使球面海洋占比 = 60%）。
    pub sea_threshold: f64,
    s_cont: u32,
    s_warp: u32,
    s_mtn: u32,
    s_mmask: u32,
    s_hill: u32,
    s_humid: u32,
}

impl TerrainGen {
    pub fn new(seed: u64) -> Self {
        let mut t = Self {
            seed,
            sea_threshold: 0.0,
            s_cont: fold_seed(seed, 1),
            s_warp: fold_seed(seed, 2),
            s_mtn: fold_seed(seed, 3),
            s_mmask: fold_seed(seed, 4),
            s_hill: fold_seed(seed, 5),
            s_humid: fold_seed(seed, 6),
        };
        t.calibrate();
        t
    }

    /// 以球面面积加权的分位数校准海平面阈值。
    fn calibrate(&mut self) {
        let (nx, nz) = (192usize, 96usize);
        let mut v: Vec<(f64, f64)> = Vec::with_capacity(nx * nz);
        for j in 0..nz {
            let lat = 90.0 - (j as f64 + 0.5) / nz as f64 * 180.0;
            let w = lat.to_radians().cos();
            for i in 0..nx {
                let lon = -180.0 + (i as f64 + 0.5) / nx as f64 * 360.0;
                v.push((self.continent_value(sphere_point(lat, lon)), w));
            }
        }
        v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let total: f64 = v.iter().map(|x| x.1).sum();
        let mut acc = 0.0;
        for (c, w) in &v {
            acc += w;
            if acc >= OCEAN_FRACTION * total {
                self.sea_threshold = *c;
                return;
            }
        }
    }

    /// 大陆场（域扭曲 fBm），约 [-1, 1]。
    pub fn continent_value(&self, p: DVec3) -> f64 {
        let w = DVec3::new(
            fbm3(self.s_warp, p.x * 2.0, p.y * 2.0, p.z * 2.0, 3),
            fbm3(self.s_warp.wrapping_add(1), p.x * 2.0 + 5.2, p.y * 2.0, p.z * 2.0, 3),
            fbm3(self.s_warp.wrapping_add(2), p.x * 2.0, p.y * 2.0 + 9.1, p.z * 2.0, 3),
        ) * 0.28;
        let q = (p + w) * 1.35;
        fbm3(self.s_cont, q.x, q.y, q.z, 6)
    }

    /// 未经机场压平的原始地形高度（m，海底为负）。
    pub fn base_height_p(&self, p: DVec3) -> f64 {
        let c = self.continent_value(p);
        let land_t = c - self.sea_threshold;
        if land_t <= 0.0 {
            let d = -land_t;
            return -(180.0 * smoothstep(0.0, 0.03, d) + 3800.0 * smoothstep(0.03, 0.5, d));
        }
        let inland = smoothstep(0.0, 0.5, land_t);
        let coast = smoothstep(0.0, 0.05, land_t);
        let plain = 260.0 * inland;
        let hills = (fbm3(self.s_hill, p.x * 28.0, p.y * 28.0, p.z * 28.0, 5) * 0.5 + 0.5) * (20.0 + 150.0 * inland) * coast;
        let detail = fbm3(self.s_hill.wrapping_add(7), p.x * 150.0, p.y * 150.0, p.z * 150.0, 3) * 9.0 * coast;
        let mmask_n = fbm3(self.s_mmask, p.x * 2.4, p.y * 2.4, p.z * 2.4, 3) * 0.5 + 0.5;
        let mmask = smoothstep(0.38, 0.62, mmask_n) * smoothstep(0.04, 0.30, land_t);
        let ridge = ridged3(self.s_mtn, p.x * 5.5, p.y * 5.5, p.z * 5.5, 6);
        let mtn = 5100.0 * mmask * ridge.powf(1.35);
        (plain + hills + detail + mtn).clamp(0.3, MAX_LAND_HEIGHT)
    }

    pub fn base_height(&self, x: f64, z: f64) -> f64 {
        self.base_height_p(world_to_sphere(x, z))
    }

    /// 局地湿度扰动噪声 [-1,1]。
    pub fn humidity_noise(&self, x: f64, z: f64) -> f64 {
        let p = world_to_sphere(x, z);
        fbm3(self.s_humid, p.x * 4.0, p.y * 4.0, p.z * 4.0, 3)
    }
}

/// 采样最终地形高度（m）：含机场胶囊 SDF 压平与平滑过渡，保证零悬空、零穿模。
pub fn sample_height(world: &World, x: f64, z: f64) -> f64 {
    let base = world.terrain.base_height(x, z);
    world.flatten(x, z, base)
}

/// 地形坡度（rad），30 m 差分。
pub fn slope(world: &World, x: f64, z: f64) -> f64 {
    let e = 30.0;
    let dx = (sample_height(world, x + e, z) - sample_height(world, x - e, z)) / (2.0 * e);
    let dz = (sample_height(world, x, z + e) - sample_height(world, x, z - e)) / (2.0 * e);
    dx.hypot(dz).atan()
}

/// 法线（世界系，单位向量）。
pub fn normal(world: &World, x: f64, z: f64) -> DVec3 {
    let e = 8.0;
    let dx = (sample_height(world, x + e, z) - sample_height(world, x - e, z)) / (2.0 * e);
    let dz = (sample_height(world, x, z + e) - sample_height(world, x, z - e)) / (2.0 * e);
    DVec3::new(-dx, 1.0, -dz).normalize()
}
