//! world_gen — 纯 Rust 1:32 过程化星球：坐标、气候季节、地形、机场与导航台。
//!
//! **严禁依赖 bevy**；全部逻辑可无头 `cargo test`。

pub mod airport;
pub mod climate;
pub mod coords;
pub mod noise;
pub mod terrain;

use airport::{Airport, GRID_NX, GRID_NZ};
use climate::{Biome, ClimateSample};
use coords::*;
use terrain::TerrainGen;

/// 整个星球：地形发生器 + 全球机场表 + 机场空间网格。
pub struct World {
    pub seed: u64,
    pub terrain: TerrainGen,
    airports: Vec<Airport>,
    grid: Vec<Option<usize>>,
}

impl World {
    pub fn new(seed: u64) -> Self {
        let terrain = TerrainGen::new(seed);
        let airports = airport::generate(&terrain);
        let mut grid = vec![None; (GRID_NX * GRID_NZ) as usize];
        for (i, a) in airports.iter().enumerate() {
            let (ci, cj) = airport::cell_of(a.center.0, a.center.1);
            grid[(cj * GRID_NX + ci) as usize] = Some(i);
        }
        Self { seed, terrain, airports, grid }
    }

    pub fn airports(&self) -> &[Airport] {
        &self.airports
    }

    pub fn airport_by_icao(&self, icao: &str) -> Option<&Airport> {
        self.airports.iter().find(|a| a.icao.eq_ignore_ascii_case(icao))
    }

    /// 离 (x, z) 最近的机场及距离（m）。
    pub fn nearest_airport(&self, x: f64, z: f64) -> Option<(&Airport, f64)> {
        self.airports
            .iter()
            .map(|a| (a, wrapped_distance(a.center, (x, z))))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }

    /// 若 (x, z) 处于某机场的跑道影响范围内（压平胶囊 + 混合带内），返回该机场。
    /// 供地面表面类型（沥青/土路）与客户端快速判定使用。
    pub fn airport_near(&self, x: f64, z: f64) -> Option<&Airport> {
        let (ci, cj) = airport::cell_of(x, z);
        for dj in -1..=1 {
            let j = cj + dj;
            if !(0..GRID_NZ).contains(&j) {
                continue;
            }
            for di in -1..=1 {
                let i = (ci + di).rem_euclid(GRID_NX);
                if let Some(idx) = self.grid[(j * GRID_NX + i) as usize] {
                    let ap = &self.airports[idx];
                    if ap.capsule_sdf(x, z) < ap.blend_width {
                        return Some(ap);
                    }
                }
            }
        }
        None
    }

    /// 压平：对 (x,z) 周围 3×3 网格内的机场应用胶囊 SDF 混合。
    pub(crate) fn flatten(&self, x: f64, z: f64, base: f64) -> f64 {
        let (ci, cj) = airport::cell_of(x, z);
        let mut h = base;
        let mut t_min = 1.0f64;
        for dj in -1..=1 {
            let j = cj + dj;
            if !(0..GRID_NZ).contains(&j) {
                continue;
            }
            for di in -1..=1 {
                let i = (ci + di).rem_euclid(GRID_NX);
                if let Some(idx) = self.grid[(j * GRID_NX + i) as usize] {
                    let ap = &self.airports[idx];
                    let sdf = ap.capsule_sdf(x, z);
                    if sdf >= ap.blend_width {
                        continue;
                    }
                    let t = if sdf <= 0.0 { 0.0 } else { smoothstep(sdf / ap.blend_width) };
                    if t < t_min {
                        t_min = t;
                        h = ap.elevation + (base - ap.elevation) * t;
                    }
                }
            }
        }
        h
    }

    /// 最终地形高度（含机场压平）。
    pub fn height(&self, x: f64, z: f64) -> f64 {
        terrain::sample_height(self, x, z)
    }

    pub fn is_ocean(&self, x: f64, z: f64) -> bool {
        self.terrain.base_height(x, z) <= 0.0
    }

    pub fn slope(&self, x: f64, z: f64) -> f64 {
        terrain::slope(self, x, z)
    }

    /// 某点气候采样。
    pub fn climate(&self, x: f64, z: f64, day_of_year: f64) -> ClimateSample {
        let (lat, _) = world_to_latlon(x, z);
        let h = self.height(x, z);
        let ocean = h <= 0.0;
        let temp_sl = climate::sea_level_temp(lat, day_of_year, ocean);
        ClimateSample {
            temp_sl,
            temp: temp_sl - 0.0065 * h.max(0.0),
            humidity: climate::humidity(lat, self.terrain.humidity_noise(x, z)),
            snow_line: climate::snow_line(lat, day_of_year),
        }
    }

    /// 生物群系（含动态积雪：随季节与海拔移动雪线）。
    pub fn biome(&self, x: f64, z: f64, day_of_year: f64) -> Biome {
        let h = self.height(x, z);
        let c = self.climate(x, z, day_of_year);
        climate::classify(h, self.slope(x, z), c.temp_sl, c.humidity, c.snow_line)
    }

    /// 顶点色（RGB 0..1），带基于高度的明暗抖动。
    pub fn color_at(&self, x: f64, z: f64, day_of_year: f64) -> [f32; 3] {
        let b = self.biome(x, z, day_of_year);
        let mut c = climate::biome_color(b);
        // 使用球面点采样噪声：经度方向天然无缝（避免 ±180° 接缝处颜色跳变）。
        let k = WORLD_WIDTH / std::f64::consts::TAU / 220.0;
        let sp = world_to_sphere(x, z) * k;
        let n = noise::noise3(fold(self.seed), sp.x, sp.y, sp.z) as f32 * 0.07;
        for v in &mut c {
            *v = (*v + n).clamp(0.0, 1.0);
        }
        c
    }
}

fn fold(seed: u64) -> u32 {
    noise::fold_seed(seed, 77)
}

#[inline]
fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
