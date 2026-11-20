//! 程序化机场：选址、跑道朝向(对齐盛行风)、ICAO 代码、跑道编号、PAPI、风向袋、VOR/ILS。

use crate::climate::prevailing_wind;
use crate::coords::*;
use crate::noise::hash01;
use crate::terrain::TerrainGen;
use std::collections::HashSet;

/// 机场网格单元数（≈40 km × 40 km：世界宽/31，世界高/16）。
pub const GRID_NX: i32 = 31;
pub const GRID_NZ: i32 = 16;
pub const CELL_W: f64 = WORLD_WIDTH / GRID_NX as f64;
pub const CELL_H: f64 = WORLD_HEIGHT / GRID_NZ as f64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirportKind {
    /// 野外土路跑道（600 m）。
    Dirt,
    /// 支线沥青跑道（1,500 m）。
    Regional,
    /// 国际枢纽（3,000 m，带 ILS）。
    International,
}

impl AirportKind {
    pub fn runway_length(self) -> f64 {
        match self {
            Self::Dirt => 600.0,
            Self::Regional => 1_500.0,
            Self::International => 3_000.0,
        }
    }
    pub fn runway_width(self) -> f64 {
        match self {
            Self::Dirt => 18.0,
            Self::Regional => 30.0,
            Self::International => 45.0,
        }
    }
    /// 压平胶囊半径（跑道中线到平坦区边缘）。
    pub fn flat_radius(self) -> f64 {
        match self {
            Self::Dirt => 40.0,
            Self::Regional => 70.0,
            Self::International => 110.0,
        }
    }
    pub fn blend_width(self) -> f64 {
        match self {
            Self::Dirt => 300.0,
            Self::Regional => 600.0,
            Self::International => 900.0,
        }
    }
    fn max_relief(self) -> f64 {
        match self {
            Self::Dirt => 45.0,
            Self::Regional => 28.0,
            Self::International => 18.0,
        }
    }
    pub fn asphalt(self) -> bool {
        self != Self::Dirt
    }
}

#[derive(Clone, Debug)]
pub struct Vor {
    pub ident: String,
    pub freq_mhz: f64,
    pub pos: (f64, f64),
}

impl Vor {
    /// 飞机相对电台的磁（此处为真）径向与距离：(radial°, dist m)。
    pub fn radial_and_distance(&self, x: f64, z: f64) -> (f64, f64) {
        let dx = wrapped_dx(self.pos.0, x);
        let dz = z - self.pos.1;
        (dir_heading(dx, dz), dx.hypot(dz))
    }
}

#[derive(Clone, Debug)]
pub struct Ils {
    pub ident: String,
    pub freq_mhz: f64,
    /// 服务的跑道端（0 或 1）。
    pub end: usize,
    /// 进近航道（着陆航向°）。
    pub course_deg: f64,
    pub glide_deg: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IlsDeviation {
    /// 航向道偏差（度）：+ = 飞机在航道右侧（指针应指向左，提示向左修正）。
    pub loc_deg: f64,
    /// 下滑道偏差（度）：+ = 飞机在下滑道上方。
    pub gs_deg: f64,
    pub dist_to_threshold: f64,
    pub loc_valid: bool,
    pub gs_valid: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Papi {
    pub positions: [(f64, f64); 4],
    pub glide_deg: f64,
}

#[derive(Clone, Debug)]
pub struct Airport {
    pub icao: String,
    pub name: String,
    pub kind: AirportKind,
    pub center: (f64, f64),
    pub elevation: f64,
    pub lat: f64,
    pub lon: f64,
    /// 跑道端 0 的着陆航向（真航向°，10..=180）。端 1 为 +180°。
    pub heading0: f64,
    pub length: f64,
    pub width: f64,
    pub flat_radius: f64,
    pub blend_width: f64,
    pub asphalt: bool,
    pub ils: Vec<Ils>,
    pub vor: Option<Vor>,
    /// 跑道两端入口位置（世界 xz）。着陆在端 e 时，从 `ends[e]` 向 `ends[1-e]` 方向滑跑。
    pub ends: [(f64, f64); 2],
}

impl Airport {
    /// 端 0 着陆方向的水平单位向量 (dx, dz)。
    pub fn dir0(&self) -> (f64, f64) {
        heading_dir(self.heading0)
    }

    pub fn heading_of_end(&self, end: usize) -> f64 {
        (self.heading0 + 180.0 * end as f64).rem_euclid(360.0)
    }

    /// 着陆方向的水平单位向量（端 e）。
    pub fn landing_dir(&self, end: usize) -> (f64, f64) {
        heading_dir(self.heading_of_end(end))
    }

    /// 端号，如 "09"、"27"。
    pub fn designator(&self, end: usize) -> String {
        let n = (self.heading_of_end(end) / 10.0).round() as i32;
        let n = if n == 0 { 36 } else { n };
        format!("{:02}", n)
    }

    /// "09/27"。
    pub fn runway_name(&self) -> String {
        format!("{}/{}", self.designator(0), self.designator(1))
    }

    /// 到跑道线段的胶囊 SDF（m）：≤ 0 为压平区。
    pub fn capsule_sdf(&self, x: f64, z: f64) -> f64 {
        let a = self.ends[0];
        let dx = wrapped_dx(a.0, self.ends[1].0);
        let dz = self.ends[1].1 - a.1;
        let px = wrapped_dx(a.0, x);
        let pz = z - a.1;
        let l2 = dx * dx + dz * dz;
        let t = ((px * dx + pz * dz) / l2).clamp(0.0, 1.0);
        (px - dx * t).hypot(pz - dz * t) - self.flat_radius
    }

    /// 相对跑道的局部坐标：(沿跑道距离[自端0], 横向偏移[相对端 0 着陆方向右侧为正])。
    pub fn runway_local(&self, x: f64, z: f64) -> (f64, f64) {
        let (dx, dz) = self.dir0();
        let px = wrapped_dx(self.ends[0].0, x);
        let pz = z - self.ends[0].1;
        let along = px * dx + pz * dz;
        let lateral = px * (-dz) + pz * dx;
        (along, lateral)
    }

    /// 世界点（沿跑道距离, 横向偏移）→ 世界 xz。
    pub fn runway_point(&self, along: f64, lateral: f64) -> (f64, f64) {
        let (dx, dz) = self.dir0();
        (
            wrap_x(self.ends[0].0 + dx * along - dz * lateral),
            self.ends[0].1 + dz * along + dx * lateral,
        )
    }

    /// PAPI（四盏灯）：位于着陆端入口后 300 m、跑道左侧 `width/2+15 m`，沿横向排布，下滑角 3°。
    pub fn papi(&self, end: usize) -> Papi {
        let s = if end == 0 { 1.0 } else { -1.0 };
        let along0 = if end == 0 { 300.0 } else { self.length - 300.0 };
        let base_lat = -s * (self.width / 2.0 + 15.0);
        let mut positions = [(0.0, 0.0); 4];
        for (i, p) in positions.iter_mut().enumerate() {
            *p = self.runway_point(along0, base_lat - s * 9.0 * i as f64);
        }
        Papi { positions, glide_deg: 3.0 }
    }

    pub fn windsock_pos(&self) -> (f64, f64) {
        self.runway_point(250.0, self.width / 2.0 + 40.0)
    }

    pub fn tower_pos(&self) -> (f64, f64) {
        self.runway_point(self.length * 0.5, self.width / 2.0 + 90.0)
    }

    /// 跑道边灯位置（两侧，每 60 m 一盏）。
    pub fn edge_lights(&self) -> Vec<(f64, f64)> {
        let n = (self.length / 60.0).floor() as i32;
        let off = self.width / 2.0 + 2.0;
        let mut v = Vec::with_capacity(2 * (n as usize + 1));
        for i in 0..=n {
            let a = i as f64 * self.length / n as f64;
            v.push(self.runway_point(a, off));
            v.push(self.runway_point(a, -off));
        }
        v
    }

    /// ILS 偏差（飞机世界位置 + MSL 高度）。
    pub fn ils_deviation(&self, ils: &Ils, x: f64, z: f64, alt_msl: f64) -> IlsDeviation {
        let end = ils.end;
        let (ddx, ddz) = self.landing_dir(end);
        let t = self.ends[end];
        let vx = wrapped_dx(t.0, x);
        let vz = z - t.1;
        let along = vx * ddx + vz * ddz; // 自入口起，沿着陆方向
        let lateral = vx * (-ddz) + vz * ddx; // 右侧为正
        // 航向道天线位于远端：距离 = L - along
        let a = self.length - along;
        let loc = lateral.atan2(a).to_degrees();
        // 下滑道天线位于入口后 300 m
        let d = 300.0 - along;
        let gs_angle = (alt_msl - self.elevation).atan2(d).to_degrees();
        IlsDeviation {
            loc_deg: loc,
            gs_deg: gs_angle - ils.glide_deg,
            dist_to_threshold: -along,
            loc_valid: a > 0.0 && a < 33_000.0 && loc.abs() < 35.0 && along < self.length,
            gs_valid: d > 0.0 && d < 28_000.0 && lateral.abs() < d * 0.6,
        }
    }
}

/// 网格单元索引。
pub fn cell_of(x: f64, z: f64) -> (i32, i32) {
    let i = (((wrap_x(x) + WORLD_WIDTH / 2.0) / CELL_W).floor() as i32).rem_euclid(GRID_NX);
    let j = (((clamp_z(z) + WORLD_HEIGHT / 2.0) / CELL_H).floor() as i32).clamp(0, GRID_NZ - 1);
    (i, j)
}

fn letter(h: f64) -> char {
    (b'A' + ((h * 26.0) as u8).min(25)) as char
}

fn make_name(seed: u64, ci: i64, cj: i64, kind: AirportKind) -> String {
    const A: [&str; 12] = ["Ka", "Mor", "Del", "Ves", "Tar", "Lun", "Bri", "Oso", "Fen", "Aru", "Nor", "Sal"];
    const B: [&str; 10] = ["vik", "ton", "mar", "dale", "haven", "ria", "stad", "port", "ford", "mere"];
    let a = A[(hash01(seed, ci, cj, 31) * 12.0) as usize % 12];
    let b = B[(hash01(seed, ci, cj, 32) * 10.0) as usize % 10];
    let suffix = match kind {
        AirportKind::International => " Intl",
        AirportKind::Regional => " Regional",
        AirportKind::Dirt => " Strip",
    };
    format!("{a}{b}{suffix}")
}

/// 确定性生成全球机场；返回列表（按网格行优先）。
pub fn generate(terrain: &TerrainGen) -> Vec<Airport> {
    let seed = terrain.seed;
    let mut out: Vec<Airport> = Vec::new();
    let mut used: HashSet<String> = HashSet::new();

    for cj in 0..GRID_NZ {
        for ci in 0..GRID_NX {
            let (cj64, ci64) = (cj as i64, ci as i64);
            if hash01(seed, ci64, cj64, 1) > 0.78 {
                continue;
            }
            let r = hash01(seed, ci64, cj64, 2);
            let want = if r < 0.14 {
                AirportKind::International
            } else if r < 0.52 {
                AirportKind::Regional
            } else {
                AirportKind::Dirt
            };
            let kinds: &[AirportKind] = match want {
                AirportKind::International => &[AirportKind::International, AirportKind::Regional, AirportKind::Dirt],
                AirportKind::Regional => &[AirportKind::Regional, AirportKind::Dirt],
                AirportKind::Dirt => &[AirportKind::Dirt],
            };
            let x0 = -WORLD_WIDTH / 2.0 + ci as f64 * CELL_W;
            let z0 = -WORLD_HEIGHT / 2.0 + cj as f64 * CELL_H;

            let mut chosen: Option<Airport> = None;
            'kinds: for &kind in kinds {
                let mut best: Option<(f64, Airport)> = None;
                for k in 0..16i64 {
                    let ux = hash01(seed, ci64, cj64, 100 + k * 2);
                    let uz = hash01(seed, ci64, cj64, 101 + k * 2);
                    let cx = wrap_x(x0 + 5_000.0 + ux * (CELL_W - 10_000.0));
                    let cz = z0 + 5_000.0 + uz * (CELL_H - 10_000.0);
                    let (lat, lon) = world_to_latlon(cx, cz);
                    if lat.abs() > 78.0 {
                        continue;
                    }
                    // 跑道对齐盛行风：着陆迎风
                    let (from, _) = prevailing_wind(lat);
                    let mut n = ((from / 10.0).round() as i32).rem_euclid(18);
                    if n == 0 {
                        n = 18;
                    }
                    let heading0 = n as f64 * 10.0;
                    let len = kind.runway_length();
                    let (dx, dz) = heading_dir(heading0);
                    let (px, pz) = (-dz, dx);
                    // 选址：沿跑道及两侧采样
                    let (mut lo, mut hi, mut sum, mut cnt) = (f64::MAX, f64::MIN, 0.0, 0.0);
                    let lat_off = kind.flat_radius() + kind.blend_width() * 0.25;
                    let mut all_land = true;
                    let steps = 14;
                    for s in 0..=steps {
                        let a = -len / 2.0 - 120.0 + (len + 240.0) * s as f64 / steps as f64;
                        for &o in &[0.0, lat_off, -lat_off] {
                            let h = terrain.base_height(cx + dx * a + px * o, cz + dz * a + pz * o);
                            lo = lo.min(h);
                            hi = hi.max(h);
                            if h < 2.5 || h > 2200.0 {
                                all_land = false;
                            }
                            if o == 0.0 {
                                sum += h;
                                cnt += 1.0;
                            }
                        }
                    }
                    if !all_land || hi - lo > kind.max_relief() {
                        continue;
                    }
                    let elev = sum / cnt;
                    // 偏好沿海平原/低洼谷地：评分越低越好
                    let score = (hi - lo) + 0.01 * elev;
                    if best.as_ref().map_or(true, |b| score < b.0) {
                        let ends = [
                            (wrap_x(cx - dx * len / 2.0), cz - dz * len / 2.0),
                            (wrap_x(cx + dx * len / 2.0), cz + dz * len / 2.0),
                        ];
                        best = Some((
                            score,
                            Airport {
                                icao: String::new(),
                                name: make_name(seed, ci64, cj64, kind),
                                kind,
                                center: (cx, cz),
                                elevation: elev,
                                lat,
                                lon,
                                heading0,
                                length: len,
                                width: kind.runway_width(),
                                flat_radius: kind.flat_radius(),
                                blend_width: kind.blend_width(),
                                asphalt: kind.asphalt(),
                                ils: Vec::new(),
                                vor: None,
                                ends,
                            },
                        ));
                    }
                }
                if let Some((_, ap)) = best {
                    chosen = Some(ap);
                    break 'kinds;
                }
            }
            let Some(mut ap) = chosen else { continue };

            // ICAO：区域前缀(经度扇区/纬度带) + 2 位哈希字母，保证唯一
            let p1 = (b'A' + ((ap.lon + 180.0) / 360.0 * 20.0).floor().clamp(0.0, 19.0) as u8) as char;
            let mut salt = 0;
            loop {
                let p2 = (b'A' + (((ap.lat + 90.0) / 180.0) * 8.0).floor().clamp(0.0, 7.0) as u8) as char;
                let c3 = letter(hash01(seed, ci64, cj64, 40 + salt));
                let c4 = letter(hash01(seed, ci64, cj64, 41 + salt));
                let code = format!("{p1}{p2}{c3}{c4}");
                if used.insert(code.clone()) {
                    ap.icao = code;
                    break;
                }
                salt += 2;
            }

            // 无线电：VOR 112.00–117.95 MHz；ILS LOC 108.10–111.95 MHz（十分位为奇数）
            let vor_f = 112.0 + 0.05 * (hash01(seed, ci64, cj64, 50) * 120.0).floor();
            let has_vor = ap.kind == AirportKind::International || (ap.kind == AirportKind::Regional && hash01(seed, ci64, cj64, 51) < 0.5);
            if has_vor {
                let (vx, vz) = ap.runway_point(ap.length * 0.5, ap.width / 2.0 + 180.0);
                ap.vor = Some(Vor { ident: ap.icao[1..].to_string(), freq_mhz: vor_f, pos: (vx, vz) });
            }
            if ap.kind == AirportKind::International {
                let k = (hash01(seed, ci64, cj64, 52) * 20.0).floor();
                let f0 = 108.1 + 0.2 * k;
                let f1 = 108.1 + 0.2 * ((k + 7.0) % 20.0);
                for (e, f) in [(0usize, f0), (1usize, f1)] {
                    ap.ils.push(Ils {
                        ident: format!("I{}", &ap.icao[1..]),
                        freq_mhz: (f * 100.0).round() / 100.0,
                        end: e,
                        course_deg: ap.heading_of_end(e),
                        glide_deg: 3.0,
                    });
                }
            }
            out.push(ap);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny() -> Airport {
        Airport {
            icao: "TEST".into(),
            name: "t".into(),
            kind: AirportKind::International,
            center: (0.0, 0.0),
            elevation: 100.0,
            lat: 0.0,
            lon: 0.0,
            heading0: 90.0,
            length: 3000.0,
            width: 45.0,
            flat_radius: 110.0,
            blend_width: 900.0,
            asphalt: true,
            ils: vec![],
            vor: None,
            ends: [(-1500.0, 0.0), (1500.0, 0.0)],
        }
    }

    #[test]
    fn designators_and_geometry() {
        let a = tiny();
        assert_eq!(a.runway_name(), "09/27");
        assert!(a.capsule_sdf(0.0, 0.0) < 0.0);
        assert!(a.capsule_sdf(0.0, 500.0) > 0.0);
        assert!(a.capsule_sdf(1500.0 + 200.0, 0.0) > 0.0);
        let (al, la) = a.runway_local(0.0, 20.0);
        assert!((al - 1500.0).abs() < 1e-6 && (la - 20.0).abs() < 1e-6); // 朝东，右侧 = 南 = +Z
    }

    #[test]
    fn ils_on_glidepath_has_zero_deviation() {
        let a = tiny();
        let ils = Ils { ident: "IXXX".into(), freq_mhz: 109.1, end: 0, course_deg: 90.0, glide_deg: 3.0 };
        // 在端 0 入口西侧 6 km、航道中线、3° 下滑道上（下滑道天线在入口后 300 m）
        let d = 6000.0 + 300.0;
        let alt = a.elevation + d * 3f64.to_radians().tan();
        let dev = a.ils_deviation(&ils, -1500.0 - 6000.0, 0.0, alt);
        assert!(dev.loc_deg.abs() < 1e-6 && dev.gs_deg.abs() < 1e-6, "{dev:?}");
        assert!(dev.loc_valid && dev.gs_valid);
        // 在航道右侧（南）：+ ；在下滑道上方：+
        let dev2 = a.ils_deviation(&ils, -7500.0, 200.0, alt + 100.0);
        assert!(dev2.loc_deg > 0.0 && dev2.gs_deg > 0.0);
    }

    #[test]
    fn papi_has_four_lights_left_of_runway() {
        let a = tiny();
        let p = a.papi(0);
        for &(_, z) in &p.positions {
            assert!(z < -(a.width / 2.0)); // 朝东着陆，左侧 = 北 = -Z
        }
    }
}
