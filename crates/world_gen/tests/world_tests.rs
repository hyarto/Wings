//! M2 验收：确定性、海洋占比、机场平坦度、季节反转、经度环绕无缝。

use world_gen::airport::AirportKind;
use world_gen::climate;
use world_gen::coords::*;
use world_gen::terrain::MAX_LAND_HEIGHT;
use world_gen::World;

const SEED: u64 = 20240601;

fn world() -> &'static World {
    use std::sync::OnceLock;
    static W: OnceLock<World> = OnceLock::new();
    W.get_or_init(|| World::new(SEED))
}

#[test]
fn same_seed_is_fully_deterministic() {
    let a = World::new(777);
    let b = World::new(777);
    for i in 0..2000 {
        let x = wrap_x(i as f64 * 613.7 - 400_000.0);
        let z = (i as f64 * 211.3 % WORLD_HEIGHT) - WORLD_HEIGHT / 2.0;
        assert_eq!(a.height(x, z).to_bits(), b.height(x, z).to_bits());
    }
    assert_eq!(a.airports().len(), b.airports().len());
    for (p, q) in a.airports().iter().zip(b.airports()) {
        assert_eq!(p.icao, q.icao);
        assert_eq!(p.center, q.center);
        assert_eq!(p.elevation.to_bits(), q.elevation.to_bits());
        assert_eq!(p.runway_name(), q.runway_name());
    }
    let c = World::new(778);
    let diff = (0..200).filter(|i| {
        let x = *i as f64 * 5000.0 - 500_000.0;
        (a.height(x, 1000.0) - c.height(x, 1000.0)).abs() > 1.0
    }).count();
    assert!(diff > 100, "different seeds must differ");
}

#[test]
fn ocean_fraction_is_reasonable() {
    for seed in [1u64, 42, 777, SEED] {
        let w = World::new(seed);
        let (nx, nz) = (160usize, 80usize);
        let (mut ocean_sphere, mut total_sphere, mut ocean_map) = (0.0, 0.0, 0usize);
        for j in 0..nz {
            let lat = 90.0 - (j as f64 + 0.5) / nz as f64 * 180.0;
            let wgt = lat.to_radians().cos();
            for i in 0..nx {
                let lon = -180.0 + (i as f64 + 0.5) / nx as f64 * 360.0;
                let (x, z) = latlon_to_world(lat, lon);
                let o = w.is_ocean(x, z);
                if o {
                    ocean_sphere += wgt;
                    ocean_map += 1;
                }
                total_sphere += wgt;
            }
        }
        let fs = ocean_sphere / total_sphere;
        let fm = ocean_map as f64 / (nx * nz) as f64;
        println!("seed {seed}: ocean sphere {:.1}% map {:.1}%", fs * 100.0, fm * 100.0);
        assert!((0.55..=0.65).contains(&fs), "sphere ocean fraction {fs}");
        assert!((0.48..=0.72).contains(&fm), "map ocean fraction {fm}");
    }
}

#[test]
fn terrain_heights_are_bounded_and_have_mountains() {
    let w = world();
    let (mut max_h, mut min_h) = (f64::MIN, f64::MAX);
    for j in 0..120 {
        for i in 0..240 {
            let x = -WORLD_WIDTH / 2.0 + (i as f64 + 0.5) / 240.0 * WORLD_WIDTH;
            let z = -WORLD_HEIGHT / 2.0 + (j as f64 + 0.5) / 120.0 * WORLD_HEIGHT;
            let h = w.height(x, z);
            max_h = max_h.max(h);
            min_h = min_h.min(h);
        }
    }
    println!("height range {min_h:.0} .. {max_h:.0}");
    assert!(max_h <= MAX_LAND_HEIGHT + 1e-6);
    assert!(max_h > 2500.0, "world should contain real mountains, max {max_h}");
    assert!(min_h < -1500.0);
}

#[test]
fn longitude_wraps_seamlessly() {
    let w = world();
    let half = WORLD_WIDTH / 2.0;
    for k in 0..200 {
        let z = -WORLD_HEIGHT / 2.0 + (k as f64 + 0.5) / 200.0 * WORLD_HEIGHT;
        // 周期性
        assert_eq!(w.height(half - 3.0, z).to_bits(), w.height(-half - 3.0, z).to_bits());
        // 跨 ±180° 无跳变：相距 2 m 的两点高度差 < 3 m
        let a = w.height(half - 1.0, z);
        let b = w.height(-half + 1.0, z);
        assert!((a - b).abs() < 3.0, "seam jump {} at z {z}", (a - b).abs());
    }
}

#[test]
fn airports_exist_unique_and_well_formed() {
    let w = world();
    let aps = w.airports();
    println!("{} airports", aps.len());
    assert!(aps.len() > 40, "got {}", aps.len());
    let mut seen = std::collections::HashSet::new();
    let (mut d, mut r, mut i) = (0, 0, 0);
    for a in aps {
        assert_eq!(a.icao.len(), 4);
        assert!(a.icao.chars().all(|c| c.is_ascii_uppercase()));
        assert!(seen.insert(a.icao.clone()), "dup ICAO {}", a.icao);
        match a.kind {
            AirportKind::Dirt => { d += 1; assert_eq!(a.length, 600.0); assert!(!a.asphalt); }
            AirportKind::Regional => { r += 1; assert_eq!(a.length, 1500.0); assert!(a.ils.is_empty()); }
            AirportKind::International => {
                i += 1;
                assert_eq!(a.length, 3000.0);
                assert_eq!(a.ils.len(), 2, "hub needs ILS");
                for ils in &a.ils {
                    assert!((108.1..=111.95).contains(&ils.freq_mhz), "ILS {}", ils.freq_mhz);
                    assert!((((ils.freq_mhz * 10.0).round() as i64) % 2) == 1, "odd tenth {}", ils.freq_mhz);
                }
            }
        }
        if let Some(v) = &a.vor {
            assert!((112.0..=117.95).contains(&v.freq_mhz));
        }
        // 跑道名
        let name = a.runway_name();
        assert_eq!(name.len(), 5);
        let n0: i32 = name[0..2].parse().unwrap();
        let n1: i32 = name[3..5].parse().unwrap();
        assert!(n0 >= 1 && n0 <= 18 && n1 == n0 + 18);
        // 跑道对齐盛行风（±10°）
        let (from, _) = climate::prevailing_wind(a.lat);
        let diff = ((a.heading0 - from + 540.0) % 360.0 - 180.0).abs().min(((a.heading0 + 180.0 - from + 540.0) % 360.0 - 180.0).abs());
        assert!(diff <= 10.0, "runway {} not aligned with wind from {from} at {}", name, a.lat);
        // 海拔合理（陆地、低地）
        assert!(a.elevation > 2.0 && a.elevation < 2200.0);
    }
    assert!(d > 3 && r > 3 && i > 1, "kinds: dirt {d} regional {r} intl {i}");
}

#[test]
fn airport_runway_areas_are_perfectly_flat() {
    let w = world();
    for a in w.airports() {
        let n = (a.length / 10.0) as usize;
        let mut max_err: f64 = 0.0;
        for s in 0..=n {
            let along = s as f64 * a.length / n as f64;
            for lat in [-a.width / 2.0, 0.0, a.width / 2.0, a.flat_radius * 0.9, -a.flat_radius * 0.9] {
                let (x, z) = a.runway_point(along, lat);
                max_err = max_err.max((w.height(x, z) - a.elevation).abs());
            }
        }
        // 两端半圆区域
        for ang in 0..16 {
            let t = ang as f64 / 16.0 * std::f64::consts::TAU;
            for e in 0..2 {
                let (ex, ez) = a.ends[e];
                let r = a.flat_radius * 0.95;
                let x = ex + t.cos() * r;
                let z = ez + t.sin() * r;
                if a.capsule_sdf(x, z) <= 0.0 {
                    max_err = max_err.max((w.height(x, z) - a.elevation).abs());
                }
            }
        }
        assert!(max_err < 0.05, "{} runway height error {max_err}", a.icao);
        // 相邻 10 m 采样之间坡度误差 < 0.05 m
        let mut max_step: f64 = 0.0;
        let mut prev = None;
        for s in 0..=n {
            let (x, z) = a.runway_point(s as f64 * a.length / n as f64, 0.0);
            let h = w.height(x, z);
            if let Some(p) = prev { max_step = f64::max(max_step, (h - p as f64).abs()); }
            prev = Some(h);
        }
        assert!(max_step < 0.05, "{} step {max_step}", a.icao);
    }
}

#[test]
fn terrain_blends_smoothly_around_airports() {
    let w = world();
    for a in w.airports().iter().take(40) {
        // 沿垂直跑道方向外推，高度变化应连续（相邻 20 m 点差 < 20 m，且无穿模台阶）
        let mut prev = None;
        for k in 0..60 {
            let lat = a.width / 2.0 + k as f64 * 25.0;
            let (x, z) = a.runway_point(a.length / 2.0, lat);
            let h = w.height(x, z);
            if let Some(p) = prev { assert!((h - p as f64).abs() < 25.0, "{} cliff {} at {k}", a.icao, (h - p as f64).abs()); }
            prev = Some(h);
        }
    }
}

#[test]
fn hemispheres_have_opposite_seasons_in_world() {
    let w = world();
    // 在南北对称纬度各找一个陆地点（同经度），比较同一日期温度
    let mut pairs = 0;
    for k in 0..400 {
        let lon = -180.0 + k as f64 * 0.9;
        let (xn, zn) = latlon_to_world(45.0, lon);
        let (xs, zs) = latlon_to_world(-45.0, lon);
        let (n_jan, s_jan) = (w.climate(xn, zn, 15.0).temp_sl, w.climate(xs, zs, 15.0).temp_sl);
        let (n_jul, s_jul) = (w.climate(xn, zn, 196.0).temp_sl, w.climate(xs, zs, 196.0).temp_sl);
        assert!(n_jan < s_jan && n_jul > s_jul, "reversal failed at lon {lon}");
        pairs += 1;
    }
    assert!(pairs > 300);
}

#[test]
fn snowline_moves_with_season_on_the_same_mountain() {
    let w = world();
    // 找到北半球中纬度的一座高山
    let mut found = None;
    'o: for j in 0..200 {
        for i in 0..400 {
            let lat = 35.0 + j as f64 * 0.1;
            let lon = -180.0 + i as f64 * 0.9;
            let (x, z) = latlon_to_world(lat, lon);
            let h = w.height(x, z);
            if h > 3600.0 { found = Some((x, z, h)); break 'o; }
        }
    }
    let (x, z, h) = found.expect("a high mountain in northern mid-latitudes");
    let jan = w.climate(x, z, 15.0).snow_line;
    let jul = w.climate(x, z, 196.0).snow_line;
    println!("mountain {h:.0} m: snowline jan {jan:.0} jul {jul:.0}");
    assert!(jan + 800.0 < jul);
    // 冬季同一座山积雪面积更大：山顶冬季必为雪，且至少冬季的雪线比夏季低
    assert!(matches!(w.biome(x, z, 15.0), climate::Biome::Snow | climate::Biome::IceCap | climate::Biome::BareRock));
}
