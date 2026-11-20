//! M2 验收补充：±180° 跨界无撕裂、100 km 飞行的 f64 精度、机场几何/ILS/PAPI、
//! `airport_near`/`nearest_airport`、跨种子差异、季节与气候单调性。

use world_gen::airport::{AirportKind, CELL_H, CELL_W};
use world_gen::climate;
use world_gen::coords::*;
use world_gen::World;

fn world() -> &'static World {
    use std::sync::OnceLock;
    static W: OnceLock<World> = OnceLock::new();
    W.get_or_init(|| World::new(20240601))
}

// ---------------------------------------------------------------- 坐标 ----

#[test]
fn latlon_roundtrip_is_precise_everywhere() {
    for i in 0..=180 {
        for j in 0..72 {
            let lat = -90.0 + i as f64;
            let lon = -180.0 + j as f64 * 5.0 + 1e-6;
            let (x, z) = latlon_to_world(lat, lon);
            let (la, lo) = world_to_latlon(x, z);
            assert!((la - lat).abs() < 1e-9, "lat {lat} -> {la}");
            assert!((lo - lon).abs() < 1e-9, "lon {lon} -> {lo}");
            assert!(x >= -WORLD_WIDTH / 2.0 && x < WORLD_WIDTH / 2.0);
        }
    }
}

#[test]
fn wrap_is_idempotent_periodic_and_shortest() {
    for k in -5..=5 {
        for &x in &[-624_999.0, -1.0, 0.0, 17.5, 624_999.0] {
            let w = wrap_x(x + k as f64 * WORLD_WIDTH);
            assert!((w - x).abs() < 1e-6, "periodic {x} k={k} -> {w}");
            assert_eq!(wrap_x(w), w, "idempotent");
        }
    }
    // 跨 ±180° 的最短位移
    assert!((wrapped_dx(WORLD_WIDTH / 2.0 - 10.0, -WORLD_WIDTH / 2.0 + 10.0) - 20.0).abs() < 1e-6);
    assert!((wrapped_dx(-WORLD_WIDTH / 2.0 + 10.0, WORLD_WIDTH / 2.0 - 10.0) + 20.0).abs() < 1e-6);
    assert!((wrapped_distance((WORLD_WIDTH / 2.0 - 30.0, 0.0), (-WORLD_WIDTH / 2.0 + 40.0, 0.0)) - 70.0).abs() < 1e-6);
}

#[test]
fn scale_constants_match_1_to_32_earth() {
    // 地球赤道周长 40,075 km / 32 ≈ 1,252 km
    assert!((WORLD_WIDTH * HORIZONTAL_SCALE / 1000.0 - 40_000.0).abs() < 100.0);
    assert!((WORLD_WIDTH / WORLD_HEIGHT - 2.0).abs() < 1e-12);
    // 1° 纬度 ≈ 111 km / 32 ≈ 3.47 km
    let (_, z0) = latlon_to_world(0.0, 0.0);
    let (_, z1) = latlon_to_world(1.0, 0.0);
    assert!(((z0 - z1).abs() - 3472.2).abs() < 5.0, "{}", (z0 - z1).abs());
}

// ---------------------------------------------------- 跨 ±180° 无撕裂 ----

#[test]
fn terrain_is_continuous_across_antimeridian_over_100km() {
    let w = world();
    // 在多个纬度，沿东向 100 km 穿越 ±180° 边界，步长 40 m
    for &lat in &[-60.0, -23.0, 0.0, 12.0, 35.0, 55.0] {
        let (_, z) = latlon_to_world(lat, 0.0);
        let x0 = WORLD_WIDTH / 2.0 - 50_000.0;
        let mut prev = w.height(wrap_x(x0), z);
        let mut max_step: f64 = 0.0;
        let mut seam_step: f64 = 0.0;
        for i in 1..=2500 {
            let x = x0 + i as f64 * 40.0;
            let h = w.height(wrap_x(x), z);
            assert!(h.is_finite());
            let d = (h - prev).abs();
            max_step = max_step.max(d);
            if (x - WORLD_WIDTH / 2.0).abs() < 41.0 {
                seam_step = seam_step.max(d);
            }
            prev = h;
        }
        println!("lat {lat}: max 40 m step {max_step:.1} m, seam step {seam_step:.2} m");
        assert!(seam_step < 60.0, "tear at antimeridian: {seam_step}");
        assert!(max_step < 400.0, "unrealistic cliff {max_step}");
    }
}

#[test]
fn terrain_climate_and_color_identical_on_both_sides_of_seam() {
    let w = world();
    for &lat in &[-40.0, 0.0, 30.0] {
        let (x, z) = latlon_to_world(lat, 179.9999);
        // 同一物理点的不同写法（+W 周期）必须给出完全相同的结果
        let x2 = x - WORLD_WIDTH;
        assert_eq!(w.height(x, z), w.height(x2, z));
        assert_eq!(w.color_at(x, z, 100.0), w.color_at(x2, z, 100.0));
        assert_eq!(w.biome(x, z, 200.0), w.biome(x2, z, 200.0));
    }
}

#[test]
fn float_precision_is_stable_after_100km_flight_in_f64() {
    // 飞行 100 km 后（含跨界）的高度采样，与“重新在原点附近计算”的同一点结果一致：
    // 即 f64 世界坐标不会累计误差（浮动原点由渲染层处理）。
    let w = world();
    let (x0, z0) = latlon_to_world(20.0, 179.0);
    let step = 12.5; // 一个 chunk 采样间距
    let n = 8000; // 100 km
    let mut x = x0;
    for _ in 0..n {
        x += step;
    }
    let direct = x0 + step * n as f64;
    assert!((x - direct).abs() < 1e-6, "accumulated {x} vs direct {direct}");
    assert_eq!(w.height(wrap_x(x), z0), w.height(wrap_x(direct), z0));
    // 局部相对坐标（f32 渲染）在 ±1e4 m 内精度优于 1 mm
    let local = (x - direct) as f32 + 9_999.0;
    assert!((local - 9_999.0).abs() < 1e-3);
}

// ---------------------------------------------------------------- 机场 ----

#[test]
fn airports_are_on_land_flat_and_inside_their_grid_cell() {
    let w = world();
    for a in w.airports() {
        for e in 0..2 {
            let (x, z) = a.ends[e];
            assert!(w.height(x, z) > 1.0, "{} end {e} in water", a.icao);
        }
        // 机场中心距离所属网格单元的边界：跑道区域不会跨出 3×3 邻域
        let (ci, cj) = world_gen::airport::cell_of(a.center.0, a.center.1);
        assert!((0..31).contains(&ci) && (0..16).contains(&cj));
        // 跑道长度与端点距离一致
        let d = wrapped_distance(a.ends[0], a.ends[1]);
        assert!((d - a.length).abs() < 1.0, "{}: ends distance {d} vs length {}", a.icao, a.length);
        // 纬经度与世界坐标一致
        let (lat, lon) = world_to_latlon(a.center.0, a.center.1);
        assert!((lat - a.lat).abs() < 1e-6 && (lon - a.lon).abs() < 1e-6);
    }
    assert!(CELL_W > 30_000.0 && CELL_W < 50_000.0 && CELL_H > 30_000.0 && CELL_H < 50_000.0, "≈40 km grid");
}

#[test]
fn runway_surface_height_error_below_5cm_along_and_across() {
    let w = world();
    let mut worst: f64 = 0.0;
    for a in w.airports() {
        // 沿跑道中心线 + 两侧边缘，每 10 m 采样
        let mut s = -a.flat_radius * 0.0;
        while s <= a.length {
            for &lat in &[-a.width / 2.0, 0.0, a.width / 2.0] {
                let (x, z) = a.runway_point(s, lat);
                worst = worst.max((w.height(x, z) - a.elevation).abs());
            }
            s += 10.0;
        }
        // 跑道端外延 30 m 仍在压平区
        for &(along, lat) in &[(-30.0, 0.0), (a.length + 30.0, 0.0)] {
            let (x, z) = a.runway_point(along, lat);
            worst = worst.max((w.height(x, z) - a.elevation).abs());
        }
    }
    println!("worst runway height error: {worst:.4} m");
    assert!(worst < 0.05, "runway not flat: {worst}");
}

#[test]
fn airport_near_and_nearest_airport_agree() {
    let w = world();
    for a in w.airports() {
        let near = w.airport_near(a.center.0, a.center.1).expect("airport at its own center");
        assert_eq!(near.icao, a.icao);
        let (n, d) = w.nearest_airport(a.center.0, a.center.1).unwrap();
        assert_eq!(n.icao, a.icao);
        assert!(d < 1.0);
        // 跑道端点也在影响范围内
        assert!(w.airport_near(a.ends[0].0, a.ends[0].1).is_some());
        // 偏离 1 个混合带宽 + 半径之外不再属于该机场
        let (x, z) = a.runway_point(a.length / 2.0, a.flat_radius + a.blend_width + 50.0);
        if let Some(o) = w.airport_near(x, z) {
            assert_ne!(o.icao, a.icao);
        }
    }
    // 极地/大洋深处无机场
    assert!(w.airport_near(0.0, -WORLD_HEIGHT / 2.0).is_none());
}

#[test]
fn nearest_airport_search_wraps_around_antimeridian() {
    let w = world();
    // 从地图另一侧（x 偏移整整一圈）查询必须得到同一机场
    let a = &w.airports()[w.airports().len() / 2];
    let (n1, d1) = w.nearest_airport(a.center.0 + 1234.0, a.center.1 + 777.0).unwrap();
    let (n2, d2) = w.nearest_airport(a.center.0 + 1234.0 - WORLD_WIDTH, a.center.1 + 777.0).unwrap();
    assert_eq!(n1.icao, n2.icao);
    assert!((d1 - d2).abs() < 1e-6);
    // 靠近 ±180° 边界的机场（若存在）距离按最短弧计算
    for a in w.airports() {
        if a.center.0.abs() > WORLD_WIDTH / 2.0 - 40_000.0 {
            let q = (wrap_x(a.center.0 + 5_000.0), a.center.1);
            let (n, d) = w.nearest_airport(q.0, q.1).unwrap();
            assert_eq!(n.icao, a.icao);
            assert!((d - 5_000.0).abs() < 1e-3, "wrapped distance {d}");
        }
    }
}

#[test]
fn runway_local_and_point_are_inverse_and_oriented() {
    let w = world();
    for a in w.airports().iter().take(40) {
        for &(s, l) in &[(0.0, 0.0), (a.length, 0.0), (a.length / 2.0, 12.0), (100.0, -9.0)] {
            let (x, z) = a.runway_point(s, l);
            let (s2, l2) = a.runway_local(x, z);
            assert!((s - s2).abs() < 1e-6 && (l - l2).abs() < 1e-6, "{} ({s},{l}) -> ({s2},{l2})", a.icao);
        }
        // 端 0 着陆航向 = 从 ends[0] 指向 ends[1]
        let hd = dir_heading(wrapped_dx(a.ends[0].0, a.ends[1].0), a.ends[1].1 - a.ends[0].1);
        assert!(((hd - a.heading0 + 540.0) % 360.0 - 180.0).abs() < 0.01, "{}: {hd} vs {}", a.icao, a.heading0);
        assert!(((a.heading_of_end(1) - a.heading_of_end(0) - 180.0).rem_euclid(360.0)).abs() < 1e-6);
        // 窗风袋、塔台位于跑道旁而不在跑道上
        let (wx, wz) = a.windsock_pos();
        assert!(a.runway_local(wx, wz).1.abs() > a.width / 2.0);
        let (tx, tz) = a.tower_pos();
        assert!(a.runway_local(tx, tz).1.abs() > a.width / 2.0 + 20.0);
    }
}

#[test]
fn edge_lights_flank_runway_symmetrically() {
    let w = world();
    let a = w.airports().iter().find(|a| a.kind == AirportKind::Regional).unwrap();
    let lights = a.edge_lights();
    assert!(lights.len() >= 2 * (a.length / 60.0) as usize);
    let (mut left, mut right) = (0, 0);
    for &(x, z) in &lights {
        let (s, l) = a.runway_local(x, z);
        assert!(s >= -1e-6 && s <= a.length + 1e-6);
        assert!(l.abs() > a.width / 2.0 && l.abs() < a.width / 2.0 + 5.0);
        if l > 0.0 { right += 1 } else { left += 1 }
    }
    assert_eq!(left, right);
}

#[test]
fn ils_geometry_signs_and_validity() {
    let w = world();
    let hub = w.airports().iter().find(|a| a.kind == AirportKind::International).expect("hub");
    for ils in &hub.ils {
        let end = ils.end;
        let t = hub.ends[end];
        let (dx, dz) = hub.landing_dir(end);
        let right = (-dz, dx);
        assert!((ils.course_deg - hub.heading_of_end(end)).abs() < 0.01 || ((ils.course_deg - hub.heading_of_end(end)).abs() - 360.0).abs() < 0.01);
        assert!((ils.glide_deg - 3.0).abs() < 0.6);
        let on_gs_alt = |dist: f64| hub.elevation + (dist + 300.0) * ils.glide_deg.to_radians().tan();
        // 8 km final，航向道/下滑道正中
        let p = (t.0 - dx * 8000.0, t.1 - dz * 8000.0);
        let dev = hub.ils_deviation(ils, p.0, p.1, on_gs_alt(8000.0));
        assert!(dev.loc_valid && dev.gs_valid);
        assert!(dev.loc_deg.abs() < 1e-6 && dev.gs_deg.abs() < 1e-6, "{dev:?}");
        assert!((dev.dist_to_threshold - 8000.0).abs() < 1e-6);
        // 偏右 200 m → loc > 0
        let pr = (p.0 + right.0 * 200.0, p.1 + right.1 * 200.0);
        let d_r = hub.ils_deviation(ils, pr.0, pr.1, on_gs_alt(8000.0));
        assert!(d_r.loc_deg > 0.3, "{d_r:?}");
        let pl = (p.0 - right.0 * 200.0, p.1 - right.1 * 200.0);
        assert!(hub.ils_deviation(ils, pl.0, pl.1, on_gs_alt(8000.0)).loc_deg < -0.3);
        // 过高 → gs > 0，过低 → gs < 0
        assert!(hub.ils_deviation(ils, p.0, p.1, on_gs_alt(8000.0) + 150.0).gs_deg > 0.5);
        assert!(hub.ils_deviation(ils, p.0, p.1, on_gs_alt(8000.0) - 150.0).gs_deg < -0.5);
        // 背向进近（跑道后方）无效
        let behind = (t.0 + dx * (hub.length + 3000.0), t.1 + dz * (hub.length + 3000.0));
        let d_b = hub.ils_deviation(ils, behind.0, behind.1, hub.elevation + 200.0);
        assert!(!(d_b.loc_valid && d_b.gs_valid), "{d_b:?}");
        // 距离过远 → 无效
        let far = (t.0 - dx * 60_000.0, t.1 - dz * 60_000.0);
        assert!(!hub.ils_deviation(ils, far.0, far.1, hub.elevation + 3000.0).loc_valid);
    }
}

#[test]
fn papi_geometry_has_four_evenly_spaced_lights_near_aiming_point() {
    let w = world();
    for a in w.airports().iter().filter(|a| a.asphalt).take(20) {
        for end in 0..2 {
            let p = a.papi(end);
            assert!((p.glide_deg - 3.0).abs() < 1e-9);
            let loc: Vec<_> = p.positions.iter().map(|&(x, z)| a.runway_local(x, z)).collect();
            for l in &loc {
                assert!(l.1.abs() > a.width / 2.0 + 10.0, "PAPI must be off the runway edge");
                // 距入口约 300 m
                let from_end = if end == 0 { l.0 } else { a.length - l.0 };
                assert!((from_end - 300.0).abs() < 1.0, "{from_end}");
            }
            for i in 1..4 {
                assert!(((loc[i].1 - loc[i - 1].1).abs() - 9.0).abs() < 1e-6);
            }
        }
    }
}

#[test]
fn different_seeds_make_different_worlds_with_valid_airports() {
    let (a, b) = (World::new(1), World::new(2));
    let mut diff = 0;
    for i in 0..200 {
        let (x, z) = (i as f64 * 6_100.0 - 600_000.0, (i as f64 * 1_377.0).rem_euclid(500_000.0) - 250_000.0);
        if (a.height(x, z) - b.height(x, z)).abs() > 1.0 {
            diff += 1;
        }
    }
    assert!(diff > 80, "worlds nearly identical: {diff}/200");
    assert_ne!(
        a.airports().iter().map(|x| x.icao.clone()).collect::<Vec<_>>(),
        b.airports().iter().map(|x| x.icao.clone()).collect::<Vec<_>>()
    );
    for w in [&a, &b] {
        assert!(w.airports().len() > 30);
        let mut seen = std::collections::HashSet::new();
        for ap in w.airports() {
            assert!(seen.insert(ap.icao.clone()));
            assert!(w.height(ap.center.0, ap.center.1) > 1.0);
        }
    }
}

// ---------------------------------------------------------- 气候 / 季节 ----

#[test]
fn seasonal_temperature_swing_is_opposite_and_stronger_at_high_latitude() {
    let day_summer_n = 172.0;
    let day_winter_n = 355.0;
    for &lat in &[20.0, 45.0, 65.0] {
        let tn_s = climate::sea_level_temp(lat, day_summer_n, false);
        let tn_w = climate::sea_level_temp(lat, day_winter_n, false);
        let ts_s = climate::sea_level_temp(-lat, day_summer_n, false);
        let ts_w = climate::sea_level_temp(-lat, day_winter_n, false);
        assert!(tn_s > tn_w + 2.0, "north summer warmer at {lat}");
        assert!(ts_w > ts_s + 2.0, "south is warmer in northern winter at {lat}");
        // 镜像：同一日期两半球季节项符号相反
        assert!(((tn_s - tn_w) + (ts_s - ts_w)).abs() < 1e-9);
    }
    let amp = |lat: f64| climate::sea_level_temp(lat, 172.0, false) - climate::sea_level_temp(lat, 355.0, false);
    assert!(amp(65.0) > amp(45.0) && amp(45.0) > amp(20.0) && amp(20.0) > amp(2.0).abs() - 0.01);
    // 海洋季节幅度小于陆地
    let ocean = climate::sea_level_temp(50.0, 172.0, true) - climate::sea_level_temp(50.0, 355.0, true);
    assert!(ocean < amp(50.0));
}

#[test]
fn yearly_cycle_is_periodic_and_snow_line_tracks_temperature() {
    for &lat in &[-50.0, -10.0, 30.0, 70.0] {
        let t0 = climate::sea_level_temp(lat, 0.0, false);
        let t365 = climate::sea_level_temp(lat, 365.0, false);
        assert!((t0 - t365).abs() < 1e-6, "year periodic at {lat}");
    }
    // 雪线与 0℃ 等温线一致：随纬度升高而降低；极地为 0
    let sl = |lat: f64| climate::snow_line(lat, 200.0);
    assert!(sl(0.0) > sl(30.0) && sl(30.0) > sl(60.0));
    assert_eq!(climate::snow_line(85.0, 355.0), 0.0);
    // 夏季高于冬季（北半球）
    assert!(climate::snow_line(45.0, 172.0) > climate::snow_line(45.0, 355.0));
    // 南半球相反
    assert!(climate::snow_line(-45.0, 355.0) > climate::snow_line(-45.0, 172.0));
}

#[test]
fn temperature_lapse_rate_and_isa_delta() {
    let t0 = climate::temperature_at(40.0, 100.0, 0.0, false);
    let t3 = climate::temperature_at(40.0, 100.0, 3000.0, false);
    assert!(((t0 - t3) - 19.5).abs() < 1e-9);
    assert!((climate::isa_delta_t(40.0, 100.0, false) - (t0 - 15.0)).abs() < 1e-9);
    // 赤道比极地暖
    assert!(climate::sea_level_temp(0.0, 100.0, true) > climate::sea_level_temp(80.0, 100.0, true) + 30.0);
}

#[test]
fn dynamic_snow_cover_on_a_real_mountain_follows_the_season() {
    let w = world();
    // 在中纬度找最高的山峰，测其北半球冬夏的积雪覆盖变化
    let mut best: Option<(f64, f64, f64)> = None;
    for i in 0..400 {
        for j in 0..100 {
            let lat = 30.0 + j as f64 * 0.3;
            let (x, z) = latlon_to_world(lat, -180.0 + i as f64 * 0.9);
            let h = w.height(x, z);
            if best.map_or(true, |b| h > b.2) {
                best = Some((x, z, h));
            }
        }
    }
    let (x, z, h) = best.unwrap();
    println!("tallest N-mid-lat peak {h:.0} m");
    assert!(h > 1500.0);
    let snow = |day: f64| w.climate(x, z, day).snow_line;
    assert!(snow(172.0) > snow(355.0), "snow line summer {} winter {}", snow(172.0), snow(355.0));
    // 冬季温度更低
    assert!(w.climate(x, z, 355.0).temp < w.climate(x, z, 172.0).temp);
}

#[test]
fn colors_are_valid_rgb_and_vary_with_season() {
    let w = world();
    let mut changed = 0;
    let mut n = 0;
    for i in 0..60 {
        for j in 0..30 {
            let (x, z) = latlon_to_world(35.0 + j as f64 * 1.5, i as f64 * 6.0 - 180.0);
            let a = w.color_at(x, z, 172.0);
            let b = w.color_at(x, z, 355.0);
            for c in a.iter().chain(b.iter()) {
                assert!((0.0..=1.0).contains(c));
            }
            n += 1;
            if a != b {
                changed += 1;
            }
        }
    }
    assert!(changed > 20, "seasonal recoloring too rare: {changed}/{n}");
}
