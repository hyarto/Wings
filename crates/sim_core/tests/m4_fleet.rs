//! M4 无头试飞测试：5 种机型各自独立的起飞/失速/升限/滚转/着陆测试 + 机型间差异 + 专属系统
//! （反桨/反推/单发失效/减速板/马赫/水面浮力与水上起降）。

use sim_core::aircraft::*;
use sim_core::atmosphere::{calibrated_airspeed, Atmosphere};
use sim_core::autopilot::*;
use sim_core::gear::{FlatGround, SurfaceType};
use sim_core::integrator::Simulation;
use sim_core::{FIXED_DT, KT};

// ------------------------------------------------------------------ 公共试飞工具

fn new_sim(code: &str, water: bool) -> Simulation {
    let cfg = AircraftConfig::by_code(code).unwrap();
    let g = FlatGround { elevation: 0.0, surface: if water { SurfaceType::Water } else { SurfaceType::Asphalt } };
    let mut s = Simulation::new(cfg, Box::new(g), (0.0, 0.0), 0.0);
    if water {
        s.place_on_water();
    }
    s
}

fn takeoff_flaps(code: &str) -> f64 {
    match code {
        "TWIN-6" => 0.27,
        "JET-39" => 0.5,
        "LINER-73" => 5.0 / 40.0,
        "SEA-415" => 0.33,
        _ => 0.0,
    }
}

#[derive(Debug, Clone, Copy)]
struct Takeoff {
    liftoff_kt: f64,
    roll: f64,
    crashed: bool,
    climbing: bool,
}

fn takeoff(code: &str, water: bool) -> Takeoff {
    let mut sim = new_sim(code, water);
    takeoff_from(&mut sim, code)
}

fn takeoff_from(sim: &mut Simulation, code: &str) -> Takeoff {
    let cfg = sim.config.clone();
    sim.controls.flaps = takeoff_flaps(code);
    sim.state.surfaces.flaps = sim.controls.flaps;
    sim.controls.parking_brake = !sim.state.on_ground || true;
    for _ in 0..360 {
        sim.step();
    }
    sim.controls.parking_brake = false;
    sim.controls.throttle = 1.0;
    let mut ph = PitchHold::default();
    let z0 = sim.state.rigid.pos.z;
    let mut lift: Option<(f64, f64)> = None;
    let mut out = Takeoff { liftoff_kt: f64::NAN, roll: f64::NAN, crashed: false, climbing: false };
    for _ in 0..(120 * 400) {
        let ias = sim.diag.ias_true / KT;
        sim.controls.yaw = heading_to_rudder(sim, 0.0);
        let target = if ias >= cfg.v_rotate / KT { 9.0 } else { 0.0 };
        sim.controls.pitch = ph.control(sim, target, FIXED_DT);
        sim.step();
        if sim.state.damage.crashed {
            out.crashed = true;
            return out;
        }
        if lift.is_none() && !sim.state.on_ground && sim.state.air_time > 0.3 {
            lift = Some((sim.diag.ias_true / KT, (sim.state.rigid.pos.z - z0).abs()));
        }
        if lift.is_some() && sim.diag.agl > 30.0 + cfg.rest_cg_height {
            out.climbing = sim.diag.vertical_speed > 1.0;
            break;
        }
    }
    if let Some((v, d)) = lift {
        out.liftoff_kt = v;
        out.roll = d;
    }
    out
}

/// 平飞减速失速，返回失速 IAS (kt) 与峰值 AoA (deg)。
fn stall(code: &str, flaps: f64, gear: bool) -> (f64, f64) {
    let mut sim = new_sim(code, false);
    let cfg = sim.config.clone();
    sim.set_airborne(3000.0, cfg.v_approach * 1.5, 0.0);
    sim.controls.flaps = flaps;
    sim.state.surfaces.flaps = flaps;
    sim.set_gear_instant(gear);
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut target = 3.0;
    let mut peak_cl = 0.0;
    let mut res = (0.0, 0.0);
    for _ in 0..(120 * 400) {
        sim.controls.throttle = 0.0;
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        target += 0.3 * FIXED_DT;
        sim.controls.pitch = ph.control(&sim, target.min(30.0), FIXED_DT);
        sim.step();
        if sim.state.time < 4.0 {
            continue;
        }
        let cl = sim.diag.aero.surfaces[1].cl;
        if cl > peak_cl {
            peak_cl = cl;
            res = (sim.diag.ias_true / KT, sim.diag.aoa.to_degrees());
        }
        if cl < peak_cl - 0.3 && sim.diag.ias_true / KT < cfg.v_approach / KT * 1.2 {
            break;
        }
    }
    assert!(peak_cl > 0.0, "{code}: no stall");
    res
}

fn ias_for_mach(m: f64, alt: f64) -> f64 {
    let atm = Atmosphere::default().sample(alt);
    calibrated_airspeed(m * atm.speed_of_sound, &atm) / KT
}

/// 全油门爬升到升限。返回 (ROC<0.5 m/s 的高度, 最终高度)。
fn ceiling(code: &str) -> (f64, f64) {
    let mut sim = new_sim(code, false);
    let cfg = sim.config.clone();
    sim.state.fuel_kg = cfg.fuel_capacity * 0.7;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut sp = SpeedToPitch::default();
    sim.set_airborne(500.0, cfg.v_best_climb, 0.0);
    sim.set_gear_instant(false);
    let mach = match code {
        "LINER-73" => 0.74,
        "JET-39" => 0.5,
        _ => 0.4,
    };
    let mut smooth = 0.0;
    let mut roc05 = None;
    let mut last_alt = 0.0;
    for i in 0..(120 * 60 * 150) {
        let alt = sim.state.rigid.pos.y;
        let tgt = (cfg.v_best_climb / KT).min(ias_for_mach(mach, alt)).max(60.0);
        sim.controls.throttle = 1.0;
        let tp = sp.target_pitch(&sim, tgt * KT, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        smooth += (sim.diag.vertical_speed - smooth) * 0.002;
        if i % (120 * 30) == 0 {
            last_alt = alt;
            assert!(!sim.state.damage.any(), "{code}: damaged in climb at {alt:.0} m");
            if roc05.is_none() && alt > 1500.0 && smooth < 0.5 {
                roc05 = Some(alt);
            }
            if i > 120 * 600 && smooth < 0.15 {
                break;
            }
        }
    }
    (roc05.unwrap_or(last_alt), last_alt)
}

/// 满舵副翼稳态滚转率 deg/s。
fn roll_rate(code: &str, ias_kt: f64) -> f64 {
    let mut sim = new_sim(code, false);
    sim.set_airborne(1500.0, ias_kt * KT, 0.0);
    sim.set_gear_instant(false);
    sim.controls.throttle = 0.6;
    let mut ph = PitchHold::default();
    let mut best: f64 = 0.0;
    for i in 0..(120 * 4) {
        sim.controls.roll = 1.0;
        sim.controls.pitch = ph.control(&sim, 2.0, FIXED_DT);
        sim.step();
        if i > 120 {
            best = best.max((-sim.state.rigid.w.z).to_degrees());
        }
    }
    best
}

#[derive(Debug, Clone, Copy)]
struct Landing {
    touchdown_kt: f64,
    sink: f64,
    stop_dist: f64,
    crashed: bool,
    damaged: bool,
}

/// 3° 下滑道进近 → 拉平 → 接地 → 刹车/反推/关车停稳。`gear_down=None` 表示按水/陆默认。
fn land_on(sim: &mut Simulation, flaps: f64, reverse: bool, water: bool, gear_down: Option<bool>) -> Landing {
    let cfg = sim.config.clone();
    let vref = cfg.v_approach / KT;
    let start_z = sim.state.rigid.pos.z + 6000.0;
    let x0 = sim.state.rigid.pos.x;
    let alt = (6000.0 + 300.0) * 3.0f64.to_radians().tan() + 8.0;
    sim.state.rigid.pos.z = start_z;
    sim.set_airborne(alt, vref * KT, 0.0);
    for e in sim.state.engines.iter_mut() {
        e.running = true;
    }
    sim.controls.flaps = flaps;
    sim.state.surfaces.flaps = flaps;
    sim.set_gear_instant(gear_down.unwrap_or(!water));
    let (mut ph, mut bh) = (PitchHold::default(), BankHold::default());
    let mut tp = Pid::new(0.8, 0.05, 0.0, 6.0);
    let mut thr = Pid::new(0.04, 0.01, 0.0, 0.5);
    let mut td: Option<(f64, f64, f64)> = None;
    let mut stop_z = f64::NAN;
    let mut crashed = false;
    for _ in 0..(120 * 600) {
        let agl = sim.diag.agl - cfg.rest_cg_height;
        if !sim.state.on_ground {
            let tgt = if agl < 15.0 { vref - 4.0 } else { vref };
            let hf = (sim.diag.ground_speed * 0.16).max(7.0);
            let vs_t = if agl < hf { -0.5 - 1.5 * (agl / hf) } else { -sim.diag.ground_speed * 3.0f64.to_radians().tan() };
            let pitch_t = 3.0 + tp.update(vs_t - sim.diag.vertical_speed, FIXED_DT);
            sim.controls.pitch = ph.control(sim, pitch_t.clamp(-6.0, 10.0), FIXED_DT);
            sim.controls.throttle = if agl < hf * 0.6 { 0.0 } else { (0.35 + thr.update(tgt - sim.diag.ias_true / KT, FIXED_DT)).clamp(0.0, 1.0) };
            sim.controls.roll = bh.control(sim, 0.0, FIXED_DT);
            sim.controls.yaw = 0.0;
        } else {
            if td.is_none() {
                td = Some((sim.state.rigid.pos.z, sim.state.last_touchdown_sink, sim.diag.ias_true / KT));
            }
            sim.controls.throttle = if reverse { 0.9 } else { 0.0 };
            sim.controls.reverse = reverse;
            sim.controls.speedbrake = 1.0;
            sim.controls.pitch = 0.0;
            sim.controls.roll = 0.0;
            sim.controls.yaw = heading_to_rudder(sim, 0.0);
            if !water {
                sim.controls.brake_left = 1.0;
                sim.controls.brake_right = 1.0;
            }
            if sim.diag.ground_speed < 15.0 {
                sim.controls.reverse = false;
                sim.controls.throttle = 0.0;
            }
            if water && sim.diag.ground_speed < 9.0 {
                for e in sim.state.engines.iter_mut() {
                    e.running = false;
                }
            }
            if sim.diag.ground_speed < if water { 2.0 } else { 0.5 } {
                stop_z = sim.state.rigid.pos.z;
                break;
            }
        }
        sim.step();
        if sim.state.damage.crashed {
            crashed = true;
            break;
        }
    }
    sim.controls = Default::default();
    sim.controls.gear_down = sim.state.gear_pos > 0.5;
    let _ = x0;
    let (z, sink, ias) = td.unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    Landing { touchdown_kt: ias, sink, stop_dist: (z - stop_z).abs(), crashed, damaged: sim.state.damage.any() }
}

fn land(code: &str, reverse: bool) -> Landing {
    let mut sim = new_sim(code, false);
    let fl = *sim.config.flap_detents.last().unwrap() * 0.8;
    land_on(&mut sim, fl, reverse, false, None)
}

/// 定高定速平飞（全油门），返回 (TAS kt, 马赫)。
fn cruise(code: &str, alt: f64) -> (f64, f64) {
    let mut sim = new_sim(code, false);
    sim.set_airborne(alt, sim.config.v_best_climb * 0.8, 0.0);
    sim.set_gear_instant(false);
    let (mut ph, mut bh) = (PitchHold::default(), BankHold::default());
    let mut pid = Pid::new(0.5, 0.02, 0.0, 10.0);
    for _ in 0..(120 * 600) {
        sim.controls.throttle = 1.0;
        let tp = 2.0 + pid.update(-(sim.state.rigid.pos.y - alt) * 0.05 - sim.diag.vertical_speed * 0.5, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp.clamp(-10.0, 12.0), FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
    }
    assert!(!sim.state.damage.any(), "{code} damaged in cruise");
    (sim.diag.tas / KT, sim.diag.mach)
}

// ------------------------------------------------------------------ 配置完整性

#[test]
fn five_aircraft_are_registered_with_full_surface_sets() {
    let all = builtin();
    assert_eq!(all.len(), 5);
    for (cfg, code) in all.iter().zip(CODES) {
        assert_eq!(cfg.code, code);
        assert!(AircraftConfig::by_code(&code.to_lowercase()).is_some());
        // 至少 7 个气动面，且包含规定的种类
        assert!(cfg.surfaces.len() >= 7, "{code}");
        for k in [
            SurfaceKind::WingLeftOuter,
            SurfaceKind::WingLeftInner,
            SurfaceKind::WingRightInner,
            SurfaceKind::WingRightOuter,
            SurfaceKind::HorizontalStab,
            SurfaceKind::VerticalStab,
            SurfaceKind::Fuselage,
        ] {
            assert!(cfg.surfaces.iter().any(|s| s.kind == k), "{code} missing {k:?}");
        }
        // 机翼面积与分段面积一致
        let a: f64 = cfg.surfaces.iter().filter(|s| s.kind.is_wing()).map(|s| s.area).sum();
        assert!((a - cfg.wing_area).abs() < 0.01 * cfg.wing_area, "{code}: wing area {a} vs {}", cfg.wing_area);
        assert!(!cfg.engines.is_empty() && cfg.contacts.len() >= 6);
        assert!(!cfg.blurb().is_empty());
        assert!(cfg.flap_detents.first() == Some(&0.0) && cfg.flap_detents.last() == Some(&1.0));
        assert!(cfg.structure.limit_load_pos > 2.0 && cfg.structure.v_ne > cfg.structure.v_fe);
    }
    let codes: std::collections::HashSet<_> = all.iter().map(|a| a.code).collect();
    assert_eq!(codes.len(), 5);
    // 机型之间的关键差异
    let by = |c: &str| AircraftConfig::by_code(c).unwrap();
    assert_eq!(by("TWIN-6").engines.len(), 2);
    assert_eq!(by("JET-39").engines.len(), 1);
    assert!(matches!(by("JET-39").engines[0].kind, EngineKind::Turbofan { .. }));
    assert!(matches!(by("TWIN-6").engines[0].kind, EngineKind::Turboprop { .. }));
    assert!(matches!(by("LINER-73").engines[0].kind, EngineKind::Turbofan { reverse_ratio, .. } if reverse_ratio > 0.2));
    assert!(by("SEA-415").amphibious && !by("LINER-73").amphibious);
    assert!(by("SEA-415").contacts.iter().any(|c| c.kind == ContactKind::Float));
    assert!(by("JET-39").structure.limit_load_pos >= 8.0 && by("JET-39").structure.limit_load_neg <= -4.0);
    assert!(by("LINER-73").mass_empty > 20.0 * by("SKY-172").mass_empty);
}

#[test]
fn every_land_aircraft_rests_stably_on_its_gear() {
    for code in ["TWIN-6", "JET-39", "LINER-73", "SEA-415"] {
        let mut sim = new_sim(code, false);
        let y0 = sim.state.rigid.pos.y;
        sim.controls.parking_brake = true;
        for _ in 0..(120 * 20) {
            sim.step();
        }
        let wheels = sim.diag.gear.contacts.iter().filter(|c| c.in_contact && !c.is_hull).count();
        assert!(wheels >= 3, "{code}: only {wheels} wheels in contact");
        assert!(!sim.state.damage.any(), "{code} damaged at rest");
        assert!(sim.diag.ground_speed < 0.3, "{code} creeping {}", sim.diag.ground_speed);
        assert!((sim.state.rigid.pos.y - y0).abs() < 0.25, "{code}: settled {} vs {}", sim.state.rigid.pos.y, y0);
        assert!(sim.diag.pitch.abs() < 2.5 && sim.diag.roll.abs() < 1.0, "{code} pitch {} roll {}", sim.diag.pitch, sim.diag.roll);
    }
}

// ------------------------------------------------------------------ TWIN-6

#[test]
fn twin6_stol_takeoff_stall_and_landing() {
    let t = takeoff("TWIN-6", false);
    println!("TWIN-6 {t:?}");
    assert!(!t.crashed && t.climbing);
    assert!((55.0..=72.0).contains(&t.liftoff_kt), "{t:?}");
    assert!((100.0..=350.0).contains(&t.roll), "STOL roll {t:?}");
    let (clean, a0) = stall("TWIN-6", 0.0, false);
    let (full, a1) = stall("TWIN-6", 1.0, true);
    println!("TWIN-6 stall clean {clean:.1} kt ({a0:.1}°) full flap {full:.1} kt ({a1:.1}°)");
    assert!((62.0..=78.0).contains(&clean) && (13.0..=19.0).contains(&a0));
    assert!((46.0..=62.0).contains(&full), "STOL flaps stall {full}");
    assert!(full < clean * 0.85);
    let l = land("TWIN-6", false);
    println!("TWIN-6 {l:?}");
    assert!(!l.crashed && !l.damaged && l.sink < 3.5);
    assert!(l.stop_dist < 350.0, "{l:?}");
}

#[test]
fn twin6_ceiling_and_cruise() {
    let (c, last) = ceiling("TWIN-6");
    println!("TWIN-6 ceiling {c:.0} (final {last:.0})");
    assert!((6300.0..=8500.0).contains(&c), "ceiling {c}");
    let (tas, _) = cruise("TWIN-6", 3000.0);
    println!("TWIN-6 cruise {tas:.0} kt");
    assert!((140.0..=185.0).contains(&tas), "cruise {tas}");
    let r = roll_rate("TWIN-6", 112.0);
    assert!((12.0..=70.0).contains(&r), "roll rate {r}");
}

#[test]
fn twin6_reverse_pitch_shortens_landing() {
    let a = land("TWIN-6", false);
    let b = land("TWIN-6", true);
    println!("no reverse {:.0} m, reverse {:.0} m", a.stop_dist, b.stop_dist);
    assert!(!b.crashed && b.stop_dist < a.stop_dist * 0.92, "{a:?} {b:?}");
}

/// 单发失效：不对称推力使机头偏向失效侧；顺桨显著降低阻力；方向舵可保持航向并继续飞行。
#[test]
fn twin6_engine_out_asymmetric_thrust_and_feathering() {
    let fly = |feather: bool, rudder: bool| {
        let mut sim = new_sim("TWIN-6", false);
        sim.set_airborne(1500.0, 90.0 * KT, 0.0);
        sim.controls.throttle = 1.0;
        let mut ph = PitchHold::default();
        let mut bh = BankHold::default();
        let mut sp = SpeedToPitch::default();
        for _ in 0..(120 * 5) {
            let tp = sp.target_pitch(&sim, 90.0 * KT, FIXED_DT);
            sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
            sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
            sim.step();
        }
        sim.fail_engine(0); // 左发
        sim.controls.feather = feather;
        let h0 = sim.diag.heading;
        let mut yaw_after_3s = 0.0;
        for i in 0..(120 * 60) {
            let tp = sp.target_pitch(&sim, 90.0 * KT, FIXED_DT);
            sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
            if rudder {
                sim.controls.yaw = heading_to_rudder(&sim, 0.0);
                let bank = heading_to_bank(&sim, 0.0, 5.0);
                sim.controls.roll = bh.control(&sim, bank, FIXED_DT);
            } else {
                sim.controls.roll = 0.0;
            }
            sim.step();
            if i == 120 * 3 {
                let mut d = sim.diag.heading - h0;
                if d > 180.0 { d -= 360.0 }
                if d < -180.0 { d += 360.0 }
                yaw_after_3s = d;
            }
        }
        (yaw_after_3s, sim.diag.heading, sim.diag.vertical_speed, sim.diag.ias_true / KT, sim.state.damage.any())
    };
    // 无舵：左发失效 → 机头向左（航向减小）
    let (yaw3, ..) = fly(false, false);
    println!("uncorrected yaw after 3 s: {yaw3:.1}°");
    assert!(yaw3 < -2.0, "left engine out should yaw left, got {yaw3}");
    // 有舵、未顺桨 vs 顺桨
    let (_, h_a, vs_a, ias_a, dmg_a) = fly(false, true);
    let (_, h_b, vs_b, ias_b, dmg_b) = fly(true, true);
    println!("windmilling: hdg {h_a:.0} vs {vs_a:.2} ias {ias_a:.0};  feathered: hdg {h_b:.0} vs {vs_b:.2} ias {ias_b:.0}");
    let off = |h: f64| ((h + 180.0) % 360.0 - 180.0).abs();
    assert!(off(h_a) < 20.0 && off(h_b) < 20.0, "rudder should hold heading: {h_a} {h_b}");
    assert!(!dmg_a && !dmg_b);
    assert!(vs_b > vs_a + 0.8, "feathering must reduce drag: windmill {vs_a} feathered {vs_b}");
    assert!(ias_b > 70.0, "feathered single-engine flight must be sustainable ({ias_b} kt)");
}

// ------------------------------------------------------------------ JET-39

#[test]
fn jet39_takeoff_stall_roll_and_landing() {
    let t = takeoff("JET-39", false);
    println!("JET-39 {t:?}");
    assert!(!t.crashed && t.climbing);
    assert!((92.0..=118.0).contains(&t.liftoff_kt) && (350.0..=800.0).contains(&t.roll), "{t:?}");
    let (clean, a0) = stall("JET-39", 0.0, false);
    println!("JET-39 stall {clean:.1} kt ({a0:.1}°)");
    assert!((92.0..=115.0).contains(&clean) && (13.0..=19.0).contains(&a0));
    let (full, _) = stall("JET-39", 1.0, true);
    assert!(full < clean * 0.88, "{full} vs {clean}");
    let fast = roll_rate("JET-39", 270.0);
    println!("JET-39 roll {fast:.0}°/s");
    assert!(fast > 90.0, "aerobatic roll rate {fast}");
    let l = land("JET-39", false);
    println!("JET-39 {l:?}");
    assert!(!l.crashed && !l.damaged && l.stop_dist < 900.0, "{l:?}");
}

#[test]
fn jet39_ceiling_and_cruise() {
    let (c, last) = ceiling("JET-39");
    println!("JET-39 ceiling {c:.0} (final {last:.0})");
    assert!((9800.0..=12600.0).contains(&c), "ceiling {c}");
    let (tas, mach) = cruise("JET-39", 3000.0);
    println!("JET-39 cruise {tas:.0} kt M{mach:.2}");
    assert!((330.0..=420.0).contains(&tas), "cruise {tas}");
}

#[test]
fn jet39_pulls_high_g_within_limits_and_breaks_beyond_vne() {
    // +8 G 级拉升：应能达到 >6.5 G，且不超极限载荷（12 G）而不损坏
    let mut sim = new_sim("JET-39", false);
    sim.set_airborne(3000.0, 300.0 * KT, 0.0);
    sim.set_gear_instant(false);
    sim.controls.throttle = 1.0;
    let (mut max_nz, mut min_nz): (f64, f64) = (0.0, 0.0);
    for _ in 0..(120 * 3) {
        sim.controls.pitch = 1.0;
        sim.step();
        max_nz = max_nz.max(sim.diag.nz);
    }
    println!("JET-39 max nz {max_nz:.2}  damage {:?}", sim.state.damage);
    assert!(max_nz > 6.5, "max G {max_nz}");
    assert!(!sim.state.damage.left_wing_failed && !sim.state.damage.right_wing_failed);
    // -4 G 推杆：能产生明显负过载
    let mut sim = new_sim("JET-39", false);
    sim.set_airborne(3000.0, 300.0 * KT, 0.0);
    sim.set_gear_instant(false);
    sim.controls.throttle = 1.0;
    for _ in 0..(120 * 3) {
        sim.controls.pitch = -1.0;
        sim.step();
        min_nz = min_nz.min(sim.diag.nz);
        if sim.state.damage.any() {
            break;
        }
    }
    println!("JET-39 min nz {min_nz:.2}");
    assert!(min_nz < -3.0, "negative G {min_nz}");
    // 超速俯冲：V_NE 437 kt 之后断翼
    let mut sim = new_sim("JET-39", false);
    sim.set_airborne(9000.0, 250.0 * KT, 0.0);
    sim.set_gear_instant(false);
    sim.controls.throttle = 1.0;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut warned = false;
    let mut broke = None;
    for _ in 0..(120 * 120) {
        sim.controls.pitch = ph.control(&sim, -50.0, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        warned |= sim.diag.warnings.overspeed;
        if sim.state.damage.left_wing_failed || sim.state.damage.right_wing_failed {
            broke = Some(sim.diag.ias_true / KT);
            break;
        }
    }
    println!("JET-39 wing failed at {broke:?}");
    assert!(warned);
    assert!(broke.expect("should break") > 437.0);
}

#[test]
fn jet39_gear_retracts_and_speedbrake_slows() {
    let mut sim = new_sim("JET-39", false);
    sim.set_airborne(2000.0, 200.0 * KT, 0.0);
    assert!(sim.state.gear_pos >= 0.99);
    sim.controls.gear_down = false;
    sim.controls.throttle = 0.5;
    for _ in 0..(120 * 8) {
        sim.step();
    }
    assert!(sim.state.gear_pos < 0.01, "gear {}", sim.state.gear_pos);
    // 减速板：相同初始条件下，开板 15 s 后速度更低
    let run = |sb: f64| {
        let mut s = new_sim("JET-39", false);
        s.set_airborne(3000.0, 250.0 * KT, 0.0);
        s.set_gear_instant(false);
        let mut ph = PitchHold::default();
        for _ in 0..(120 * 15) {
            s.controls.throttle = 0.0;
            s.controls.speedbrake = sb;
            s.controls.pitch = ph.control(&s, 0.0, FIXED_DT);
            s.step();
        }
        s.diag.ias_true / KT
    };
    let (clean, braked) = (run(0.0), run(1.0));
    println!("speedbrake: clean {clean:.0} kt vs braked {braked:.0} kt");
    assert!(braked < clean - 8.0);
}

// ------------------------------------------------------------------ LINER-73

#[test]
fn liner73_takeoff_stall_and_inertia() {
    let t = takeoff("LINER-73", false);
    println!("LINER-73 {t:?}");
    assert!(!t.crashed && t.climbing);
    assert!((130.0..=165.0).contains(&t.liftoff_kt) && (800.0..=1800.0).contains(&t.roll), "{t:?}");
    let (clean, a0) = stall("LINER-73", 0.0, false);
    let (full, a1) = stall("LINER-73", 1.0, true);
    println!("LINER-73 stall clean {clean:.1} kt ({a0:.1}°) landing {full:.1} kt ({a1:.1}°)");
    assert!((128.0..=152.0).contains(&clean) && (13.0..=19.0).contains(&a0));
    assert!((98.0..=125.0).contains(&full) && full < clean * 0.88, "{full}");
    // 大惯性：滚转缓慢
    let r = roll_rate("LINER-73", 224.0);
    println!("LINER-73 roll {r:.0}°/s");
    assert!((8.0..=45.0).contains(&r), "roll {r}");
}

#[test]
fn liner73_ceiling_cruise_and_mach_limit() {
    let (c, last) = ceiling("LINER-73");
    println!("LINER-73 ceiling {c:.0} (final {last:.0})");
    assert!((11_200.0..=13_300.0).contains(&c), "ceiling {c}");
    let (tas, mach) = cruise("LINER-73", 10_500.0);
    println!("LINER-73 cruise {tas:.0} kt M{mach:.3}");
    assert!((420.0..=480.0).contains(&tas), "cruise TAS {tas}");
    assert!(mach > 0.72 && mach < 0.82, "Mach {mach}: wave drag should cap the cruise");
}

#[test]
fn liner73_reverse_thrust_and_spoilers_shorten_the_landing() {
    let a = land("LINER-73", false);
    let b = land("LINER-73", true);
    println!("LINER no reverse {a:?}\nLINER reverse {b:?}");
    assert!(!a.crashed && !a.damaged && !b.crashed && !b.damaged);
    assert!(a.sink < 3.2 && b.sink < 3.2);
    assert!(b.stop_dist < a.stop_dist * 0.92, "reverse thrust must help: {} vs {}", b.stop_dist, a.stop_dist);
    assert!(a.stop_dist > 400.0 && a.stop_dist < 2200.0);
    // 扰流板：增阻减升
    let run = |sb: f64| {
        let mut s = new_sim("LINER-73", false);
        s.set_airborne(6000.0, 280.0 * KT, 0.0);
        s.set_gear_instant(false);
        let mut ph = PitchHold::default();
        for _ in 0..(120 * 20) {
            s.controls.throttle = 0.0;
            s.controls.speedbrake = sb;
            s.controls.pitch = ph.control(&s, 0.0, FIXED_DT);
            s.step();
        }
        (s.diag.ias_true / KT, s.diag.vertical_speed)
    };
    let (clean, braked) = (run(0.0), run(1.0));
    println!("spoilers: {clean:?} vs {braked:?}");
    assert!(braked.0 < clean.0 - 8.0 && braked.1 < clean.1);
}

#[test]
fn liner73_flap_schedule_is_multistage_and_slow() {
    let cfg = AircraftConfig::by_code("LINER-73").unwrap();
    assert!(cfg.flap_detents.len() >= 7);
    let mut sim = new_sim("LINER-73", false);
    sim.set_airborne(3000.0, 180.0 * KT, 0.0);
    sim.controls.flaps = 1.0;
    for _ in 0..(120 * 10) {
        sim.step();
    }
    assert!(sim.state.surfaces.flaps > 0.15 && sim.state.surfaces.flaps < 0.35, "flaps after 10 s: {}", sim.state.surfaces.flaps);
    // 超 V_FE 放襟翼 → 告警
    let mut sim = new_sim("LINER-73", false);
    sim.set_airborne(3000.0, 260.0 * KT, 0.0);
    sim.controls.flaps = 0.5;
    sim.state.surfaces.flaps = 0.5;
    sim.step();
    assert!(sim.diag.warnings.flap_overspeed);
}

// ------------------------------------------------------------------ SEA-415

#[test]
fn sea415_land_runway_ceiling_and_stall() {
    let t = takeoff("SEA-415", false);
    println!("SEA-415 land takeoff {t:?}");
    assert!(!t.crashed && t.climbing);
    assert!((72.0..=98.0).contains(&t.liftoff_kt) && (250.0..=700.0).contains(&t.roll), "{t:?}");
    let (clean, a0) = stall("SEA-415", 0.0, false);
    let (full, _) = stall("SEA-415", 1.0, true);
    println!("SEA-415 stall clean {clean:.1} full flap {full:.1} ({a0:.1}°)");
    assert!((72.0..=90.0).contains(&clean) && (13.0..=19.0).contains(&a0) && full < clean * 0.85);
    let (c, last) = ceiling("SEA-415");
    println!("SEA-415 ceiling {c:.0} (final {last:.0})");
    assert!((5300.0..=7200.0).contains(&c), "ceiling {c}");
    let l = land("SEA-415", false);
    assert!(!l.crashed && !l.damaged && l.stop_dist < 700.0, "{l:?}");
}

#[test]
fn sea415_floats_in_calm_water_and_buoyancy_scales_with_load() {
    let mut sim = new_sim("SEA-415", true);
    // 怠速推力会让水上飞机缓慢滑行：此处关车（系泊）观察静态浮力
    sim.state.engines.iter_mut().for_each(|e| e.running = false);
    let y0 = sim.state.rigid.pos.y;
    for _ in 0..(120 * 30) {
        sim.step();
    }
    assert!(!sim.state.damage.any());
    assert!(sim.state.on_ground && sim.diag.gear.float_contact);
    assert!((sim.state.rigid.pos.y - y0).abs() < 0.25, "bob {} vs {}", sim.state.rigid.pos.y, y0);
    assert!(sim.diag.ground_speed < 0.3 && sim.diag.pitch.abs() < 2.0 && sim.diag.roll.abs() < 2.0, "gs {} pitch {} roll {} y {}", sim.diag.ground_speed, sim.diag.pitch, sim.diag.roll, sim.state.rigid.pos.y);
    // 浮力 = 重量：总法向力 ≈ mg
    let w = sim.total_mass() * sim_core::G;
    assert!((sim.diag.gear.total_normal / w - 1.0).abs() < 0.15, "buoyancy {} vs {}", sim.diag.gear.total_normal, w);
    // 载重更大 → 吃水更深（重心更低）
    let mut heavy = new_sim("SEA-415", true);
    heavy.config.payload += 2500.0;
    heavy.place_on_water();
    heavy.state.engines.iter_mut().for_each(|e| e.running = false);
    for _ in 0..(120 * 30) {
        heavy.step();
    }
    println!("light y {:.2}, heavy y {:.2}", sim.state.rigid.pos.y, heavy.state.rigid.pos.y);
    assert!(heavy.state.rigid.pos.y < sim.state.rigid.pos.y - 0.04);
    assert!(!heavy.state.damage.any());
    // 水面平静时漂移很小；水动阻尼使风/侧移很快衰减
    sim.state.rigid.vel.x = 5.0;
    for _ in 0..(120 * 40) {
        sim.step();
    }
    assert!(sim.state.rigid.vel.x.abs() < 0.5, "sideways motion not damped: {}", sim.state.rigid.vel.x);
}

#[test]
fn sea415_water_takeoff() {
    let t = takeoff("SEA-415", true);
    println!("SEA-415 water takeoff {t:?}");
    assert!(!t.crashed && t.climbing, "{t:?}");
    assert!((80.0..=115.0).contains(&t.liftoff_kt), "{t:?}");
    assert!((500.0..=1500.0).contains(&t.roll), "water run {t:?}");
    // 水上起飞滑跑明显长于陆地跑道
    let land_t = takeoff("SEA-415", false);
    assert!(t.roll > land_t.roll * 1.3, "water {} land {}", t.roll, land_t.roll);
}

/// DoD：SEA-415 可成功在海面降落并重新起飞。
#[test]
fn sea415_lands_on_the_sea_and_takes_off_again() {
    let mut sim = new_sim("SEA-415", true);
    let l = land_on(&mut sim, 0.66, true, true, None);
    println!("SEA water landing {l:?}");
    assert!(!l.crashed && !l.damaged, "{l:?}");
    assert!(l.sink < 4.0 && l.stop_dist.is_finite(), "{l:?}");
    // 停稳后漂浮
    for _ in 0..(120 * 10) {
        sim.step();
    }
    assert!(sim.state.on_ground && sim.diag.gear.float_contact && sim.diag.ground_speed < 3.0);
    assert!(sim.state.gear_pos < 0.05, "gear should be up for water ops");
    // 重新起飞（重启发动机）
    for e in sim.state.engines.iter_mut() {
        e.running = true;
    }
    sim.state.rigid.vel = glam::DVec3::ZERO;
    let heading = sim.diag.heading;
    let _ = heading;
    let t = takeoff_from(&mut sim, "SEA-415");
    println!("SEA water re-takeoff {t:?}");
    assert!(!t.crashed && t.climbing && t.liftoff_kt.is_finite(), "{t:?}");
    assert!(!sim.state.damage.any());
}

#[test]
fn gear_down_water_landing_flips_but_floats_do_not() {
    let mut sim = new_sim("SEA-415", true);
    let l = land_on(&mut sim, 0.66, false, true, Some(true));
    println!("gear-down ditching {l:?}");
    assert!(l.crashed, "landing on water with wheels extended should flip the aircraft");
    // 机轮收起的同一进近则成功
    let mut sim = new_sim("SEA-415", true);
    let ok = land_on(&mut sim, 0.66, true, true, Some(false));
    assert!(!ok.crashed);
    // 非两栖机（LINER）在海面放轮着水同样翻覆
    let mut sim = new_sim("LINER-73", true);
    sim.state.damage = Default::default();
    let fl = 0.5;
    let r = land_on(&mut sim, fl, false, true, Some(true));
    assert!(r.crashed, "airliner ditching at 140 kt must be fatal: {r:?}");
}

// ------------------------------------------------------------------ 机型差异

#[test]
fn aircraft_are_significantly_different() {
    let codes = ["SEA-415", "TWIN-6", "JET-39", "LINER-73"];
    let rolls: Vec<f64> = codes.iter().map(|c| takeoff(c, *c == "SEA-415" && false).roll).collect();
    let (sea, twin, jet, liner) = (rolls[0], rolls[1], rolls[2], rolls[3]);
    println!("takeoff rolls: sea {sea:.0} twin {twin:.0} jet {jet:.0} liner {liner:.0}");
    assert!(twin < sea && sea < jet + 150.0 && jet < liner, "roll order");
    assert!(liner > 1.8 * jet && sea > 1.5 * twin);

    let stalls: Vec<f64> = ["TWIN-6", "SEA-415", "JET-39", "LINER-73"].iter().map(|c| stall(c, 0.0, false).0).collect();
    println!("clean stall: {stalls:?}");
    assert!(stalls.windows(2).all(|w| w[1] > w[0] + 5.0), "stall order {stalls:?}");

    let ceil: Vec<f64> = ["SEA-415", "TWIN-6", "JET-39", "LINER-73"].iter().map(|c| ceiling(c).0).collect();
    println!("ceilings: {ceil:?}");
    assert!(ceil.windows(2).all(|w| w[1] > w[0] + 400.0), "ceiling order {ceil:?}");
    // 与规格值相对误差 ≤ 20%
    for (c, spec) in ceil.iter().zip([6000.0, 7500.0, 11_000.0, 12_500.0]) {
        assert!((c - spec).abs() / spec < 0.20, "ceiling {c} vs spec {spec}");
    }

    let cruises = [cruise("TWIN-6", 3000.0).0, cruise("SEA-415", 3000.0).0, cruise("JET-39", 3000.0).0, cruise("LINER-73", 10_500.0).0];
    println!("cruise TAS: {cruises:?}");
    assert!(cruises.windows(2).all(|w| w[1] > w[0] + 15.0), "cruise order {cruises:?}");

    let r_jet = roll_rate("JET-39", 200.0);
    let r_twin = roll_rate("TWIN-6", 120.0);
    let r_liner = roll_rate("LINER-73", 220.0);
    println!("roll rates jet {r_jet:.0} twin {r_twin:.0} liner {r_liner:.0}");
    assert!(r_jet > 2.0 * r_twin && r_twin > r_liner * 0.8 && r_jet > 3.0 * r_liner);
}

#[test]
fn mach_model_wave_drag_and_buffet() {
    use sim_core::aero::{mach_stall_shift, wave_drag};
    assert_eq!(wave_drag(0.5, 0.74), 0.0);
    assert!(wave_drag(0.80, 0.74) > wave_drag(0.76, 0.74));
    assert!(wave_drag(0.85, 0.74) > 0.02);
    assert_eq!(mach_stall_shift(0.7, 0.74), 0.0);
    assert!(mach_stall_shift(0.84, 0.74) > mach_stall_shift(0.79, 0.74));
}

/// 高空“棺材角”：同一重量下，高度越高，低速失速（1G）与高速马赫抖振之间的速度窗口越窄。
#[test]
fn liner73_coffin_corner_window_narrows_with_altitude() {
    use sim_core::aero::{effective_mach_crit, mach_stall_shift};
    let cfg = liner73();
    let mass = cfg.mass_empty + cfg.payload + 6000.0;
    let w = mass * sim_core::G;
    let clmax = 1.55;
    let window = |alt: f64| {
        let atm = Atmosphere::default().sample(alt);
        let v_stall = (2.0 * w / (atm.density * cfg.wing_area * clmax)).sqrt();
        let m_stall = v_stall / atm.speed_of_sound;
        // 1G 平飞：CL = W / (q S)；抖振起始 = 临界迎角提前 2°
        let mut m = 0.5;
        loop {
            let cl = w / (0.5 * atm.density * (m * atm.speed_of_sound).powi(2) * cfg.wing_area);
            let mc = effective_mach_crit(cfg.mach_crit, cl / 0.9 * 0.9 * 1.0 + 0.0);
            if mach_stall_shift(m, mc) >= 2f64.to_radians() || m > 1.2 {
                break;
            }
            m += 0.002;
        }
        m - m_stall
    };
    let (lo, mid, hi) = (window(6000.0), window(10_000.0), window(12_500.0));
    println!("margin Mach: 6 km {lo:.3}, 10 km {mid:.3}, 12.5 km {hi:.3}");
    assert!(lo > mid + 0.03 && mid > hi + 0.03 && hi < 0.25, "{lo} {mid} {hi}");
}
