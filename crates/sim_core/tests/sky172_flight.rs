//! SKY-172 无头自动试飞测试套件（M1 + M3 物理验收）。

use sim_core::aircraft::sky172;
use sim_core::autopilot::*;
use sim_core::atmosphere::Atmosphere;
use sim_core::gear::FlatGround;
use sim_core::integrator::Simulation;
use sim_core::{FIXED_DT, KT};

fn new_sim() -> Simulation {
    Simulation::new(sky172(), Box::new(FlatGround::default()), (0.0, 0.0), 0.0)
}

fn settle(sim: &mut Simulation) {
    sim.controls.parking_brake = true;
    for _ in 0..240 {
        sim.step();
    }
    sim.controls.parking_brake = false;
}

/// 满油门起飞滑跑；返回 (离地 IAS kt, 滑跑距离 m)。
fn takeoff_run(sim: &mut Simulation) -> (f64, f64) {
    settle(sim);
    sim.controls.throttle = 1.0;
    let mut ph = PitchHold::default();
    let z0 = sim.state.rigid.pos.z;
    for _ in 0..(120 * 60) {
        let ias = sim.diag.ias_true / KT;
        sim.controls.yaw = heading_to_rudder(sim, 0.0);
        sim.controls.pitch = if ias >= 55.0 { ph.control(sim, 8.0, FIXED_DT) } else { 0.0 };
        sim.step();
        if !sim.diag.gear.wheel_contact && sim.state.air_time > 0.2 {
            return (sim.diag.ias_true / KT, (sim.state.rigid.pos.z - z0).abs());
        }
    }
    panic!("never lifted off");
}

#[test]
fn takeoff_rolls_and_lifts_off_at_55_to_65_kt() {
    let mut sim = new_sim();
    let (ias, roll) = takeoff_run(&mut sim);
    println!("liftoff at {ias:.1} kt after {roll:.0} m");
    assert!((55.0..=65.0).contains(&ias), "liftoff IAS {ias}");
    assert!(roll > 150.0 && roll < 600.0, "roll {roll}");
    // 离地后稳定爬升，无结构损伤
    let mut ph = PitchHold::default();
    let mut sp = SpeedToPitch::default();
    for _ in 0..(120 * 20) {
        sim.controls.yaw = 0.0;
        let tp = sp.target_pitch(&sim, 74.0 * KT, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
        sim.step();
    }
    assert!(sim.diag.agl > 30.0, "agl {}", sim.diag.agl);
    assert!(!sim.state.damage.any());
}

#[test]
fn ground_effect_and_runway_heading_hold() {
    let mut sim = new_sim();
    takeoff_run(&mut sim);
    // 方向舵保持航向：横向偏移应很小
    assert!(sim.state.rigid.pos.x.abs() < 15.0, "drift {}", sim.state.rigid.pos.x);
}

/// 收油门、缓慢增加拉杆直到失速；返回 (峰值升力时 AoA°, 失速表速 kt, 失速后 6 s 掉高 m, 最大 CL 面板)。
fn stall_experiment(ice: f64) -> (f64, f64, f64) {
    let mut sim = new_sim();
    // 冷空气：无云不积冰也不融冰，便于固定初始结冰量
    sim.weather.atmosphere = Atmosphere::new(-18.0, 101_325.0);
    sim.set_airborne(1800.0, 75.0 * KT, 0.0);
    sim.state.ice_load = ice;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut target = 4.0;
    let mut peak_cl = 0.0;
    let mut peak = (0.0, 0.0, 0.0);
    let mut t_after: Option<f64> = None;
    let mut alt_at_peak = 0.0;
    for i in 0..(120 * 240) {
        sim.controls.throttle = 0.0;
        sim.controls.yaw = 0.0;
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        let stalled = t_after.is_some();
        if !stalled {
            target += 0.35 * FIXED_DT; // 约 0.35°/s 缓慢增加俯仰
            sim.controls.pitch = ph.control(&sim, target.min(25.0), FIXED_DT);
        } else {
            sim.controls.pitch = 1.0; // 持续拉杆
        }
        sim.step();
        let cl = sim.diag.aero.surfaces[1].cl;
        if cl > peak_cl {
            peak_cl = cl;
            peak = (sim.diag.aoa.to_degrees(), sim.diag.ias_true / KT, cl);
            alt_at_peak = sim.state.rigid.pos.y;
        }
        // 峰值之后 CL 明显下降 → 判定失速发生
        if t_after.is_none() && cl < peak_cl - 0.25 {
            t_after = Some(sim.state.time);
        }
        if let Some(t0) = t_after {
            if sim.state.time - t0 > 6.0 {
                let loss = alt_at_peak - sim.state.rigid.pos.y;
                return (peak.0, peak.1, loss);
            }
        }
        let _ = i;
    }
    panic!("no stall happened");
}

#[test]
fn power_off_stall_at_critical_aoa_with_height_loss() {
    let (aoa, ias, loss) = stall_experiment(0.0);
    println!("stall: aoa {aoa:.1} deg, ias {ias:.1} kt, height loss {loss:.0} m");
    assert!((13.0..=20.0).contains(&aoa), "critical AoA {aoa}");
    assert!((42.0..=58.0).contains(&ias), "stall speed {ias}");
    assert!(loss > 20.0, "should lose height, lost {loss}");
}

#[test]
fn icing_raises_stall_speed_significantly() {
    let (_, clean, _) = stall_experiment(0.0);
    let (_, iced, _) = stall_experiment(0.5);
    println!("stall speed clean {clean:.1} kt, 50% ice {iced:.1} kt");
    assert!(iced > clean * 1.08, "clean {clean} iced {iced}");
}

/// 以最佳爬升速度持续爬升，返回 (高度, 爬升率) 采样。
fn climb_profile() -> Vec<(f64, f64)> {
    let mut sim = new_sim();
    sim.set_airborne(300.0, 74.0 * KT, 0.0);
    sim.state.fuel_kg = 144.0;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut sp = SpeedToPitch::default();
    let mut out = Vec::new();
    let mut last_alt = sim.state.rigid.pos.y;
    let mut smooth_vs = 0.0;
    for i in 0..(120 * 60 * 90) {
        sim.controls.throttle = 1.0;
        let tp = sp.target_pitch(&sim, 70.0 * KT, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        smooth_vs += (sim.diag.vertical_speed - smooth_vs) * 0.002;
        if i % (120 * 30) == 0 {
            let alt = sim.state.rigid.pos.y;
            out.push((alt, smooth_vs));
            // 1 小时内累计爬升率已接近 0 → 结束
            if i > 120 * 600 && smooth_vs < 0.15 {
                break;
            }
            last_alt = alt;
        }
    }
    let _ = last_alt;
    out
}

#[test]
fn climb_rate_naturally_decays_to_zero_near_ceiling() {
    let prof = climb_profile();
    for (a, v) in prof.iter().step_by(6) {
        println!("alt {a:.0} m  roc {v:.2} m/s ({:.0} fpm)", v * 196.85);
    }
    let roc_at = |alt: f64| prof.iter().min_by(|a, b| (a.0 - alt).abs().partial_cmp(&(b.0 - alt).abs()).unwrap()).unwrap().1;
    let (max_alt, _) = *prof.last().unwrap();
    assert!(roc_at(600.0) > 2.5, "sea-level ROC {}", roc_at(600.0));
    assert!(roc_at(2500.0) > roc_at(3800.0));
    assert!(roc_at(3800.0) < 1.6, "ROC at 3800 {}", roc_at(3800.0));
    assert!(max_alt > 3600.0 && max_alt < 5200.0, "service/absolute ceiling {max_alt}");
    let last_roc = prof.last().unwrap().1;
    assert!(last_roc < 0.5, "ROC near ceiling {last_roc}");
}

#[test]
fn overspeed_dive_breaks_wing() {
    let mut sim = new_sim();
    sim.set_airborne(3500.0, 100.0 * KT, 0.0);
    sim.controls.throttle = 1.0;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut warned = false;
    let mut broke_at = None;
    for _ in 0..(120 * 90) {
        sim.controls.pitch = ph.control(&sim, -40.0, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        warned |= sim.diag.warnings.overspeed;
        if sim.state.damage.left_wing_failed || sim.state.damage.right_wing_failed {
            broke_at = Some(sim.diag.ias_true / KT);
            break;
        }
    }
    println!("wing failed at {broke_at:?} kt");
    assert!(warned, "overspeed warning must precede failure");
    let v = broke_at.expect("wing should fail in sustained overspeed");
    assert!(v > 163.0, "failure below VNE: {v}");
}

#[test]
fn over_g_pull_breaks_wing_but_normal_g_does_not() {
    // 正常 2.5G 拉升不应损坏
    let mut sim = new_sim();
    sim.set_airborne(2500.0, 110.0 * KT, 0.0);
    sim.controls.throttle = 1.0;
    for _ in 0..(120 * 4) {
        sim.controls.pitch = 0.3;
        sim.step();
    }
    assert!(sim.diag.nz > 1.2);
    assert!(!sim.state.damage.any(), "normal pull damaged aircraft (nz {})", sim.diag.nz);

    // 150 kt 全拉杆：超过极限过载 → 折翼
    let mut sim = new_sim();
    sim.set_airborne(2500.0, 150.0 * KT, 0.0);
    sim.controls.throttle = 1.0;
    let mut max_nz: f64 = 0.0;
    for _ in 0..(120 * 6) {
        sim.controls.pitch = 1.0;
        sim.step();
        max_nz = max_nz.max(sim.diag.nz);
        if sim.state.damage.left_wing_failed || sim.state.damage.right_wing_failed {
            break;
        }
    }
    println!("max nz {max_nz:.2}");
    assert!(max_nz > 3.8, "should exceed limit load, got {max_nz}");
    assert!(sim.state.damage.left_wing_failed || sim.state.damage.right_wing_failed);
}

#[test]
fn icing_environment_degrades_airplane_and_freezes_pitot() {
    use sim_core::weather::CloudLayer;
    let mut sim = new_sim();
    sim.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0); // 云中约 -9℃
    sim.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    sim.set_airborne(2000.0, 90.0 * KT, 0.0);
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut alt_pid = Pid::new(0.6, 0.02, 0.0, 10.0);
    for _ in 0..(120 * 200) {
        sim.controls.throttle = 0.85;
        let tp = 3.0 + alt_pid.update(-(sim.state.rigid.pos.y - 2000.0) * 0.05 - sim.diag.vertical_speed * 0.5, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp.clamp(-10.0, 15.0), FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
            }
    println!("ice {:.2} pitot_blocked {}", sim.state.ice_load, sim.state.pitot_blocked);
    assert!(sim.diag.in_cloud);
    assert!(sim.state.ice_load > 0.3);
    assert!(sim.state.pitot_blocked, "pitot should freeze with heat off");
    // 皮托管冻结后，仪表空速不再跟随真实空速
    sim.controls.throttle = 0.3;
    for _ in 0..(120 * 20) {
        sim.step();
    }
    assert!((sim.diag.ias - sim.diag.ias_true).abs() > 2.0 * KT, "frozen IAS {} vs true {}", sim.diag.ias, sim.diag.ias_true);
}

/// 进近 → 接地 → 刹车停稳。
#[test]
fn approach_touchdown_and_brake_to_stop() {
    let mut sim = Simulation::new(sky172(), Box::new(FlatGround::default()), (0.0, 4200.0), 0.0);
    // 跑道入口在 z=0，向北（-Z）。从入口前 4000 m、3° 下滑道上开始进近。
    let start_z = 4000.0;
    // 下滑道瞄准点：入口后 400 m
    let alt = (start_z + 400.0) * 3.0f64.to_radians().tan() + 8.0;
    sim.state.rigid.pos.z = start_z;
    sim.state.rigid.pos.x = 0.0;
    sim.set_airborne(alt, 68.0 * KT, 0.0);
    sim.controls.flaps = 2.0 / 3.0;
    sim.state.surfaces.flaps = 2.0 / 3.0;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut tp = Pid::new(0.8, 0.05, 0.0, 6.0);
    let mut thr = Pid::new(0.04, 0.01, 0.0, 0.5);
    let mut touched = None;
    let mut stopped_at = None;
    for _ in 0..(120 * 400) {
        let agl = sim.diag.agl - sim.config.rest_cg_height;
        let on_ground = sim.state.on_ground;
        if !on_ground {
            let tgt_ias = if agl < 15.0 { 62.0 * KT } else { 68.0 * KT };
            // 下滑道：目标垂直速率 ≈ -V·tan(3°)；5 m 以下拉平
            let vs_t = if agl < 6.0 { -0.6 } else { -sim.diag.ground_speed * 3.0f64.to_radians().tan() };
            let pitch_t = 2.0 + tp.update(vs_t - sim.diag.vertical_speed, FIXED_DT);
            sim.controls.pitch = ph.control(&sim, pitch_t.clamp(-6.0, 9.0), FIXED_DT);
            sim.controls.throttle = if agl < 5.0 { 0.0 } else { (0.35 + thr.update((tgt_ias - sim.diag.ias_true) / KT, FIXED_DT)).clamp(0.0, 1.0) };
            sim.controls.roll = {
                let hdg = 360.0 + (-sim.state.rigid.pos.x * 0.02).clamp(-8.0, 8.0);
                let bank = heading_to_bank(&sim, hdg, 12.0);
                bh.control(&sim, bank, FIXED_DT)
            };
            sim.controls.yaw = 0.0;
        } else {
            if touched.is_none() {
                touched = Some((sim.state.rigid.pos.z, sim.state.last_touchdown_sink, sim.diag.ias_true / KT));
            }
            sim.controls.throttle = 0.0;
            sim.controls.flaps = 0.0;
            sim.controls.pitch = 0.4;
            sim.controls.roll = 0.0;
            sim.controls.yaw = heading_to_rudder(&sim, 0.0);
            if sim.diag.ground_speed < 35.0 {
                sim.controls.brake_left = 0.8;
                sim.controls.brake_right = 0.8;
            }
        }
        sim.step();
        if (sim.state.time * 10.0) as i64 % 100 == 0 && sim.state.time.fract() < FIXED_DT {
            println!("t={:.0} z={:.0} x={:.1} agl={:.1} ias={:.0} vs={:.2} pitch={:.1} gnd={}", sim.state.time, sim.state.rigid.pos.z, sim.state.rigid.pos.x, sim.diag.agl, sim.diag.ias_true / KT, sim.diag.vertical_speed, sim.diag.pitch, sim.state.on_ground);
        }
        if sim.state.on_ground && sim.diag.ground_speed < 0.3 && touched.is_some() {
            stopped_at = Some(sim.state.rigid.pos.z);
            break;
        }
        if sim.state.damage.crashed {
            break;
        }
    }
    let (tz, sink, tias) = touched.expect("never touched down");
    println!("touchdown z={tz:.0} sink={sink:.2} m/s ias={tias:.0}kt, stopped at {stopped_at:?}");
    assert!(!sim.state.damage.crashed, "crashed");
    assert!(!sim.state.damage.any(), "damage {:?}", sim.state.damage);
    assert!(sink < 2.5, "touchdown sink {sink}");
    let sz = stopped_at.expect("did not stop");
    // 跑道 1500 m：入口 z=0，末端 z=-1500
    assert!(tz < 60.0 && tz > -900.0, "touchdown point {tz}");
    assert!(sz > -1500.0, "overran runway, stopped at {sz}");
}
