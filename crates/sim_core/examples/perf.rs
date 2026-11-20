//! 性能测量工具：cargo run --release -p sim_core --example perf -- CODE [takeoff|climb|stall|roll|land|cruise|all]
use sim_core::aircraft::*;
use sim_core::atmosphere::{calibrated_airspeed, Atmosphere};
use sim_core::autopilot::*;
use sim_core::gear::{FlatGround, SurfaceType};
use sim_core::integrator::Simulation;
use sim_core::{FIXED_DT, KT};

fn new_sim(code: &str, water: bool) -> Simulation {
    let cfg = AircraftConfig::by_code(code).unwrap();
    let g = FlatGround { elevation: 0.0, surface: if water { SurfaceType::Water } else { SurfaceType::Asphalt } };
    let mut s = Simulation::new(cfg, Box::new(g), (0.0, 0.0), 0.0);
    if water { s.place_on_water(); }
    s
}

fn takeoff_flaps(code: &str) -> f64 {
    match code { "TWIN-6" => 0.27, "JET-39" => 0.5, "LINER-73" => 5.0 / 40.0, "SEA-415" => 0.33, _ => 0.0 }
}

fn pgains(code: &str) -> (PitchHold, BankHold) {
    let _ = code;
    (PitchHold::default(), BankHold::default())
}

/// 返回 (liftoff IAS kt, 滑跑距离 m, 过 15m 距离, 是否稳定爬升)
fn takeoff(code: &str, water: bool, verbose: bool) -> (f64, f64, f64) {
    let mut sim = new_sim(code, water);
    let cfg = sim.config.clone();
    sim.controls.flaps = takeoff_flaps(code);
    sim.state.surfaces.flaps = sim.controls.flaps;
    sim.controls.parking_brake = true;
    for _ in 0..360 { sim.step(); }
    sim.controls.parking_brake = false;
    sim.controls.throttle = 1.0;
    let (mut ph, _) = pgains(code);
    let z0 = sim.state.rigid.pos.z;
    let mut lift: Option<(f64, f64)> = None;
    let mut d15 = f64::NAN;
    let mut tmax = 240;
    if water { tmax = 400; }
    for i in 0..(120 * tmax) {
        let ias = sim.diag.ias_true / KT;
        sim.controls.yaw = heading_to_rudder(&sim, 0.0);
        let rot = cfg.v_rotate / KT;
        let target = if ias >= rot { 9.0 } else { 0.0 };
        sim.controls.pitch = ph.control(&sim, target, FIXED_DT);
        sim.step();
        if sim.state.damage.crashed { println!("  CRASHED t={:.1}", sim.state.time); break; }
        if lift.is_none() && !sim.state.on_ground && sim.state.air_time > 0.3 {
            lift = Some((sim.diag.ias_true / KT, (sim.state.rigid.pos.z - z0).abs()));
        }
        if lift.is_some() && d15.is_nan() && sim.diag.agl > 15.0 + cfg.rest_cg_height {
            d15 = (sim.state.rigid.pos.z - z0).abs();
        }
        if verbose && i % 600 == 0 {
            println!("   t={:.1} ias={:.0} gs={:.0} pitch={:.1} z={:.0} y={:.2} draft={:.2}", sim.state.time, ias, sim.diag.ground_speed / KT, sim.diag.pitch, (sim.state.rigid.pos.z - z0).abs(), sim.state.rigid.pos.y, sim.diag.gear.total_draft);
        }
        if d15.is_finite() { break; }
    }
    let (v, d) = lift.unwrap_or((f64::NAN, f64::NAN));
    (v, d, d15)
}

fn ias_for_mach(m: f64, alt: f64) -> f64 {
    let atm = Atmosphere::default().sample(alt);
    calibrated_airspeed(m * atm.speed_of_sound, &atm) / KT
}

/// 爬升到升限。返回 (alt, roc) 剖面。
fn climb(code: &str, verbose: bool) -> Vec<(f64, f64, f64)> {
    let mut sim = new_sim(code, false);
    let cfg = sim.config.clone();
    sim.state.fuel_kg = cfg.fuel_capacity * 0.7;
    let (mut ph, mut bh) = pgains(code);
    let mut sp = SpeedToPitch::default();
    let start = 500.0;
    sim.set_airborne(start, cfg.v_best_climb, 0.0);
    sim.set_gear_instant(false);
    sim.controls.gear_down = false;
    let mach_climb = match code { "LINER-73" => 0.74, "JET-39" => 0.5, "TWIN-6" => 0.4, _ => 0.4 };
    let mut out = vec![];
    let mut smooth = 0.0;
    for i in 0..(120 * 60 * 150) {
        let alt = sim.state.rigid.pos.y;
        let tgt = (cfg.v_best_climb / KT).min(ias_for_mach(mach_climb, alt)).max(60.0);
        sim.controls.throttle = 1.0;
        let tp = sp.target_pitch(&sim, tgt * KT, FIXED_DT);
        sim.controls.pitch = ph.control(&sim, tp, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        smooth += (sim.diag.vertical_speed - smooth) * 0.002;
        if i % (120 * 30) == 0 {
            out.push((alt, smooth, sim.diag.ias_true / KT));
            if verbose && out.len() % 6 == 0 { println!("   alt {:.0} roc {:.2} ias {:.0} mach {:.2} fuel {:.0}", alt, smooth, sim.diag.ias_true / KT, sim.diag.mach, sim.state.fuel_kg); }
            if i > 120 * 600 && smooth < 0.15 { break; }
            if sim.state.damage.any() { println!("  DAMAGE at {alt:.0}"); break; }
        }
    }
    out
}

/// 平飞减速失速：返回 (失速 IAS kt, 峰值 AoA)
fn stall(code: &str, flaps: f64, gear: bool) -> (f64, f64) {
    let mut sim = new_sim(code, false);
    let cfg = sim.config.clone();
    let alt = 3000.0;
    sim.set_airborne(alt, cfg.v_approach * 1.5, 0.0);
    sim.controls.flaps = flaps;
    sim.state.surfaces.flaps = flaps;
    sim.set_gear_instant(gear);
    let (mut ph, mut bh) = pgains(code);
    let mut target = 3.0;
    let mut peak_cl = 0.0;
    let mut res = (0.0, 0.0);
    for _ in 0..(120 * 400) {
        sim.controls.throttle = 0.0;
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        target += 0.3 * FIXED_DT;
        sim.controls.pitch = ph.control(&sim, target.min(30.0), FIXED_DT);
        sim.step();
        let cl = sim.diag.aero.surfaces[1].cl;
        if sim.state.time < 4.0 { continue; }
        if cl > peak_cl { peak_cl = cl; res = (sim.diag.ias_true / KT, sim.diag.aoa.to_degrees()); }
        if cl < peak_cl - 0.3 && sim.diag.ias_true / KT < cfg.v_approach / KT * 1.2 { break; }
    }
    res
}

/// 满舵副翼稳态滚转率（deg/s）
fn roll_rate(code: &str, ias_kt: f64, alt: f64) -> f64 {
    let mut sim = new_sim(code, false);
    sim.set_airborne(alt, ias_kt * KT, 0.0);
    sim.set_gear_instant(false);
    sim.controls.throttle = 0.6;
    let (mut ph, _) = pgains(code);
    let mut best: f64 = 0.0;
    for i in 0..(120 * 4) {
        sim.controls.roll = 1.0;
        sim.controls.pitch = ph.control(&sim, 2.0, FIXED_DT);
        sim.step();
        if i > 120 * 1 { best = best.max((-sim.state.rigid.w.z).to_degrees()); }
    }
    best
}

/// 进近 + 着陆滑跑。返回 (接地 IAS, 接地下沉率, 停止距离)
fn land(code: &str, flaps: f64, reverse: bool, water: bool) -> (f64, f64, f64, bool) {
    let mut sim = new_sim(code, water);
    let cfg = sim.config.clone();
    let vref = cfg.v_approach / KT;
    let start_z = 6000.0;
    let alt = (start_z + 300.0) * 3.0f64.to_radians().tan() + 8.0;
    sim.state.rigid.pos.z = start_z;
    sim.set_airborne(alt, vref * KT, 0.0);
    sim.controls.flaps = flaps;
    sim.state.surfaces.flaps = flaps;
    sim.set_gear_instant(!water);
    let (mut ph, mut bh) = pgains(code);
    let mut tp = Pid::new(0.8, 0.05, 0.0, 6.0);
    let mut thr = Pid::new(0.04, 0.01, 0.0, 0.5);
    let mut td: Option<(f64, f64, f64)> = None;
    let mut stop_z = f64::NAN;
    let mut crashed = false;
    for i in 0..(120 * 600) {
        let agl = sim.diag.agl - cfg.rest_cg_height;
        if !sim.state.on_ground {
            let tgt = if agl < 15.0 { vref - 4.0 } else { vref };
            let hf = (sim.diag.ground_speed * 0.16).max(7.0);
            let vs_t = if agl < hf { -0.5 - 1.5 * (agl / hf) } else { -sim.diag.ground_speed * 3.0f64.to_radians().tan() };
            let pitch_t = 3.0 + tp.update(vs_t - sim.diag.vertical_speed, FIXED_DT);
            sim.controls.pitch = ph.control(&sim, pitch_t.clamp(-6.0, 10.0), FIXED_DT);
            sim.controls.throttle = if agl < hf * 0.6 { 0.0 } else { (0.35 + thr.update((tgt - sim.diag.ias_true / KT) , FIXED_DT)).clamp(0.0, 1.0) };
            sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
            sim.controls.yaw = 0.0;
        } else {
            if td.is_none() { td = Some((sim.state.rigid.pos.z, sim.state.last_touchdown_sink, sim.diag.ias_true / KT)); }
            sim.controls.throttle = if reverse { 0.9 } else { 0.0 };
            sim.controls.reverse = reverse;
            sim.controls.speedbrake = 1.0;
            sim.controls.pitch = 0.0;
            sim.controls.roll = 0.0;
            sim.controls.yaw = heading_to_rudder(&sim, 0.0);
            if !water { sim.controls.brake_left = 1.0; sim.controls.brake_right = 1.0; }
            if sim.diag.ground_speed < 15.0 { sim.controls.reverse = false; sim.controls.throttle = 0.0; }
            if water && sim.diag.ground_speed < 9.0 { for e in sim.state.engines.iter_mut() { e.running = false; } }
            if sim.diag.ground_speed < if water { 2.0 } else { 0.5 } { stop_z = sim.state.rigid.pos.z; break; }
        }
        sim.step();
        if std::env::var("VERB").is_ok() && td.is_some() && i % 300 == 0 { println!("   land t={:.1} gs={:.1} y={:.2} pitch={:.1} draft={:.2} z={:.0}", sim.state.time, sim.diag.ground_speed, sim.state.rigid.pos.y, sim.diag.pitch, sim.diag.gear.total_draft, sim.state.rigid.pos.z); }
        let _ = i;
        if sim.state.damage.crashed { crashed = true; break; }
    }
    let (z, sink, ias) = td.unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    (ias, sink, (z - stop_z).abs(), crashed)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let code = args.get(1).map(|s| s.as_str()).unwrap_or("TWIN-6");
    let what = args.get(2).map(|s| s.as_str()).unwrap_or("all");
    let all = what == "all";
    let cfg = AircraftConfig::by_code(code).unwrap();
    println!("== {} == MTOW {:.0} kg  wing {:.1} m2", code, cfg.max_takeoff_mass(), cfg.wing_area);
    if all || what == "takeoff" {
        let (v, d, d15) = takeoff(code, false, what == "takeoff");
        println!("takeoff: liftoff {v:.1} kt  roll {d:.0} m  to-15m {d15:.0} m");
    }
    if all || what == "stall" {
        let (v0, a0) = stall(code, 0.0, false);
        let (v1, a1) = stall(code, *cfg.flap_detents.last().unwrap(), true);
        println!("stall: clean {v0:.1} kt (aoa {a0:.1})   landing cfg {v1:.1} kt (aoa {a1:.1})");
    }
    if all || what == "roll" {
        println!("roll rate: low {:.0} deg/s @ {:.0}kt, fast {:.0} deg/s @ {:.0}kt", roll_rate(code, cfg.v_approach / KT * 1.6, 1500.0), cfg.v_approach / KT * 1.6, roll_rate(code, cfg.v_approach / KT * 2.4, 1500.0), cfg.v_approach / KT * 2.4);
    }
    if all || what == "land" {
        let fl = *cfg.flap_detents.last().unwrap() * 0.8;
        let (ias, sink, dist, crashed) = land(code, fl, false, false);
        println!("landing: touchdown {ias:.0} kt sink {sink:.2} m/s  stop dist {dist:.0} m crashed={crashed}");
        if cfg.engines.iter().any(|e| !matches!(e.kind, EngineKind::Piston { .. })) {
            let (ias, sink, dist, crashed) = land(code, fl, true, false);
            println!("landing w/ reverse: touchdown {ias:.0} kt sink {sink:.2}  stop dist {dist:.0} m crashed={crashed}");
        }
    }
    if all || what == "climb" {
        let prof = climb(code, what == "climb");
        let (a, r, _) = *prof.last().unwrap();
        let roc = |alt: f64| prof.iter().min_by(|x, y| (x.0 - alt).abs().partial_cmp(&(y.0 - alt).abs()).unwrap()).unwrap().1;
        println!("climb: ceiling(final) {a:.0} m  roc {r:.2}  roc@1000 {:.1} m/s roc@3000 {:.1} roc@6000 {:.1} (profile pts {})", roc(1000.0), roc(3000.0), roc(6000.0), prof.len());
        // 业务升限：ROC < 0.5 m/s 首次出现的高度
        if let Some(p) = prof.iter().find(|p| p.1 < 0.5 && p.0 > 1500.0) { println!("       roc<0.5 at {:.0} m", p.0); }
    }
    if what == "water" {
        let (v, d, d15) = takeoff(code, true, true);
        println!("water takeoff: liftoff {v:.1} kt  run {d:.0} m  to-15m {d15:.0} m");
        let (ias, sink, dist, crashed) = land(code, 0.66, true, true);
        println!("water landing: touchdown {ias:.0} kt sink {sink:.2} m/s  stop dist {dist:.0} m crashed={crashed}");
    }
    if what == "cruise" {
        for alt in [1000.0, 3000.0, 6000.0, 9000.0, 11000.0] {
            let mut sim = new_sim(code, false);
            sim.set_airborne(alt, cfg.v_best_climb, 0.0);
            sim.set_gear_instant(false);
            let (mut ph, mut bh) = pgains(code);
            let mut pid = Pid::new(0.5, 0.02, 0.0, 10.0);
            for _ in 0..(120 * 600) {
                sim.controls.throttle = 1.0;
                let tp = 2.0 + pid.update(-(sim.state.rigid.pos.y - alt) * 0.05 - sim.diag.vertical_speed * 0.5, FIXED_DT);
                sim.controls.pitch = ph.control(&sim, tp.clamp(-10.0, 12.0), FIXED_DT);
                sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
                sim.step();
            }
            println!("  cruise @ {alt:.0} m: TAS {:.0} kt IAS {:.0} kt M{:.2} alt {:.0}", sim.diag.tas / KT, sim.diag.ias_true / KT, sim.diag.mach, sim.state.rigid.pos.y);
        }
    }
}
