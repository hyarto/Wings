//! M3 验收补充：3D 风场、风切变、乱流、降水/能见度、湿滑跑道、结冰链路（含皮托管加热/除冰）、
//! 结构损坏（超 G、负 G、V_FE 卡襟翼、重着陆、折翼后果）。全部无头运行。

use glam::DVec3;
use sim_core::aircraft::sky172;
use sim_core::atmosphere::Atmosphere;
use sim_core::autopilot::*;
use sim_core::gear::FlatGround;
use sim_core::integrator::Simulation;
use sim_core::weather::*;
use sim_core::{FIXED_DT, KT};

fn new_sim() -> Simulation {
    Simulation::new(sky172(), Box::new(FlatGround::default()), (0.0, 0.0), 0.0)
}

/// 简易定高定航飞行：保持俯仰/坡度，返回每步 nz 序列。
fn fly_level(sim: &mut Simulation, secs: f64, throttle: f64, target_alt: f64) -> Vec<f64> {
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut alt = Pid::new(0.5, 0.02, 0.0, 8.0);
    let mut nz = Vec::new();
    for _ in 0..(secs * 120.0) as usize {
        sim.controls.throttle = throttle;
        let tp = 3.0 + alt.update(-(sim.state.rigid.pos.y - target_alt) * 0.05 - sim.diag.vertical_speed * 0.5, FIXED_DT);
        sim.controls.pitch = ph.control(sim, tp.clamp(-8.0, 12.0), FIXED_DT);
        sim.controls.roll = bh.control(sim, 0.0, FIXED_DT);
        sim.step();
        nz.push(sim.diag.nz);
    }
    nz
}

fn std_dev(v: &[f64]) -> f64 {
    let m = v.iter().sum::<f64>() / v.len() as f64;
    (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
}

// ------------------------------------------------------------------ 风场 ----

#[test]
fn heading_vector_and_wind_from_convention() {
    let n = heading_vector(0.0);
    assert!((n - DVec3::new(0.0, 0.0, -1.0)).length() < 1e-12, "north is -Z");
    let e = heading_vector(90.0);
    assert!((e - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-12, "east is +X");
    // 西风（来自 270°）把物体吹向东（+X）
    let mut w = WeatherState::calm();
    w.set_wind_aloft(270.0, 10.0);
    assert!((w.wind_aloft - DVec3::new(10.0, 0.0, 0.0)).length() < 1e-9);
    // 北风（来自 0°）吹向南（+Z）
    w.set_surface_wind(0.0, 7.0);
    assert!((w.surface_wind - DVec3::new(0.0, 0.0, 7.0)).length() < 1e-9);
}

#[test]
fn wind_profile_increases_with_height_and_reaches_aloft_value() {
    let mut w = WeatherState::calm();
    w.set_surface_wind(270.0, 5.0);
    w.set_wind_aloft(270.0, 25.0);
    let at = |agl: f64| w.wind_at(DVec3::new(0.0, agl, 0.0), agl, 0.0).length();
    assert!(at(2.0) < at(10.0), "log profile: {} {}", at(2.0), at(10.0));
    assert!(at(10.0) < at(1500.0));
    assert!(at(1500.0) < at(5000.0) + 1e-9);
    assert!((at(6000.0) - 25.0).abs() < 1e-6, "aloft wind {}", at(6000.0));
    // 无乱流/阵风时风场与时间无关（确定性）
    assert_eq!(w.wind_at(DVec3::new(10.0, 800.0, 3.0), 800.0, 0.0), w.wind_at(DVec3::new(10.0, 800.0, 3.0), 800.0, 99.0));
}

#[test]
fn wind_shear_layer_step_applies_delta_across_layer() {
    let mut w = WeatherState::calm();
    w.shear_layers.push(ShearLayer { altitude: 1200.0, thickness: 100.0, delta: DVec3::new(-12.0, 0.0, 6.0) });
    let below = w.wind_at(DVec3::new(0.0, 1100.0, 0.0), 1100.0, 0.0);
    let mid = w.wind_at(DVec3::new(0.0, 1200.0, 0.0), 1200.0, 0.0);
    let above = w.wind_at(DVec3::new(0.0, 1300.0, 0.0), 1300.0, 0.0);
    let d = above - below;
    assert!((d.x + 12.0).abs() < 0.8 && (d.z - 6.0).abs() < 0.8, "delta {d:?}");
    // 层中点为一半（平滑过渡，不是阶跃跳变）
    let half = mid - below;
    assert!(half.x < -3.0 && half.x > -9.0, "mid-layer {half:?}");
}

#[test]
fn turbulence_and_gusts_are_deterministic_and_bounded() {
    let mut a = WeatherState::calm();
    a.set_surface_wind(240.0, 8.0);
    a.set_wind_aloft(240.0, 8.0);
    a.turbulence = 0.7;
    a.gust = 6.0;
    let b = a.clone();
    let mut c = a.clone();
    c.seed = 777;
    let mut differs = false;
    let mut max_dev: f64 = 0.0;
    let mean = DVec3::ZERO;
    let mut smooth = a.clone();
    smooth.turbulence = 0.0;
    smooth.gust = 0.0;
    for i in 0..400 {
        let p = DVec3::new(i as f64 * 13.0, 600.0 + i as f64, -(i as f64) * 7.0);
        let t = i as f64 * 0.37;
        let wa = a.wind_at(p, 600.0, t);
        assert_eq!(wa, b.wind_at(p, 600.0, t), "same seed must be identical");
        if (wa - c.wind_at(p, 600.0, t)).length() > 1e-6 {
            differs = true;
        }
        assert!(wa.is_finite());
        let _ = mean;
        max_dev = max_dev.max((wa - smooth.wind_at(p, 600.0, t)).length());
        assert!(wa.length() < 60.0, "wind magnitude runaway {}", wa.length());
    }
    assert!(differs, "different seed must change turbulence");
    assert!(max_dev > 1.0, "turbulence should perturb wind: {max_dev}");
    // 乱流强度为 0 时无扰动
    let w0 = smooth.wind_at(DVec3::new(0.0, 600.0, 0.0), 600.0, 0.0);
    let w1 = smooth.wind_at(DVec3::new(500.0, 600.0, 90.0), 600.0, 12.0);
    assert!((w0 - w1).length() < 1e-9);
}

#[test]
fn headwind_and_tailwind_change_ground_speed_not_airspeed() {
    let run = |from_deg: f64| {
        let mut sim = new_sim();
        sim.weather.set_wind_aloft(from_deg, 15.0);
        sim.weather.set_surface_wind(from_deg, 15.0);
        sim.set_airborne(2500.0, 90.0 * KT, 0.0);
        sim.state.rigid.vel += sim.weather.wind_aloft; // 起始已处于气团中
        fly_level(&mut sim, 40.0, 0.7, 2500.0);
        (sim.diag.ground_speed, sim.diag.tas)
    };
    let (gs_head, tas_head) = run(0.0); // 北风 → 迎风飞北
    let (gs_tail, tas_tail) = run(180.0); // 南风 → 顺风飞北
    println!("head gs {gs_head:.1} tas {tas_head:.1}; tail gs {gs_tail:.1} tas {tas_tail:.1}");
    assert!((tas_head - tas_tail).abs() < 5.0, "airspeed should be wind independent");
    assert!(gs_tail - gs_head > 22.0, "gs diff {}", gs_tail - gs_head);
}

#[test]
fn crosswind_drifts_ground_track_downwind() {
    let mut sim = new_sim();
    sim.weather.set_wind_aloft(90.0, 15.0); // 东风，吹向西（-X）
    sim.weather.set_surface_wind(90.0, 15.0);
    sim.set_airborne(2500.0, 90.0 * KT, 0.0);
    sim.state.rigid.vel += sim.weather.wind_aloft; // 起始已处于气团中
    fly_level(&mut sim, 60.0, 0.7, 2500.0);
    println!("x after 60 s crosswind = {:.0} m", sim.state.rigid.pos.x);
    assert!(sim.state.rigid.pos.x < -400.0, "should drift west, x {}", sim.state.rigid.pos.x);
    assert!(sim.diag.sideslip.abs().to_degrees() < 10.0);
    assert!(!sim.state.damage.any());
}

#[test]
fn crossing_shear_layer_changes_indicated_airspeed() {
    let mut sim = new_sim();
    // 层上方额外 20 m/s 北风分量（来自北 → 吹向 +Z，即迎面）
    sim.weather.shear_layers.push(ShearLayer { altitude: 3000.0, thickness: 60.0, delta: DVec3::new(0.0, 0.0, 20.0) });
    sim.set_airborne(2900.0, 80.0 * KT, 0.0);
    fly_level(&mut sim, 3.0, 0.6, 2900.0);
    let wind_below = sim.diag.wind;
    let ias_below = sim.diag.ias_true;
    sim.state.rigid.pos.y = 3100.0;
    sim.step();
    let wind_above = sim.diag.wind;
    println!("wind below {wind_below:?} above {wind_above:?}");
    assert!((wind_above - wind_below).length() > 15.0);
    let mut min_ias = f64::MAX;
    let mut max_ias: f64 = 0.0;
    for _ in 0..240 {
        sim.step();
        min_ias = min_ias.min(sim.diag.ias_true);
        max_ias = max_ias.max(sim.diag.ias_true);
    }
    assert!(
        (max_ias - ias_below).abs() > 5.0 * KT || (min_ias - ias_below).abs() > 5.0 * KT,
        "ias {} -> [{}, {}]",
        ias_below / KT,
        min_ias / KT,
        max_ias / KT
    );
}

#[test]
fn turbulence_increases_load_factor_variation() {
    let run = |turb: f64| {
        let mut sim = new_sim();
        sim.weather.set_surface_wind(270.0, 8.0);
        sim.weather.set_wind_aloft(270.0, 8.0);
        sim.weather.turbulence = turb;
        sim.set_airborne(1500.0, 95.0 * KT, 0.0);
        let nz = fly_level(&mut sim, 60.0, 0.65, 1500.0);
        (std_dev(&nz[600..]), sim.state.damage.any())
    };
    let (calm, d0) = run(0.0);
    let (rough, d1) = run(0.8);
    println!("nz sigma calm {calm:.4} rough {rough:.4}");
    assert!(rough > calm * 2.0 && rough > 0.03, "calm {calm} rough {rough}");
    assert!(!d0 && !d1, "moderate turbulence must not break the airplane");
}

#[test]
fn simulation_is_deterministic_with_weather() {
    let go = || {
        let mut sim = new_sim();
        sim.weather.set_wind_aloft(300.0, 12.0);
        sim.weather.turbulence = 0.5;
        sim.weather.gust = 4.0;
        sim.set_airborne(2000.0, 90.0 * KT, 10.0);
        fly_level(&mut sim, 20.0, 0.7, 2000.0);
        sim.state.rigid.pos
    };
    assert_eq!(go(), go());
}

// ------------------------------------------------------ 降水 / 云 / 能见度 ----

#[test]
fn visibility_degrades_with_fog_rain_snow_and_cloud() {
    let ground = DVec3::new(0.0, 50.0, 0.0);
    let clear = WeatherState::calm();
    assert!(clear.visibility(ground, 0.0) >= 29_000.0);

    let mut fog = WeatherState::calm();
    fog.fog_visibility = 300.0;
    assert!((fog.visibility(ground, 0.0) - 300.0).abs() < 1.0);
    // 雾只影响低空
    assert!(fog.visibility(DVec3::new(0.0, 1500.0, 0.0), 0.0) > 20_000.0);

    let mut rain = WeatherState::calm();
    rain.precip = PrecipKind::Rain;
    rain.precip_intensity = 0.8;
    let mut snow = rain.clone();
    snow.precip = PrecipKind::Snow;
    let (vr, vs) = (rain.visibility(ground, 0.0), snow.visibility(ground, 0.0));
    println!("rain vis {vr:.0}, snow vis {vs:.0}");
    assert!(vr < 3000.0 && vs < vr, "snow worse than rain");
    rain.precip_intensity = 0.2;
    assert!(rain.visibility(ground, 0.0) > vr, "lighter rain → better visibility");

    let mut cloudy = WeatherState::calm();
    cloudy.clouds.push(CloudLayer { base: 800.0, top: 2000.0, coverage: 1.0 });
    let inside = cloudy.visibility(DVec3::new(0.0, 1400.0, 0.0), 0.0);
    assert!(inside < 400.0, "in cloud vis {inside}");
    assert!(cloudy.visibility(DVec3::new(0.0, 2500.0, 0.0), 0.0) > 20_000.0);
    assert!(cloudy.in_cloud(DVec3::new(0.0, 1400.0, 0.0)));
    assert!(!cloudy.in_cloud(DVec3::new(0.0, 500.0, 0.0)));
    assert!(!cloudy.in_cloud(DVec3::new(0.0, 2100.0, 0.0)));
}

#[test]
fn precipitation_only_below_cloud_tops() {
    let mut w = WeatherState::calm();
    w.precip = PrecipKind::Rain;
    w.precip_intensity = 0.6;
    assert!(w.precip_at(DVec3::new(0.0, 1000.0, 0.0)) > 0.0);
    assert_eq!(w.precip_at(DVec3::new(0.0, 4000.0, 0.0)), 0.0);
    w.clouds.push(CloudLayer { base: 600.0, top: 1800.0, coverage: 1.0 });
    assert!(w.precip_at(DVec3::new(0.0, 1700.0, 0.0)) > 0.0);
    assert_eq!(w.precip_at(DVec3::new(0.0, 1900.0, 0.0)), 0.0);
    w.precip = PrecipKind::None;
    assert_eq!(w.precip_at(DVec3::new(0.0, 100.0, 0.0)), 0.0);
}

#[test]
fn broken_cloud_layer_has_gaps_and_overcast_is_continuous() {
    let mut w = WeatherState::calm();
    w.clouds.push(CloudLayer { base: 1000.0, top: 2000.0, coverage: 0.5 });
    let (mut cloudy, mut clear) = (0, 0);
    for i in 0..60 {
        for j in 0..60 {
            let p = DVec3::new(i as f64 * 900.0, 1500.0, j as f64 * 900.0);
            if w.cloud_density(p) > 0.15 {
                cloudy += 1;
            } else {
                clear += 1;
            }
        }
    }
    println!("broken layer: cloudy {cloudy}, clear {clear}");
    assert!(cloudy > 200 && clear > 200, "cloudy {cloudy} clear {clear}");
    w.clouds[0].coverage = 1.0;
    for i in 0..30 {
        assert!(w.in_cloud(DVec3::new(i as f64 * 1234.0, 1500.0, i as f64 * 777.0)));
    }
}

// ---------------------------------------------------------------- 地面摩擦 ----

/// 以 25 m/s 滑跑后全刹车到停；返回停止距离。
fn braking_distance(weather: impl FnOnce(&mut WeatherState)) -> f64 {
    let mut sim = new_sim();
    weather(&mut sim.weather);
    for _ in 0..240 {
        sim.controls.parking_brake = true;
        sim.step();
    }
    sim.controls.parking_brake = false;
    sim.state.rigid.vel = DVec3::new(0.0, 0.0, -25.0);
    let z0 = sim.state.rigid.pos.z;
    for _ in 0..(120 * 60) {
        sim.controls.throttle = 0.0;
        sim.controls.brake_left = 1.0;
        sim.controls.brake_right = 1.0;
        sim.controls.yaw = heading_to_rudder(&sim, 0.0);
        sim.step();
        if sim.diag.ground_speed < 0.3 {
            break;
        }
    }
    assert!(!sim.state.damage.crashed);
    (sim.state.rigid.pos.z - z0).abs()
}

#[test]
fn wet_and_icy_runways_lengthen_braking_distance() {
    // 注意：Atmosphere::new 的第一个参数是相对 ISA 的温差 ΔT（℃）
    let dry = braking_distance(|w| w.atmosphere = Atmosphere::new(5.0, 101_325.0));
    let wet = braking_distance(|w| {
        w.atmosphere = Atmosphere::new(5.0, 101_325.0);
        w.precip = PrecipKind::Rain;
        w.precip_intensity = 0.8;
    });
    let ice = braking_distance(|w| {
        w.atmosphere = Atmosphere::new(-20.0, 101_325.0);
        w.precip = PrecipKind::Snow;
        w.precip_intensity = 0.9;
    });
    println!("stopping distance dry {dry:.0} m, wet {wet:.0} m, ice/snow {ice:.0} m");
    assert!(wet > dry * 1.25, "wet {wet} dry {dry}");
    assert!(ice > wet * 1.4, "ice {ice} wet {wet}");
}

// ------------------------------------------------------------------ 结冰 ----

fn icing_cloud_sim() -> Simulation {
    let mut sim = new_sim();
    sim.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0);
    sim.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    sim.set_airborne(2000.0, 90.0 * KT, 0.0);
    sim
}

#[test]
fn no_icing_outside_temperature_window_or_without_moisture() {
    // 暖云（> +2 ℃）：不积冰
    let mut warm = new_sim();
    warm.weather.atmosphere = Atmosphere::new(15.0, 101_325.0);
    warm.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    warm.set_airborne(1500.0, 90.0 * KT, 0.0);
    fly_level(&mut warm, 120.0, 0.7, 1500.0);
    assert!(warm.diag.in_cloud);
    assert!(warm.state.ice_load < 0.01, "warm cloud ice {}", warm.state.ice_load);

    // 极冷云（< -20 ℃）：不积冰
    let mut frigid = new_sim();
    frigid.weather.atmosphere = Atmosphere::new(-40.0, 101_325.0);
    frigid.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    frigid.set_airborne(1500.0, 90.0 * KT, 0.0);
    fly_level(&mut frigid, 120.0, 0.7, 1500.0);
    assert!(frigid.state.ice_load < 0.01, "frigid ice {}", frigid.state.ice_load);

    // 冷晴空：不积冰
    let mut dry = new_sim();
    dry.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0);
    dry.set_airborne(1500.0, 90.0 * KT, 0.0);
    fly_level(&mut dry, 120.0, 0.7, 1500.0);
    assert!(dry.state.ice_load < 0.01);
    assert!(!dry.state.pitot_blocked);
}

#[test]
fn freezing_rain_without_cloud_also_accretes_ice() {
    let mut sim = new_sim();
    sim.weather.atmosphere = Atmosphere::new(-20.0, 101_325.0); // 1000 m ≈ -11 ℃
    sim.weather.precip = PrecipKind::Rain;
    sim.weather.precip_intensity = 0.8;
    sim.set_airborne(1000.0, 90.0 * KT, 0.0);
    fly_level(&mut sim, 150.0, 0.7, 1000.0);
    println!("freezing rain ice {:.2}", sim.state.ice_load);
    assert!(sim.state.ice_load > 0.15);
}

#[test]
fn icing_adds_weight_and_drag_and_lowers_performance() {
    let mut sim = new_sim();
    let m0 = sim.total_mass();
    sim.state.ice_load = 0.6;
    assert!(sim.total_mass() > m0 + 5.0, "ice mass {}", sim.total_mass() - m0);

    let cruise = |ice: f64| {
        let mut sim = new_sim();
        sim.weather.atmosphere = Atmosphere::new(-18.0, 101_325.0);
        sim.set_airborne(1800.0, 100.0 * KT, 0.0);
        sim.state.ice_load = ice;
        fly_level(&mut sim, 50.0, 0.75, 1800.0);
        sim.diag.ias_true / KT
    };
    let (clean, iced) = (cruise(0.0), cruise(0.8));
    println!("cruise ias clean {clean:.1} kt, iced {iced:.1} kt");
    assert!(iced < clean - 3.0, "ice must reduce speed at equal power");
}

#[test]
fn pitot_heat_prevents_pitot_freezing_in_icing_conditions() {
    let mut sim = icing_cloud_sim();
    assert!(sim.config.has_pitot_heat, "SKY-172 has pitot heat switch");
    sim.controls.pitot_heat = true;
    fly_level(&mut sim, 120.0, 0.85, 2000.0);
    assert!(sim.diag.in_cloud);
    assert!(!sim.state.pitot_blocked, "pitot heat on must keep pitot clear");
    assert!((sim.diag.ias - sim.diag.ias_true).abs() < 1.0);
    // 机身仍然积冰（SKY-172 无除冰）
    assert!(sim.state.ice_load > 0.2);
}

#[test]
fn pitot_unfreezes_after_heat_is_switched_on() {
    let mut sim = icing_cloud_sim();
    fly_level(&mut sim, 60.0, 0.85, 2000.0);
    assert!(sim.state.pitot_blocked);
    sim.controls.pitot_heat = true;
    fly_level(&mut sim, 30.0, 0.85, 2000.0);
    assert!(!sim.state.pitot_blocked, "heat clears blocked pitot");
}

#[test]
fn sky172_cannot_deice_but_equipped_airframe_can() {
    let mut sim = icing_cloud_sim();
    assert!(!sim.config.has_deice);
    sim.controls.deice = true;
    fly_level(&mut sim, 120.0, 0.85, 2000.0);
    let ice_without = sim.state.ice_load;
    assert!(ice_without > 0.2, "no equipment → ice accumulates: {ice_without}");

    let mut cfg = sky172();
    cfg.has_deice = true;
    let mut sim2 = Simulation::new(cfg, Box::new(FlatGround::default()), (0.0, 0.0), 0.0);
    sim2.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0);
    sim2.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    sim2.set_airborne(2000.0, 90.0 * KT, 0.0);
    sim2.controls.deice = true;
    fly_level(&mut sim2, 120.0, 0.85, 2000.0);
    println!("ice with de-ice {:.2}, without {:.2}", sim2.state.ice_load, ice_without);
    assert!(sim2.state.ice_load < ice_without * 0.5);

    // 冷晴空中除冰逐步清冰
    let mut cfg3 = sky172();
    cfg3.has_deice = true;
    let mut sim3 = Simulation::new(cfg3, Box::new(FlatGround::default()), (0.0, 0.0), 0.0);
    sim3.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0);
    sim3.set_airborne(1500.0, 90.0 * KT, 0.0);
    sim3.state.ice_load = 0.8;
    sim3.controls.deice = true;
    fly_level(&mut sim3, 30.0, 0.8, 1500.0);
    let after30 = sim3.state.ice_load;
    fly_level(&mut sim3, 40.0, 0.8, 1500.0);
    assert!(after30 < 0.8 && sim3.state.ice_load < after30);
    assert!(sim3.state.ice_load < 0.1, "should be clean after 70 s: {}", sim3.state.ice_load);
}

#[test]
fn ice_melts_in_warm_air() {
    let mut sim = new_sim();
    sim.weather.atmosphere = Atmosphere::new(20.0, 101_325.0);
    sim.set_airborne(500.0, 90.0 * KT, 0.0);
    sim.state.ice_load = 0.5;
    fly_level(&mut sim, 150.0, 0.7, 500.0);
    assert!(sim.state.ice_load < 0.05, "melted {}", sim.state.ice_load);
}

// ------------------------------------------------------------ 结构损坏 ----

#[test]
fn negative_g_beyond_ultimate_breaks_wing() {
    let mut sim = new_sim();
    sim.set_airborne(2500.0, 160.0 * KT, 0.0);
    sim.controls.throttle = 1.0;
    let mut min_nz: f64 = 0.0;
    for _ in 0..(120 * 6) {
        sim.controls.pitch = -1.0;
        sim.step();
        min_nz = min_nz.min(sim.diag.nz);
        if sim.state.damage.left_wing_failed || sim.state.damage.right_wing_failed {
            break;
        }
    }
    println!("min nz {min_nz:.2}");
    assert!(min_nz < -1.52, "must exceed negative limit load: {min_nz}");
    assert!(sim.diag.warnings.g_limit || sim.state.damage.stress > 0.0);
}

#[test]
fn flap_overspeed_warns_then_jams_flaps() {
    let mut sim = new_sim();
    sim.set_airborne(2500.0, 125.0 * KT, 0.0);
    sim.controls.flaps = 1.0;
    sim.state.surfaces.flaps = 1.0;
    sim.controls.throttle = 0.8;
    let mut warned = false;
    for _ in 0..(120 * 3) {
        sim.controls.pitch = 0.0;
        sim.step();
        warned |= sim.diag.warnings.flap_overspeed;
    }
    println!("flaps jammed: {:?}", sim.state.damage.flaps_jammed);
    assert!(warned, "V_FE exceedance must warn");
    assert!(sim.state.damage.flaps_jammed.is_some(), "flaps should jam at > 1.25 V_FE");
    let jammed_pos = sim.state.surfaces.flaps;
    sim.controls.flaps = 0.0;
    for _ in 0..(120 * 10) {
        sim.step();
    }
    assert!((sim.state.surfaces.flaps - jammed_pos).abs() < 1e-9);
}

#[test]
fn normal_flaps_below_vfe_never_warn() {
    let mut sim = new_sim();
    sim.set_airborne(1500.0, 70.0 * KT, 0.0);
    sim.controls.flaps = 1.0;
    let mut warned = false;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    for _ in 0..(120 * 20) {
        sim.controls.throttle = 0.45;
        sim.controls.pitch = ph.control(&sim, 2.0, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        warned |= sim.diag.warnings.flap_overspeed;
    }
    assert!(sim.diag.ias_true < 85.0 * KT, "test premise: below V_FE");
    assert!(!warned && sim.state.damage.flaps_jammed.is_none());
}

#[test]
fn hard_landing_damages_gear_and_soft_landing_does_not() {
    let drop_from = |vy: f64| {
        let mut sim = new_sim();
        sim.set_airborne(3.5, 60.0 * KT, 0.0);
        sim.state.rigid.pos.y = sim.config.rest_cg_height + 0.25;
        sim.state.rigid.vel.y = vy;
        sim.controls.throttle = 0.0;
        for _ in 0..(120 * 8) {
            sim.step();
            if sim.state.damage.crashed {
                break;
            }
        }
        (sim.state.last_touchdown_sink, sim.state.damage.clone())
    };
    let (sink_soft, soft) = drop_from(-0.8);
    assert!(sink_soft < 2.0, "soft sink {sink_soft}");
    assert!(!soft.hard_landing && !soft.gear_collapsed && !soft.crashed, "{soft:?}");

    let (sink_hard, hard) = drop_from(-6.5);
    println!("hard sink {sink_hard:.2} m/s → {hard:?}");
    assert!(sink_hard > 3.0);
    assert!(hard.hard_landing || hard.gear_collapsed || hard.crashed, "{hard:?}");
}

#[test]
fn wing_failure_makes_airplane_uncontrollable() {
    let mut sim = new_sim();
    sim.set_airborne(3000.0, 100.0 * KT, 0.0);
    fly_level(&mut sim, 5.0, 0.7, 3000.0);
    assert!(sim.diag.roll.abs() < 10.0);
    sim.state.damage.left_wing_failed = true;
    let mut max_roll: f64 = 0.0;
    let alt0 = sim.state.rigid.pos.y;
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    for _ in 0..(120 * 8) {
        sim.controls.throttle = 0.7;
        sim.controls.pitch = ph.control(&sim, 3.0, FIXED_DT);
        sim.controls.roll = bh.control(&sim, 0.0, FIXED_DT);
        sim.step();
        max_roll = max_roll.max(sim.diag.roll.abs());
        if sim.state.damage.crashed {
            break;
        }
    }
    println!("max roll after wing loss {max_roll:.0} deg, alt loss {:.0}", alt0 - sim.state.rigid.pos.y);
    assert!(max_roll > 60.0, "wing loss must cause violent roll: {max_roll}");
    assert!(alt0 - sim.state.rigid.pos.y > 100.0, "must lose height");
}

#[test]
fn structural_failure_is_irreversible() {
    let mut sim = new_sim();
    sim.set_airborne(3000.0, 90.0 * KT, 0.0);
    sim.state.damage.right_wing_failed = true;
    for _ in 0..(120 * 5) {
        sim.controls.throttle = 0.0;
        sim.step();
    }
    assert!(sim.state.damage.right_wing_failed);
    assert!(sim.state.damage.any());
}

#[test]
fn ground_collision_at_speed_is_a_crash() {
    let mut sim = new_sim();
    sim.set_airborne(40.0, 100.0 * KT, 0.0);
    let mut ph = PitchHold::default();
    for _ in 0..(120 * 20) {
        sim.controls.throttle = 0.2;
        sim.controls.pitch = ph.control(&sim, -30.0, FIXED_DT);
        sim.step();
        if sim.state.damage.crashed {
            break;
        }
    }
    assert!(sim.state.damage.crashed, "nose-down into terrain must crash");
}
