//! M5：皮托加热 / 除冰 的面板状态（指示灯数据源）与除冰实际效果。

use sim_core::aircraft::{sky172, AircraftConfig};
use sim_core::atmosphere::Atmosphere;
use sim_core::autopilot::*;
use sim_core::gear::FlatGround;
use sim_core::integrator::{Simulation, SystemState};
use sim_core::weather::*;
use sim_core::{FIXED_DT, KT};

fn sim_of(cfg: AircraftConfig) -> Simulation {
    Simulation::new(cfg, Box::new(FlatGround::default()), (0.0, 0.0), 0.0)
}

fn icing_sim(code: &str) -> Simulation {
    let mut sim = sim_of(AircraftConfig::by_code(code).unwrap());
    sim.weather.atmosphere = Atmosphere::new(-14.0, 101_325.0);
    sim.weather.clouds.push(CloudLayer { base: 1000.0, top: 3500.0, coverage: 1.0 });
    sim.set_airborne(2000.0, 110.0 * KT, 0.0);
    sim
}

fn fly(sim: &mut Simulation, secs: f64, throttle: f64) {
    let mut ph = PitchHold::default();
    let mut bh = BankHold::default();
    let mut alt = Pid::new(0.5, 0.02, 0.0, 8.0);
    for _ in 0..(secs * 120.0) as usize {
        sim.controls.throttle = throttle;
        let tp = 3.0 + alt.update(-(sim.state.rigid.pos.y - 2000.0) * 0.05 - sim.diag.vertical_speed * 0.5, FIXED_DT);
        sim.controls.pitch = ph.control(sim, tp.clamp(-8.0, 12.0), FIXED_DT);
        sim.controls.roll = bh.control(sim, 0.0, FIXED_DT);
        sim.step();
    }
}

#[test]
fn pitot_heat_state_follows_switch_and_equipment() {
    let mut sim = sim_of(sky172());
    assert_eq!(sim.pitot_heat_state(), SystemState::Off);
    sim.controls.pitot_heat = true;
    assert_eq!(sim.pitot_heat_state(), SystemState::On);
    let mut cfg = sky172();
    cfg.has_pitot_heat = false;
    let sim2 = sim_of(cfg);
    assert_eq!(sim2.pitot_heat_state(), SystemState::NotFitted);
}

#[test]
fn deice_state_not_fitted_off_on_and_no_power() {
    // SKY-172 / JET-39 没有除冰系统：开关无效，面板必须能反映这一点
    for code in ["SKY-172", "JET-39"] {
        let mut s = sim_of(AircraftConfig::by_code(code).unwrap());
        s.controls.deice = true;
        assert_eq!(s.deice_state(), SystemState::NotFitted, "{code}");
    }
    let mut s = sim_of(AircraftConfig::by_code("TWIN-6").unwrap());
    assert_eq!(s.deice_state(), SystemState::Off);
    s.controls.deice = true;
    for e in &mut s.state.engines {
        e.running = true;
    }
    assert_eq!(s.deice_state(), SystemState::On);
    for e in &mut s.state.engines {
        e.running = false;
    }
    assert_eq!(s.deice_state(), SystemState::NoPower);
}

#[test]
fn deice_on_equipped_twin_keeps_airframe_clean_and_clears_existing_ice() {
    let mut off = icing_sim("TWIN-6");
    fly(&mut off, 120.0, 0.7);
    let ice_off = off.state.ice_load;
    assert!(ice_off > 0.3, "without de-ice ice builds up: {ice_off}");

    let mut on = icing_sim("TWIN-6");
    on.controls.deice = true;
    assert_eq!(on.deice_state(), SystemState::On);
    fly(&mut on, 120.0, 0.7);
    println!("TWIN-6 ice off {ice_off:.2} / on {:.2}", on.state.ice_load);
    assert!(on.state.ice_load < 0.1, "de-ice on keeps clean: {}", on.state.ice_load);

    let mut iced = icing_sim("TWIN-6");
    iced.state.ice_load = 0.9;
    iced.controls.deice = true;
    fly(&mut iced, 60.0, 0.7);
    assert!(iced.state.ice_load < 0.2, "clears existing ice: {}", iced.state.ice_load);
}

#[test]
fn deice_without_running_engine_reports_no_power() {
    let mut s = icing_sim("TWIN-6");
    s.controls.deice = true;
    for e in &mut s.state.engines {
        e.running = false;
        e.failed = true;
    }
    assert_eq!(s.deice_state(), SystemState::NoPower);
}
