//! 整定工具：求巡航点水平飞行力矩平衡所需的平尾安装角。
use sim_core::aero::{self, AeroInputs, AeroOutput};
use sim_core::aircraft::*;
use sim_core::atmosphere::{tas_from_cas, Atmosphere};
use sim_core::state::{RigidState, SurfacePositions};
use sim_core::structure::DamageState;
use sim_core::{attitude_from_euler, FWD, G, KT};
use glam::DVec3;

fn eval(cfg: &AircraftConfig, alt: f64, ias: f64, mass: f64, flaps: f64, pitch_deg: f64) -> (f64, f64, f64) {
    let atm = Atmosphere::default().sample(alt);
    let tas = tas_from_cas(ias, &atm);
    let q = attitude_from_euler(0.0, pitch_deg, 0.0);
    let mut rs = RigidState::new(DVec3::new(0.0, alt, 0.0), q);
    rs.vel = DVec3::new(0.0, 0.0, -tas); // 水平飞行：速度水平
    let dmg = DamageState::default();
    let wind = vec![DVec3::ZERO; cfg.surfaces.len()];
    let inp = AeroInputs {
        rho: atm.density,
        wind: &wind,
        ice: 0.0,
        agl: 5000.0,
        surfaces: SurfacePositions { flaps, ..Default::default() },
        trim: 0.0,
        damage: &dmg,
        slipstream: 0.0,
        sound_speed: atm.speed_of_sound,
        gear_pos: 0.0,
    };
    let mut out = AeroOutput::default();
    aero::compute(cfg, &rs, &inp, &mut out);
    // 世界系竖直力 = 机体力旋转到世界
    let fw = q * out.force;
    let lift = fw.y;
    let _ = mass;
    // 推力 ≈ 阻力（沿机头方向分量），其力矩
    let drag_along = -(out.force.dot(FWD));
    let thrust = drag_along.max(0.0);
    let mut my = 0.0;
    let n = cfg.engines.len() as f64;
    for e in &cfg.engines {
        my += (e.pos.cross(FWD * (thrust / n))).x;
    }
    (lift, out.moment.x + my, tas)
}

fn trim(cfg: &mut AircraftConfig, alt: f64, ias: f64, mass: f64, flaps: f64) -> (f64, f64, f64) {
    // 找俯仰角使升力 = 重量
    let w = mass * G;
    let solve_pitch = |cfg: &AircraftConfig| {
        let (mut lo, mut hi) = (-10.0, 20.0);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            let (l, _, _) = eval(cfg, alt, ias, mass, flaps, mid);
            if l < w { lo = mid } else { hi = mid }
        }
        0.5 * (lo + hi)
    };
    let (mut a, mut b) = (-12.0, 8.0);
    let mut inc: f64 = 0.0;
    for _ in 0..50 {
        inc = 0.5 * (a + b);
        cfg.surfaces[4].incidence = inc.to_radians();
        let p = solve_pitch(cfg);
        let (_, m, _) = eval(cfg, alt, ias, mass, flaps, p);
        // 增大安装角 → 尾升力增大 → 抬头力矩（x 正）
        if m > 0.0 { a = inc } else { b = inc }
    }
    cfg.surfaces[4].incidence = inc.to_radians();
    let p = solve_pitch(cfg);
    let (_, m, tas) = eval(cfg, alt, ias, mass, flaps, p);
    (inc, p, m / tas.max(1.0))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let pts: &[(&str, f64, f64)] = &[
        ("TWIN-6", 3000.0, 125.0),
        ("JET-39", 5000.0, 290.0),
        ("LINER-73", 10_000.0, 260.0),
        ("SEA-415", 3000.0, 150.0),
    ];
    for (code, alt, ias_kt) in pts {
        if args.len() > 1 && !args[1].eq_ignore_ascii_case(code) {
            continue;
        }
        let mut cfg = AircraftConfig::by_code(code).unwrap();
        let mass = cfg.mass_empty + cfg.payload + cfg.default_fuel_kg() * 0.6;
        let (inc, pitch, m) = trim(&mut cfg, *alt, ias_kt * KT, mass, 0.0);
        println!("{code}: cruise {alt} m {ias_kt} kt -> tail incidence {inc:.2} deg, level pitch {pitch:.2} deg (resid {m:.2})");
        // 其它速度下的需要俯仰角 + 升降舵零位时俯仰力矩斜率（静稳定性）
        for v in [0.5, 0.7, 1.0, 1.4] {
            let (l, m1, tas) = eval(&cfg, *alt, ias_kt * KT * v, mass, 0.0, pitch);
            let _ = (l, tas);
            println!("   v x{v}: Mx at same pitch {m1:.0}");
        }
        let (_, m_lo, _) = eval(&cfg, *alt, ias_kt * KT, mass, 0.0, pitch - 1.0);
        let (_, m_hi, _) = eval(&cfg, *alt, ias_kt * KT, mass, 0.0, pitch + 1.0);
        println!("   dMx/dalpha per deg = {:.0}  (negative = stable)", (m_hi - m_lo) / 2.0);
    }
}
