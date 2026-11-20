//! ISA 标准大气 + 当地温度偏移 ΔT + 海平面气压 QNH。
//! 升限不在此处硬编码——由密度对推力与升力的影响自然涌现。

pub const T0: f64 = 288.15;
pub const P0: f64 = 101_325.0;
pub const RHO0: f64 = 1.225;
pub const R_AIR: f64 = 287.052_87;
pub const GAMMA: f64 = 1.4;
pub const LAPSE: f64 = 0.0065;
pub const A0: f64 = 340.294;
pub const TROPOPAUSE: f64 = 11_000.0;
const G0: f64 = 9.806_65;

#[derive(Clone, Copy, Debug)]
pub struct AtmosphereState {
    pub altitude: f64,
    pub temperature: f64, // K
    pub pressure: f64,    // Pa
    pub density: f64,     // kg/m³
    pub speed_of_sound: f64,
}

impl AtmosphereState {
    pub fn temperature_c(&self) -> f64 {
        self.temperature - 273.15
    }
    pub fn sigma(&self) -> f64 {
        self.density / RHO0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Atmosphere {
    /// 当地海平面温度相对 ISA 的偏移（K）。
    pub delta_t: f64,
    /// 海平面气压（Pa）。
    pub qnh: f64,
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self { delta_t: 0.0, qnh: P0 }
    }
}

impl Atmosphere {
    pub fn new(delta_t: f64, qnh: f64) -> Self {
        Self { delta_t, qnh }
    }

    /// 在几何高度 `h`（MSL，m）处采样大气。
    pub fn sample(&self, h: f64) -> AtmosphereState {
        let h = h.clamp(-500.0, 40_000.0);
        let t_sl = T0 + self.delta_t;
        let expo = G0 / (R_AIR * LAPSE);
        let (t, p) = if h <= TROPOPAUSE {
            let t = t_sl - LAPSE * h;
            (t, self.qnh * (t / t_sl).powf(expo))
        } else {
            let t11 = t_sl - LAPSE * TROPOPAUSE;
            let p11 = self.qnh * (t11 / t_sl).powf(expo);
            (t11, p11 * (-G0 * (h - TROPOPAUSE) / (R_AIR * t11)).exp())
        };
        AtmosphereState {
            altitude: h,
            temperature: t,
            pressure: p,
            density: p / (R_AIR * t),
            speed_of_sound: (GAMMA * R_AIR * t).sqrt(),
        }
    }

    pub fn temperature_c(&self, h: f64) -> f64 {
        self.sample(h).temperature_c()
    }
}

/// 气压高度计示数（m）：按 ISA 反算，`setting` 为拨表气压（Pa）。
pub fn indicated_altitude(pressure: f64, setting: f64) -> f64 {
    T0 / LAPSE * (1.0 - (pressure / setting).powf(R_AIR * LAPSE / G0))
}

/// 真空速 → 校正空速 CAS（亚音速，由冲压 qc 反算）。即“表速 IAS”（忽略仪表误差）。
pub fn calibrated_airspeed(tas: f64, st: &AtmosphereState) -> f64 {
    let m = tas / st.speed_of_sound;
    let qc = st.pressure * ((1.0 + 0.2 * m * m).powf(3.5) - 1.0);
    A0 * (5.0 * ((qc / P0 + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt()
}

/// CAS → TAS。
pub fn tas_from_cas(cas: f64, st: &AtmosphereState) -> f64 {
    let qc = P0 * ((1.0 + 0.2 * (cas / A0).powi(2)).powf(3.5) - 1.0);
    let m2 = 5.0 * ((qc / st.pressure + 1.0).powf(2.0 / 7.0) - 1.0);
    m2.max(0.0).sqrt() * st.speed_of_sound
}

/// 等效空速 EAS。
pub fn equivalent_airspeed(tas: f64, st: &AtmosphereState) -> f64 {
    tas * st.sigma().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn isa_sea_level_and_altitude() {
        let a = Atmosphere::default();
        let s = a.sample(0.0);
        assert!((s.density - 1.225).abs() < 0.002);
        let s5 = a.sample(5000.0);
        assert!((s5.density - 0.7364).abs() < 0.01, "{}", s5.density);
        let s11 = a.sample(11000.0);
        assert!((s11.temperature - 216.65).abs() < 0.01);
        assert!(a.sample(12000.0).density < s11.density);
    }
    #[test]
    fn cas_tas_roundtrip() {
        let st = Atmosphere::default().sample(4000.0);
        let cas = calibrated_airspeed(80.0, &st);
        assert!(cas < 80.0);
        assert!((tas_from_cas(cas, &st) - 80.0).abs() < 1e-6);
    }
    #[test]
    fn warm_day_is_less_dense() {
        assert!(Atmosphere::new(20.0, P0).sample(0.0).density < 1.225);
    }
}
