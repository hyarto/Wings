//! 天气：3D 风场（盛行风 + 风切变层 + 乱流/阵风）、降水、云层、能见度、结冰状态机。

use crate::atmosphere::Atmosphere;
use crate::gear::GroundConditions;
use crate::noise::{fbm3, noise3};
use crate::{lerp, smoothstep};
use glam::DVec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrecipKind {
    None,
    Rain,
    Snow,
}

#[derive(Clone, Copy, Debug)]
pub struct CloudLayer {
    /// 云底/云顶（MSL, m）。
    pub base: f64,
    pub top: f64,
    /// 覆盖率 0..1（≥0.95 为连续云层）。
    pub coverage: f64,
}

/// 突变风切变层（例如逆温层/锋面）。
#[derive(Clone, Copy, Debug)]
pub struct ShearLayer {
    pub altitude: f64,
    pub thickness: f64,
    /// 穿越该层后增加的风矢量。
    pub delta: DVec3,
}

#[derive(Clone, Debug)]
pub struct WeatherState {
    pub atmosphere: Atmosphere,
    /// 10 m AGL 处风矢量（“吹向”方向，世界系）。
    pub surface_wind: DVec3,
    /// 高空（≥ 3 km AGL）风矢量。
    pub wind_aloft: DVec3,
    pub shear_layers: Vec<ShearLayer>,
    /// 乱流强度 0..1。
    pub turbulence: f64,
    /// 阵风幅度（m/s）。
    pub gust: f64,
    pub precip: PrecipKind,
    pub precip_intensity: f64,
    pub clouds: Vec<CloudLayer>,
    /// 雾导致的地面能见度（m）。
    pub fog_visibility: f64,
    pub seed: u32,
}

impl Default for WeatherState {
    fn default() -> Self {
        Self {
            atmosphere: Atmosphere::default(),
            surface_wind: DVec3::ZERO,
            wind_aloft: DVec3::ZERO,
            shear_layers: Vec::new(),
            turbulence: 0.0,
            gust: 0.0,
            precip: PrecipKind::None,
            precip_intensity: 0.0,
            clouds: Vec::new(),
            fog_visibility: 30_000.0,
            seed: 1,
        }
    }
}

/// 航向（度，0 = 北）→ 世界系水平单位向量（+X 东，-Z 北）。
pub fn heading_vector(heading_deg: f64) -> DVec3 {
    let h = heading_deg.to_radians();
    DVec3::new(h.sin(), 0.0, -h.cos())
}

impl WeatherState {
    pub fn calm() -> Self {
        Self::default()
    }

    /// 设置地面风：`from_deg` 为风的来向（气象习惯，如 270 = 西风）。
    pub fn set_surface_wind(&mut self, from_deg: f64, speed: f64) {
        self.surface_wind = -heading_vector(from_deg) * speed;
    }

    pub fn set_wind_aloft(&mut self, from_deg: f64, speed: f64) {
        self.wind_aloft = -heading_vector(from_deg) * speed;
    }

    /// 风矢量（m/s，世界系）。`agl` 为离地高度，`t` 为仿真时间。
    pub fn wind_at(&self, pos: DVec3, agl: f64, t: f64) -> DVec3 {
        let agl = agl.max(0.5);
        // 近地对数风廓线 → 梯度风
        let f_low = ((agl / 0.1).ln() / (100.0f64).ln()).clamp(0.0, 1.4);
        let low = self.surface_wind * f_low;
        let blend = smoothstep(200.0, 3500.0, agl);
        let mut w = DVec3::new(
            lerp(low.x, self.wind_aloft.x, blend),
            0.0,
            lerp(low.z, self.wind_aloft.z, blend),
        );
        for s in &self.shear_layers {
            let k = smoothstep(s.altitude - 0.5 * s.thickness, s.altitude + 0.5 * s.thickness, pos.y);
            w += s.delta * k;
        }
        let wind_speed = w.length();

        // 乱流（冻结湍流场 + 缓慢漂移）
        if self.turbulence > 0.0 {
            let sigma = self.turbulence * (1.0 + 0.12 * wind_speed) * (1.0 + 1.5 * (-agl / 250.0).exp());
            let (x, y, z) = (pos.x / 80.0 + t * 0.3, pos.y / 50.0, pos.z / 80.0);
            let tur = DVec3::new(
                fbm3(self.seed ^ 0x11, x, y, z, 3),
                0.6 * fbm3(self.seed ^ 0x22, x + 31.7, y, z, 3),
                fbm3(self.seed ^ 0x33, x, y, z + 17.3, 3),
            );
            w += tur * sigma * 2.5;
        }
        // 阵风：沿平均风向的慢变脉冲
        if self.gust > 0.0 && wind_speed > 0.1 {
            let g = (noise3(self.seed ^ 0x44, t / 5.0, 0.3, 0.7) * 0.5 + 0.5).powi(2);
            w += w / wind_speed * (self.gust * g);
        }
        w
    }

    /// 云密度 0..1（含水平覆盖噪声）。
    pub fn cloud_density(&self, pos: DVec3) -> f64 {
        let mut best: f64 = 0.0;
        for (i, c) in self.clouds.iter().enumerate() {
            if pos.y < c.base || pos.y > c.top {
                continue;
            }
            let mid = 0.5 * (c.base + c.top);
            let half = 0.5 * (c.top - c.base);
            let vprof = 1.0 - ((pos.y - mid) / half.max(1.0)).abs().powi(3);
            let horiz = if c.coverage >= 0.95 {
                1.0
            } else {
                let n = fbm3(self.seed.wrapping_add(500 + i as u32), pos.x / 1800.0, 0.0, pos.z / 1800.0, 3) * 0.5 + 0.5;
                smoothstep(0.0, 0.12, c.coverage * 1.0 - (1.0 - n) * 0.9 + 0.05)
            };
            best = best.max(vprof.clamp(0.0, 1.0) * horiz);
        }
        best
    }

    pub fn in_cloud(&self, pos: DVec3) -> bool {
        self.cloud_density(pos) > 0.15
    }

    /// 当前位置的降水强度 0..1。降水发生在“最低有效云层云顶”以下（无云层时 3 km 以下）。
    pub fn precip_at(&self, pos: DVec3) -> f64 {
        if self.precip == PrecipKind::None || self.precip_intensity <= 0.0 {
            return 0.0;
        }
        let ceiling = self
            .clouds
            .iter()
            .filter(|c| c.coverage > 0.5)
            .map(|c| c.top)
            .fold(f64::NAN, |a, b| if a.is_nan() { b } else { a.min(b) });
        let ceiling = if ceiling.is_nan() { 3000.0 } else { ceiling };
        if pos.y <= ceiling {
            self.precip_intensity
        } else {
            0.0
        }
    }

    /// 能见度（m）。
    pub fn visibility(&self, pos: DVec3, ground_alt: f64) -> f64 {
        let mut vis: f64 = 30_000.0;
        if pos.y - ground_alt < 600.0 {
            vis = vis.min(self.fog_visibility);
        }
        let p = self.precip_at(pos);
        if p > 0.0 {
            let k = if self.precip == PrecipKind::Snow { 14.0 } else { 9.0 };
            vis = vis.min(12_000.0 / (1.0 + k * p));
        }
        let cd = self.cloud_density(pos);
        if cd > 0.15 {
            vis = vis.min(lerp(500.0, 45.0, cd));
        }
        vis.max(30.0)
    }

    /// 地面湿滑/结冰状态（用于轮胎摩擦）。
    pub fn ground_conditions(&self, sat_c: f64) -> GroundConditions {
        let p = if self.precip == PrecipKind::None { 0.0 } else { self.precip_intensity };
        match self.precip {
            PrecipKind::Rain => GroundConditions {
                wetness: (p * 4.0).clamp(0.0, 1.0),
                icing: if sat_c < 0.0 { (p * 4.0).clamp(0.0, 1.0) } else { 0.0 },
            },
            PrecipKind::Snow => GroundConditions {
                wetness: (p * 3.0).clamp(0.0, 1.0),
                icing: if sat_c < 1.0 { (p * 3.0).clamp(0.0, 1.0) } else { 0.0 },
            },
            PrecipKind::None => GroundConditions {
                wetness: 0.0,
                icing: 0.0,
            },
        }
    }
}

/// 结冰输入。
#[derive(Clone, Copy, Debug)]
pub struct IcingInputs {
    pub in_cloud: bool,
    pub precip: PrecipKind,
    pub precip_intensity: f64,
    /// 静温 SAT（℃）。
    pub sat_c: f64,
    pub tas: f64,
    /// 除冰系统已开启且机型配备且引擎提供功率。
    pub deice_active: bool,
    pub pitot_heat: bool,
}

/// 结冰条件强度（0 = 无结冰条件）。
/// 触发条件：处于云层或降水中，且 SAT ∈ [-20℃, +2℃]。
pub fn icing_intensity(inp: &IcingInputs) -> f64 {
    if inp.sat_c < -20.0 || inp.sat_c > 2.0 {
        return 0.0;
    }
    let moisture = {
        let m_cloud: f64 = if inp.in_cloud { 1.0 } else { 0.0 };
        let m_rain = if inp.precip == PrecipKind::Rain { 0.5 + 0.5 * inp.precip_intensity } else { 0.0 };
        let m_snow = if inp.precip == PrecipKind::Snow { 0.25 } else { 0.0 };
        m_cloud.max(m_rain).max(m_snow)
    };
    if moisture <= 0.0 {
        return 0.0;
    }
    let tf = (1.0 - ((inp.sat_c + 9.0) / 12.0).powi(2)).clamp(0.15, 1.0);
    moisture * tf
}

/// 推进结冰状态：`ice_load`（0..1）、皮托管冻结计时。
pub fn update_ice(ice_load: &mut f64, pitot_blocked: &mut bool, pitot_timer: &mut f64, inp: &IcingInputs, dt: f64) {
    let intensity = icing_intensity(inp);
    if intensity > 0.0 {
        let rate = intensity * (inp.tas / 60.0).clamp(0.4, 2.0) / 240.0;
        *ice_load += rate * dt;
    }
    if inp.deice_active {
        *ice_load -= dt / 45.0;
    }
    if inp.sat_c > 2.0 {
        // 融化
        *ice_load -= dt / 150.0 * (1.0 + (inp.sat_c - 2.0) / 5.0);
    }
    *ice_load = ice_load.clamp(0.0, 1.0);

    if inp.pitot_heat {
        *pitot_timer = (*pitot_timer - 2.0 * dt).max(0.0);
        if *pitot_timer <= 0.0 {
            *pitot_blocked = false;
        }
    } else if intensity > 0.0 {
        *pitot_timer += dt;
        if *pitot_timer > 15.0 {
            *pitot_blocked = true;
        }
    } else if inp.sat_c > 4.0 {
        *pitot_timer = (*pitot_timer - dt).max(0.0);
        if *pitot_timer <= 0.0 {
            *pitot_blocked = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inp(sat: f64, cloud: bool) -> IcingInputs {
        IcingInputs {
            in_cloud: cloud,
            precip: PrecipKind::None,
            precip_intensity: 0.0,
            sat_c: sat,
            tas: 60.0,
            deice_active: false,
            pitot_heat: false,
        }
    }

    #[test]
    fn icing_only_in_cloud_and_temp_window() {
        assert!(icing_intensity(&inp(-8.0, true)) > 0.9);
        assert_eq!(icing_intensity(&inp(-8.0, false)), 0.0);
        assert_eq!(icing_intensity(&inp(-25.0, true)), 0.0);
        assert_eq!(icing_intensity(&inp(5.0, true)), 0.0);
    }

    #[test]
    fn ice_accumulates_pitot_freezes_deice_clears() {
        let (mut ice, mut blocked, mut timer) = (0.0, false, 0.0);
        let dt = 1.0 / 120.0;
        for _ in 0..(120 * 120) {
            update_ice(&mut ice, &mut blocked, &mut timer, &inp(-8.0, true), dt);
        }
        assert!(ice > 0.3, "ice {ice}");
        assert!(blocked);
        let mut i2 = inp(-8.0, true);
        i2.pitot_heat = true;
        i2.deice_active = true;
        for _ in 0..(120 * 120) {
            update_ice(&mut ice, &mut blocked, &mut timer, &i2, dt);
        }
        assert!(!blocked);
        assert!(ice < 0.05, "ice {ice}");
    }

    #[test]
    fn wind_profile_and_turbulence() {
        let mut w = WeatherState::calm();
        w.set_surface_wind(270.0, 10.0); // 西风 → 吹向东 (+X)
        let v = w.wind_at(DVec3::new(0.0, 10.0, 0.0), 10.0, 0.0);
        assert!(v.x > 9.0 && v.z.abs() < 1e-6);
        w.turbulence = 0.8;
        let a = w.wind_at(DVec3::new(100.0, 50.0, 0.0), 50.0, 3.0);
        let b = w.wind_at(DVec3::new(100.0, 50.0, 0.0), 50.0, 3.0);
        assert_eq!(a, b);
        assert!((a - v).length() > 0.1);
    }

    #[test]
    fn cloud_and_visibility() {
        let mut w = WeatherState::calm();
        w.clouds.push(CloudLayer { base: 1000.0, top: 2000.0, coverage: 1.0 });
        assert!(w.in_cloud(DVec3::new(0.0, 1500.0, 0.0)));
        assert!(!w.in_cloud(DVec3::new(0.0, 500.0, 0.0)));
        assert!(w.visibility(DVec3::new(0.0, 1500.0, 0.0), 0.0) < 500.0);
        assert!(w.visibility(DVec3::new(0.0, 500.0, 0.0), 0.0) > 10_000.0);
    }
}
