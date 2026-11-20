//! 气候：纬度 + 日期(季节) + 海拔 → 温度 / 湿度 / 盛行风 / 雪线 / 生物群系。
//!
//! 南北半球季节严格相反：季节项由太阳直射点赤纬 δ 驱动（滞后约 30 天）。

use std::f64::consts::PI;

/// 太阳直射点赤纬（度）：δ = 23.44°·sin(2π(day−81)/365)。
pub fn solar_declination(day_of_year: f64) -> f64 {
    23.44 * (2.0 * PI * (day_of_year - 81.0) / 365.0).sin()
}

/// 年均海平面温度（℃）随纬度。
pub fn mean_sea_level_temp(lat: f64) -> f64 {
    -25.0 + 53.0 * lat.to_radians().cos().max(0.0).powf(1.4)
}

/// 海平面温度（℃）：年均 + 半球相反的季节项（海洋季节幅度较小）。
pub fn sea_level_temp(lat: f64, day_of_year: f64, ocean: bool) -> f64 {
    let amp = 19.0 * lat.abs().to_radians().sin().powf(1.15) * if ocean { 0.55 } else { 1.0 };
    // 地面温度滞后直射点约 30 天
    let decl = solar_declination(day_of_year - 30.0);
    let season = (decl / 23.44) * lat.signum();
    mean_sea_level_temp(lat) + amp * season
}

/// 海拔 `alt`（m）处的温度（℃），标准递减率 6.5 K/km。
pub fn temperature_at(lat: f64, day_of_year: f64, alt: f64, ocean: bool) -> f64 {
    sea_level_temp(lat, day_of_year, ocean) - 0.0065 * alt.max(0.0)
}

/// 相对于 ISA 的海平面温度偏移（供大气模型使用）。
pub fn isa_delta_t(lat: f64, day_of_year: f64, ocean: bool) -> f64 {
    sea_level_temp(lat, day_of_year, ocean) - 15.0
}

/// 雪线海拔（m）：0℃ 等温线高度。海平面温度 ≤ 0℃ 时雪线为 0。
pub fn snow_line(lat: f64, day_of_year: f64) -> f64 {
    let t = sea_level_temp(lat, day_of_year, false);
    (t / 0.0065).max(0.0)
}

/// 基础湿度 0..1（赤道辐合带湿、副热带干、中纬度较湿），`noise` ∈ [-1,1] 为局地扰动。
pub fn humidity(lat: f64, noise: f64) -> f64 {
    let a = lat.abs();
    let itcz = (-((a / 9.0).powi(2))).exp() * 0.55;
    let subtropical_dry = (-(((a - 27.0) / 9.0).powi(2))).exp() * 0.55;
    let midlat_wet = (-(((a - 52.0) / 12.0).powi(2))).exp() * 0.25;
    (0.42 + itcz - subtropical_dry + midlat_wet + 0.32 * noise).clamp(0.0, 1.0)
}

/// 盛行风（来向°，风速 m/s）：信风 / 西风带 / 极地东风。
pub fn prevailing_wind(lat: f64) -> (f64, f64) {
    let a = lat.abs();
    let north = lat >= 0.0;
    // 来向（北半球）：信风 ENE(60°)，西风带 WSW(250°)，极地东风 NE(45°)
    let (from_n, speed) = if a < 28.0 {
        (60.0, 6.0)
    } else if a < 33.0 {
        // 副热带无风带 → 过渡
        (60.0 + (a - 28.0) / 5.0 * 190.0, 3.0)
    } else if a < 58.0 {
        (250.0, 10.0)
    } else if a < 63.0 {
        (250.0 + (a - 58.0) / 5.0 * 155.0, 4.0)
    } else {
        (45.0, 6.0)
    };
    let from = if north { from_n } else { (180.0 - from_n).rem_euclid(360.0) };
    (from.rem_euclid(360.0), speed)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Biome {
    DeepOcean,
    ShallowOcean,
    Beach,
    Desert,
    Grassland,
    TemperateForest,
    Conifer,
    Tundra,
    BareRock,
    Snow,
    IceCap,
}

#[derive(Clone, Copy, Debug)]
pub struct ClimateSample {
    pub temp_sl: f64,
    pub temp: f64,
    pub humidity: f64,
    pub snow_line: f64,
}

/// 生物群系分类。`height` 为地形高度（m，负为海底），`slope` 为坡度（rad）。
pub fn classify(height: f64, slope: f64, temp_sl: f64, humidity: f64, snow_line: f64) -> Biome {
    if height <= 0.0 {
        if temp_sl < -12.0 {
            return Biome::IceCap;
        }
        return if height < -250.0 { Biome::DeepOcean } else { Biome::ShallowOcean };
    }
    let temp = temp_sl - 0.0065 * height;
    if temp_sl < -10.0 {
        return Biome::IceCap;
    }
    if height < 6.0 && temp > 5.0 {
        return Biome::Beach;
    }
    if height > snow_line + 60.0 && slope < 0.95 {
        return Biome::Snow;
    }
    if height > snow_line - 250.0 && slope > 0.75 {
        return Biome::BareRock;
    }
    if slope > 1.05 {
        return Biome::BareRock;
    }
    if temp < -3.0 {
        return Biome::Snow;
    }
    if temp < 3.0 {
        return Biome::Tundra;
    }
    if temp < 9.0 {
        return if humidity > 0.38 { Biome::Conifer } else { Biome::Tundra };
    }
    if humidity < 0.28 {
        return Biome::Desert;
    }
    if humidity < 0.45 {
        return Biome::Grassland;
    }
    if temp < 15.0 {
        Biome::Conifer
    } else {
        Biome::TemperateForest
    }
}

/// 生物群系基础顶点色（RGB 0..1）。
pub fn biome_color(b: Biome) -> [f32; 3] {
    match b {
        Biome::DeepOcean => [0.02, 0.10, 0.28],
        Biome::ShallowOcean => [0.05, 0.28, 0.45],
        Biome::Beach => [0.78, 0.72, 0.52],
        Biome::Desert => [0.80, 0.66, 0.38],
        Biome::Grassland => [0.45, 0.62, 0.25],
        Biome::TemperateForest => [0.14, 0.40, 0.16],
        Biome::Conifer => [0.10, 0.30, 0.20],
        Biome::Tundra => [0.50, 0.52, 0.38],
        Biome::BareRock => [0.42, 0.38, 0.35],
        Biome::Snow => [0.93, 0.95, 0.98],
        Biome::IceCap => [0.82, 0.90, 0.96],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn declination_extremes() {
        assert!((solar_declination(172.0) - 23.44).abs() < 0.1);
        assert!((solar_declination(355.0) + 23.44).abs() < 0.5);
        assert!(solar_declination(81.0).abs() < 1e-9);
    }

    #[test]
    fn hemispheres_have_opposite_seasons() {
        let (jan, jul) = (15.0, 196.0);
        for lat in [30.0, 45.0, 60.0] {
            let n_jan = sea_level_temp(lat, jan, false);
            let n_jul = sea_level_temp(lat, jul, false);
            let s_jan = sea_level_temp(-lat, jan, false);
            let s_jul = sea_level_temp(-lat, jul, false);
            assert!(n_jul > n_jan + 5.0, "north summer warmer at {lat}");
            assert!(s_jan > s_jul + 5.0, "south summer warmer at {lat}");
            // 同日期南北半球相反
            assert!(n_jan < s_jan && n_jul > s_jul);
        }
    }

    #[test]
    fn snow_line_drops_in_winter() {
        let lat = 45.0;
        assert!(snow_line(lat, 196.0) > snow_line(lat, 15.0) + 800.0);
        assert!(snow_line(-lat, 15.0) > snow_line(-lat, 196.0) + 800.0);
        assert_eq!(snow_line(80.0, 15.0), 0.0);
    }

    #[test]
    fn winds_by_band_and_hemisphere() {
        let (f, _) = prevailing_wind(45.0);
        assert!((f - 250.0).abs() < 1.0); // 西风带
        let (f, _) = prevailing_wind(10.0);
        assert!(f > 30.0 && f < 100.0); // 东北信风
        let (fs, _) = prevailing_wind(-10.0);
        assert!(fs > 80.0 && fs < 150.0); // 东南信风
    }

    #[test]
    fn biomes() {
        assert_eq!(classify(-1000.0, 0.0, 20.0, 0.5, 4000.0), Biome::DeepOcean);
        assert_eq!(classify(300.0, 0.05, 28.0, 0.1, 5000.0), Biome::Desert);
        assert_eq!(classify(4800.0, 0.3, 15.0, 0.5, 3500.0), Biome::Snow);
        assert_eq!(classify(100.0, 0.05, -20.0, 0.5, 0.0), Biome::IceCap);
    }
}
