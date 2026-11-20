//! 1:32 经纬度 <-> 世界米坐标（经度无缝环绕）。
//!
//! * 水平比例 1:32：赤道周长 `WORLD_WIDTH = 1_250_000 m`，南北跨度 `WORLD_HEIGHT = 625_000 m`；
//! * 垂直比例 1:1：`Y = 0` 为海平面。
//! * 世界系：`+X` 东，`-Z` 北（`z = -lat/180·H`）。`x` 在 `[-W/2, W/2)` 内循环。

use glam::DVec3;

pub const WORLD_WIDTH: f64 = 1_250_000.0;
pub const WORLD_HEIGHT: f64 = 625_000.0;
pub const HORIZONTAL_SCALE: f64 = 32.0;

/// 把 `x` 归一化到 `[-W/2, W/2)`。
#[inline]
pub fn wrap_x(x: f64) -> f64 {
    (x + WORLD_WIDTH / 2.0).rem_euclid(WORLD_WIDTH) - WORLD_WIDTH / 2.0
}

/// 沿环绕经度方向 `from → to` 的最短带符号位移。
#[inline]
pub fn wrapped_dx(from: f64, to: f64) -> f64 {
    wrap_x(to - from)
}

#[inline]
pub fn clamp_z(z: f64) -> f64 {
    z.clamp(-WORLD_HEIGHT / 2.0, WORLD_HEIGHT / 2.0)
}

/// 世界坐标 → (纬度°, 经度°)。
pub fn world_to_latlon(x: f64, z: f64) -> (f64, f64) {
    let lon = wrap_x(x) / WORLD_WIDTH * 360.0;
    let lat = -clamp_z(z) / WORLD_HEIGHT * 180.0;
    (lat, lon)
}

/// (纬度°, 经度°) → 世界坐标 (x, z)。
pub fn latlon_to_world(lat: f64, lon: f64) -> (f64, f64) {
    let x = wrap_x(lon / 360.0 * WORLD_WIDTH);
    let z = -lat.clamp(-90.0, 90.0) / 180.0 * WORLD_HEIGHT;
    (x, z)
}

/// 经纬度 → 单位球面点（用于无缝、无极点畸变的 3D 噪声）。
pub fn sphere_point(lat_deg: f64, lon_deg: f64) -> DVec3 {
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    DVec3::new(la.cos() * lo.cos(), la.sin(), la.cos() * lo.sin())
}

pub fn world_to_sphere(x: f64, z: f64) -> DVec3 {
    let (lat, lon) = world_to_latlon(x, z);
    sphere_point(lat, lon)
}

/// 世界水平距离（考虑 X 环绕）。
pub fn wrapped_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = wrapped_dx(a.0, b.0);
    let dz = b.1 - a.1;
    dx.hypot(dz)
}

/// 航向（度，0 = 北，顺时针）→ 世界系水平单位方向 (dx, dz)。
pub fn heading_dir(heading_deg: f64) -> (f64, f64) {
    let h = heading_deg.to_radians();
    (h.sin(), -h.cos())
}

/// 世界系方向 (dx, dz) → 航向°[0,360)。
pub fn dir_heading(dx: f64, dz: f64) -> f64 {
    let mut h = dx.atan2(-dz).to_degrees();
    if h < 0.0 {
        h += 360.0;
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip_and_wrap() {
        for &(lat, lon) in &[(0.0, 0.0), (45.0, 100.0), (-60.0, -170.0), (80.0, 179.0)] {
            let (x, z) = latlon_to_world(lat, lon);
            let (la, lo) = world_to_latlon(x, z);
            assert!((la - lat).abs() < 1e-9 && (lo - lon).abs() < 1e-9);
        }
        assert!((wrap_x(WORLD_WIDTH / 2.0 + 10.0) - (-WORLD_WIDTH / 2.0 + 10.0)).abs() < 1e-6);
        assert!((wrapped_dx(WORLD_WIDTH / 2.0 - 5.0, -WORLD_WIDTH / 2.0 + 5.0) - 10.0).abs() < 1e-6);
    }
    #[test]
    fn scale_is_1_to_32() {
        // 地球赤道 40075 km / 32 ≈ 1.252e6 m
        assert!((40_075_000.0 / HORIZONTAL_SCALE - WORLD_WIDTH).abs() < 5_000.0);
        assert!((WORLD_WIDTH / WORLD_HEIGHT - 2.0).abs() < 1e-12);
    }
    #[test]
    fn sphere_is_seamless() {
        let a = world_to_sphere(WORLD_WIDTH / 2.0 - 0.001, 1000.0);
        let b = world_to_sphere(-WORLD_WIDTH / 2.0 + 0.001, 1000.0);
        assert!((a - b).length() < 1e-6);
    }
    #[test]
    fn heading_roundtrip() {
        for h in [0.0, 45.0, 90.0, 200.0, 359.0] {
            let (dx, dz) = heading_dir(h);
            assert!((dir_heading(dx, dz) - h).abs() < 1e-9);
        }
    }
}
