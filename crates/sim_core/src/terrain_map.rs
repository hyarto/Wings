//! 地形雷达图（等高线）与纵剖面的纯计算部分：高程网格、相对高度分级、等高线提取。
//! 绘制（像素/颜色）在 `app_client::terrain_radar`。
//!
//! 坐标：`x` 东、`z` 南（`-z` 北）。航向 0° = 北 = `-z`，90° = 东 = `+x`。

/// 世界对齐的方形高程网格，可分帧增量构建（避免一帧内采样过多）。
#[derive(Clone, Debug)]
pub struct TerrainGrid {
    pub cx: f64,
    pub cz: f64,
    /// 半边长（m）。
    pub half: f64,
    pub n: usize,
    data: Vec<f32>,
    rows_done: usize,
}

impl TerrainGrid {
    pub fn new(cx: f64, cz: f64, half: f64, n: usize) -> Self {
        let n = n.max(2);
        Self { cx, cz, half, n, data: vec![0.0; n * n], rows_done: 0 }
    }

    pub fn step_m(&self) -> f64 {
        2.0 * self.half / (self.n as f64 - 1.0)
    }

    pub fn is_complete(&self) -> bool {
        self.rows_done >= self.n
    }

    /// 再构建最多 `max_rows` 行；完成返回 true。
    pub fn build_rows(&mut self, height: &dyn Fn(f64, f64) -> f64, max_rows: usize) -> bool {
        let step = self.step_m();
        let mut k = 0;
        while self.rows_done < self.n && k < max_rows {
            let j = self.rows_done;
            let z = self.cz - self.half + j as f64 * step;
            for i in 0..self.n {
                let x = self.cx - self.half + i as f64 * step;
                self.data[j * self.n + i] = height(x, z) as f32;
            }
            self.rows_done += 1;
            k += 1;
        }
        self.is_complete()
    }

    /// 一次性构建完整网格。
    pub fn build_all(cx: f64, cz: f64, half: f64, n: usize, height: &dyn Fn(f64, f64) -> f64) -> Self {
        let mut g = Self::new(cx, cz, half, n);
        g.build_rows(height, usize::MAX);
        g
    }

    /// 点 (x, z) 到网格边缘的最小距离（负 = 在网格之外）。
    pub fn margin(&self, x: f64, z: f64) -> f64 {
        self.half - (x - self.cx).abs().max((z - self.cz).abs())
    }

    /// 双线性采样；网格外或尚未构建的行返回 `None`。
    pub fn sample(&self, x: f64, z: f64) -> Option<f32> {
        let step = self.step_m();
        let fx = (x - (self.cx - self.half)) / step;
        let fz = (z - (self.cz - self.half)) / step;
        if fx < 0.0 || fz < 0.0 || fx > (self.n - 1) as f64 || fz > (self.n - 1) as f64 {
            return None;
        }
        let (i0, j0) = ((fx.floor() as usize).min(self.n - 2), (fz.floor() as usize).min(self.n - 2));
        if j0 + 1 >= self.rows_done {
            return None;
        }
        let (tx, tz) = ((fx - i0 as f64) as f32, (fz - j0 as f64) as f32);
        let g = |i: usize, j: usize| self.data[j * self.n + i];
        let a = g(i0, j0) * (1.0 - tx) + g(i0 + 1, j0) * tx;
        let b = g(i0, j0 + 1) * (1.0 - tx) + g(i0 + 1, j0 + 1) * tx;
        Some(a * (1.0 - tz) + b * tz)
    }
}

/// 地形相对本机海拔的分级（TAWS 风格）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Band {
    /// 海面/水域。
    Water,
    /// 高于或接近本机高度（≥ 本机 − 30 m）：红。
    Hazard,
    /// 本机下方 30–300 m：琥珀。
    Caution,
    /// 本机下方 300–800 m：绿。
    Near,
    /// 更低：暗绿。
    Far,
}

pub fn classify(elev: f32, alt: f64) -> Band {
    let e = elev as f64;
    if e <= 0.0 {
        // 海面除非飞机贴近海面，否则不算威胁
        if alt < 30.0 {
            return Band::Hazard;
        }
        return Band::Water;
    }
    let d = alt - e;
    if d <= 30.0 {
        Band::Hazard
    } else if d <= 300.0 {
        Band::Caution
    } else if d <= 800.0 {
        Band::Near
    } else {
        Band::Far
    }
}

/// 等高线间隔（m）随量程自适应。
pub fn contour_interval(range_m: f64) -> f64 {
    if range_m <= 6_000.0 {
        25.0
    } else if range_m <= 12_000.0 {
        50.0
    } else if range_m <= 24_000.0 {
        100.0
    } else {
        200.0
    }
}

/// 相邻两点之间是否跨越等高线：`Some(false)` 普通线，`Some(true)` 计曲线（每 5 条加粗）。
pub fn contour_between(e0: f32, e1: f32, interval: f64) -> Option<bool> {
    if e0 <= 0.0 && e1 <= 0.0 {
        return None;
    }
    let k0 = (e0 as f64 / interval).floor() as i64;
    let k1 = (e1 as f64 / interval).floor() as i64;
    if k0 == k1 {
        return None;
    }
    let k = k0.max(k1);
    Some(k.rem_euclid(5) == 0)
}

/// 雷达图一个像素的结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    pub band: Band,
    /// 0 无 / 1 等高线 / 2 计曲线。
    pub contour: u8,
}

/// 渲染航向向上的圆形地形图：直径 `2*radius+1` 像素，圆外 `None`。
/// 圆半径 `radius` 像素对应 `range_m` 米。
pub fn render_cells(grid: &TerrainGrid, pos_x: f64, pos_z: f64, heading_deg: f64, alt: f64, range_m: f64, radius: i32) -> Vec<Option<Cell>> {
    let d = (2 * radius + 1) as usize;
    let mpp = range_m / radius as f64;
    let h = heading_deg.to_radians();
    let (fx, fz) = (h.sin(), -h.cos());
    let (rx, rz) = (h.cos(), h.sin());
    let mut elev: Vec<Option<f32>> = vec![None; d * d];
    for py in -radius..=radius {
        for px in -radius..=radius {
            if px * px + py * py > radius * radius {
                continue;
            }
            let right = px as f64 * mpp;
            let fwd = -(py as f64) * mpp;
            let (x, z) = (pos_x + fx * fwd + rx * right, pos_z + fz * fwd + rz * right);
            elev[(py + radius) as usize * d + (px + radius) as usize] = grid.sample(x, z);
        }
    }
    let interval = contour_interval(range_m);
    let mut out = vec![None; d * d];
    for y in 0..d {
        for x in 0..d {
            let Some(e) = elev[y * d + x] else { continue };
            let mut contour = 0u8;
            for (nx, ny) in [(x + 1, y), (x, y + 1), (x.wrapping_sub(1), y), (x, y.wrapping_sub(1))] {
                if nx >= d || ny >= d {
                    continue;
                }
                if let Some(e2) = elev[ny * d + nx] {
                    if let Some(idx) = contour_between(e, e2, interval) {
                        // 只在较高一侧画线，线条宽度恒为 1 像素
                        if e >= e2 {
                            contour = contour.max(if idx { 2 } else { 1 });
                        }
                    }
                }
            }
            out[y * d + x] = Some(Cell { band: classify(e, alt), contour });
        }
    }
    out
}

/// 沿航向的纵剖面：`n` 个采样，距离 0..range_m，取航迹两侧 ±(150+0.1d)（上限 1500 m）内的最大地形高度（海面记 0）。
pub fn profile(height: &dyn Fn(f64, f64) -> f64, pos_x: f64, pos_z: f64, heading_deg: f64, range_m: f64, n: usize) -> Vec<f32> {
    let h = heading_deg.to_radians();
    let (fx, fz) = (h.sin(), -h.cos());
    let (rx, rz) = (h.cos(), h.sin());
    (0..n)
        .map(|i| {
            let d = range_m * i as f64 / (n.max(2) - 1) as f64;
            let w = (150.0 + 0.1 * d).min(1500.0);
            let mut m = 0.0f64;
            for k in [-1.0, 0.0, 1.0] {
                let x = pos_x + fx * d + rx * w * k;
                let z = pos_z + fz * d + rz * w * k;
                m = m.max(height(x, z));
            }
            m as f32
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 东西向山脊：高度 = 20·|x|/100 起的斜坡（x 越大越高），便于验证朝向与等高线。
    fn slope_x(x: f64, _z: f64) -> f64 {
        (x * 0.1).max(0.0)
    }

    #[test]
    fn grid_bilinear_matches_function_and_is_incremental() {
        let mut g = TerrainGrid::new(0.0, 0.0, 5000.0, 101);
        assert!(g.sample(0.0, 0.0).is_none(), "nothing built yet");
        assert!(!g.build_rows(&slope_x, 30));
        assert!(!g.is_complete());
        assert!(g.build_rows(&slope_x, 1000));
        let e = g.sample(1234.0, -777.0).unwrap() as f64;
        assert!((e - 123.4).abs() < 0.5, "{e}");
        assert!(g.sample(6000.0, 0.0).is_none(), "outside");
        assert!(g.margin(0.0, 0.0) > 4999.0 && g.margin(5500.0, 0.0) < 0.0);
    }

    #[test]
    fn classify_bands_relative_to_aircraft_altitude() {
        assert_eq!(classify(1000.0, 1000.0), Band::Hazard);
        assert_eq!(classify(975.0, 1000.0), Band::Hazard);
        assert_eq!(classify(900.0, 1000.0), Band::Caution);
        assert_eq!(classify(800.0, 1000.0), Band::Caution);
        assert_eq!(classify(500.0, 1000.0), Band::Near);
        assert_eq!(classify(100.0, 1000.0), Band::Far);
        assert_eq!(classify(-3.0, 1000.0), Band::Water);
    }

    #[test]
    fn contours_cross_at_interval_and_mark_index_lines() {
        assert_eq!(contour_between(95.0, 105.0, 100.0), Some(false));
        assert_eq!(contour_between(105.0, 195.0, 100.0), None);
        // 500 m 处为计曲线
        assert_eq!(contour_between(495.0, 505.0, 100.0), Some(true));
        assert_eq!(contour_between(-5.0, -1.0, 100.0), None, "no contours on water");
        assert!(contour_interval(5_000.0) < contour_interval(40_000.0));
    }

    #[test]
    fn render_is_heading_up_and_draws_contours() {
        let g = TerrainGrid::build_all(0.0, 0.0, 30_000.0, 301, &slope_x);
        let r = 30;
        let d = (2 * r + 1) as usize;
        let at = |cells: &Vec<Option<Cell>>, px: i32, py: i32| cells[(py + r) as usize * d + (px + r) as usize];
        // 朝北：东侧（屏幕右）更高 → 右边是更危险的等级
        let n = render_cells(&g, 0.0, 0.0, 0.0, 1500.0, 20_000.0, r);
        let (l, rr) = (at(&n, -25, 0).unwrap().band, at(&n, 25, 0).unwrap().band);
        assert_eq!(l, Band::Water);
        assert_ne!(rr, Band::Water);
        // 朝南：东在屏幕左侧
        let s = render_cells(&g, 0.0, 0.0, 180.0, 1500.0, 20_000.0, r);
        assert_eq!(at(&s, 25, 0).unwrap().band, Band::Water);
        assert_ne!(at(&s, -25, 0).unwrap().band, Band::Water);
        // 圆外无数据
        assert!(at(&n, -30, -30).is_none());
        // 20 km 量程 100 m 间隔，坡度 0.1 → 每 1 km 一条等高线，应能找到
        let lines = n.iter().flatten().filter(|c| c.contour > 0).count();
        assert!(lines > 30, "contour pixels {lines}");
    }

    #[test]
    fn profile_follows_heading_and_takes_lateral_max() {
        // 朝东：高度随距离升高
        let p = profile(&slope_x, 0.0, 0.0, 90.0, 10_000.0, 11);
        assert!(p[0] < 1.0 && (p[10] - 1000.0).abs() < 1.0, "{p:?}");
        // 朝西：全是 0
        let w = profile(&slope_x, 0.0, 0.0, 270.0, 10_000.0, 11);
        assert!(w.iter().all(|v| *v < 0.01));
        // 侧向取最大：朝北飞，东侧 ±150 m 内的山被计入
        let side = profile(&slope_x, 0.0, 0.0, 0.0, 10_000.0, 5);
        assert!(side[0] >= 14.0, "{side:?}");
    }
}
