//! 320×160 原生像素缓冲仪表盘 + 4×6 点阵字体 + HUD 模式 + 复古航图。
//! 全部为纯 CPU 绘制函数（不依赖 Bevy），可直接单元测试。

use crate::pixel_pipeline::{COCKPIT_VIEW_H, H, W};

pub type Rgba = [u8; 4];
pub const TRANSPARENT: Rgba = [0, 0, 0, 0];
pub const BLACK: Rgba = [0, 0, 0, 255];
pub const WHITE: Rgba = [240, 240, 240, 255];
pub const GRAY: Rgba = [120, 120, 128, 255];
pub const PANEL: Rgba = [44, 44, 52, 255];
pub const PANEL_DK: Rgba = [26, 26, 32, 255];
pub const SKY: Rgba = [64, 120, 216, 255];
pub const GROUND: Rgba = [136, 88, 40, 255];
pub const RED: Rgba = [232, 40, 40, 255];
pub const AMBER: Rgba = [255, 176, 0, 255];
pub const GREEN: Rgba = [64, 255, 96, 255];
pub const YELLOW: Rgba = [255, 232, 40, 255];
pub const CYAN: Rgba = [80, 224, 255, 255];
pub const MAGENTA: Rgba = [232, 80, 232, 255];

/// 4×6 点阵字体：每行 4 位（高位在左）。
const GLYPHS: &[(char, [u8; 6])] = &[
    ('0', [0x6, 0x9, 0x9, 0x9, 0x9, 0x6]),
    ('1', [0x2, 0x6, 0x2, 0x2, 0x2, 0x7]),
    ('2', [0x6, 0x9, 0x1, 0x2, 0x4, 0xF]),
    ('3', [0x6, 0x9, 0x2, 0x1, 0x9, 0x6]),
    ('4', [0x1, 0x3, 0x5, 0x9, 0xF, 0x1]),
    ('5', [0xF, 0x8, 0xE, 0x1, 0x9, 0x6]),
    ('6', [0x6, 0x8, 0xE, 0x9, 0x9, 0x6]),
    ('7', [0xF, 0x1, 0x2, 0x4, 0x4, 0x4]),
    ('8', [0x6, 0x9, 0x6, 0x9, 0x9, 0x6]),
    ('9', [0x6, 0x9, 0x7, 0x1, 0x9, 0x6]),
    ('A', [0x6, 0x9, 0x9, 0xF, 0x9, 0x9]),
    ('B', [0xE, 0x9, 0xE, 0x9, 0x9, 0xE]),
    ('C', [0x7, 0x8, 0x8, 0x8, 0x8, 0x7]),
    ('D', [0xE, 0x9, 0x9, 0x9, 0x9, 0xE]),
    ('E', [0xF, 0x8, 0xE, 0x8, 0x8, 0xF]),
    ('F', [0xF, 0x8, 0xE, 0x8, 0x8, 0x8]),
    ('G', [0x7, 0x8, 0xB, 0x9, 0x9, 0x7]),
    ('H', [0x9, 0x9, 0xF, 0x9, 0x9, 0x9]),
    ('I', [0x7, 0x2, 0x2, 0x2, 0x2, 0x7]),
    ('J', [0x1, 0x1, 0x1, 0x1, 0x9, 0x6]),
    ('K', [0x9, 0xA, 0xC, 0xA, 0x9, 0x9]),
    ('L', [0x8, 0x8, 0x8, 0x8, 0x8, 0xF]),
    ('M', [0x9, 0xF, 0xF, 0x9, 0x9, 0x9]),
    ('N', [0x9, 0xD, 0xF, 0xB, 0x9, 0x9]),
    ('O', [0x6, 0x9, 0x9, 0x9, 0x9, 0x6]),
    ('P', [0xE, 0x9, 0x9, 0xE, 0x8, 0x8]),
    ('Q', [0x6, 0x9, 0x9, 0x9, 0xB, 0x7]),
    ('R', [0xE, 0x9, 0x9, 0xE, 0xA, 0x9]),
    ('S', [0x7, 0x8, 0x6, 0x1, 0x1, 0xE]),
    ('T', [0xF, 0x2, 0x2, 0x2, 0x2, 0x2]),
    ('U', [0x9, 0x9, 0x9, 0x9, 0x9, 0x6]),
    ('V', [0x9, 0x9, 0x9, 0x9, 0x6, 0x6]),
    ('W', [0x9, 0x9, 0x9, 0xF, 0xF, 0x9]),
    ('X', [0x9, 0x9, 0x6, 0x6, 0x9, 0x9]),
    ('Y', [0x9, 0x9, 0x6, 0x2, 0x2, 0x2]),
    ('Z', [0xF, 0x1, 0x2, 0x4, 0x8, 0xF]),
    ('-', [0x0, 0x0, 0xF, 0x0, 0x0, 0x0]),
    ('+', [0x0, 0x2, 0x7, 0x2, 0x0, 0x0]),
    ('.', [0x0, 0x0, 0x0, 0x0, 0x0, 0x4]),
    (':', [0x0, 0x4, 0x0, 0x0, 0x4, 0x0]),
    ('/', [0x1, 0x1, 0x2, 0x4, 0x8, 0x8]),
    ('%', [0x9, 0x1, 0x2, 0x4, 0x8, 0x9]),
    ('<', [0x1, 0x2, 0x4, 0x4, 0x2, 0x1]),
    ('>', [0x8, 0x4, 0x2, 0x2, 0x4, 0x8]),
    ('*', [0x0, 0x9, 0x6, 0xF, 0x6, 0x9]),
    ('^', [0x2, 0x5, 0x0, 0x0, 0x0, 0x0]),
    ('=', [0x0, 0xF, 0x0, 0xF, 0x0, 0x0]),
    ('#', [0x5, 0xF, 0x5, 0xF, 0x5, 0x0]),
    ('?', [0x6, 0x9, 0x2, 0x4, 0x0, 0x4]),
    (' ', [0, 0, 0, 0, 0, 0]),
];

pub fn glyph(c: char) -> Option<&'static [u8; 6]> {
    let c = c.to_ascii_uppercase();
    GLYPHS.iter().find(|(g, _)| *g == c).map(|(_, b)| b)
}

/// 原生 320×160 RGBA 像素缓冲。
#[derive(Clone)]
pub struct PixelBuffer {
    pub data: Vec<u8>,
}

impl Default for PixelBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl PixelBuffer {
    pub fn new() -> Self {
        Self { data: vec![0; (W * H * 4) as usize] }
    }
    pub fn clear(&mut self, c: Rgba) {
        for p in self.data.chunks_exact_mut(4) {
            p.copy_from_slice(&c);
        }
    }
    #[inline]
    pub fn set(&mut self, x: i32, y: i32, c: Rgba) {
        if x < 0 || y < 0 || x >= W as i32 || y >= H as i32 {
            return;
        }
        let i = ((y as u32 * W + x as u32) * 4) as usize;
        self.data[i..i + 4].copy_from_slice(&c);
    }
    pub fn get(&self, x: i32, y: i32) -> Rgba {
        if x < 0 || y < 0 || x >= W as i32 || y >= H as i32 {
            return TRANSPARENT;
        }
        let i = ((y as u32 * W + x as u32) * 4) as usize;
        [self.data[i], self.data[i + 1], self.data[i + 2], self.data[i + 3]]
    }
    pub fn rect(&mut self, x: i32, y: i32, w: i32, h: i32, c: Rgba) {
        for yy in y..y + h {
            for xx in x..x + w {
                self.set(xx, yy, c);
            }
        }
    }
    pub fn frame(&mut self, x: i32, y: i32, w: i32, h: i32, c: Rgba) {
        for xx in x..x + w {
            self.set(xx, y, c);
            self.set(xx, y + h - 1, c);
        }
        for yy in y..y + h {
            self.set(x, yy, c);
            self.set(x + w - 1, yy, c);
        }
    }
    pub fn line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, c: Rgba) {
        let (mut x, mut y) = (x0, y0);
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.set(x, y, c);
            if x == x1 && y == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }
    pub fn disc(&mut self, cx: i32, cy: i32, r: i32, c: Rgba) {
        for y in -r..=r {
            for x in -r..=r {
                if x * x + y * y <= r * r {
                    self.set(cx + x, cy + y, c);
                }
            }
        }
    }
    /// 绘制文本（字符宽 4 + 间隔 1，高 6），`scale` 为整数放大。返回结束 x。
    pub fn text(&mut self, x: i32, y: i32, s: &str, c: Rgba, scale: i32) -> i32 {
        let mut cx = x;
        for ch in s.chars() {
            if let Some(g) = glyph(ch) {
                for (row, bits) in g.iter().enumerate() {
                    for col in 0..4 {
                        if bits & (0x8 >> col) != 0 {
                            self.rect(cx + col * scale, y + row as i32 * scale, scale, scale, c);
                        }
                    }
                }
            }
            cx += 5 * scale;
        }
        cx
    }
    pub fn text_width(s: &str, scale: i32) -> i32 {
        s.chars().count() as i32 * 5 * scale - scale
    }
    pub fn text_center(&mut self, cx: i32, y: i32, s: &str, c: Rgba, scale: i32) {
        let w = Self::text_width(s, scale);
        self.text(cx - w / 2, y, s, c, scale);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewMode {
    Cockpit,
    Hud,
    Chase,
    Flyby,
}

#[derive(Clone, Debug, Default)]
pub struct IlsHud {
    pub ident: String,
    pub loc_deg: f64,
    pub gs_deg: f64,
    pub loc_valid: bool,
    pub gs_valid: bool,
    pub dist_km: f64,
}

#[derive(Clone, Debug, Default)]
pub struct FlightHud {
    pub ias_kt: f64,
    pub ias_failed: bool,
    pub gs_kt: f64,
    pub alt_ft: f64,
    pub vs_fpm: f64,
    pub agl_ft: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub aoa: f64,
    pub nz: f64,
    pub throttle: f64,
    pub rpm: f64,
    pub fuel_kg: f64,
    pub flaps_deg: f64,
    pub gear_down: bool,
    pub trim: f64,
    pub brake: bool,
    pub parking: bool,
    pub stall: bool,
    pub ice: f64,
    pub ovspd: bool,
    pub glim: bool,
    pub gear_warn: bool,
    pub ils: Option<IlsHud>,
    pub nav_text: String,
    /// 飞行航迹矢量相对机头的偏移（度）：(横向, 垂直)。
    pub fpv: (f64, f64),
    pub damage: Option<String>,
    pub frost: f64,
    pub rain: f64,
    pub time: f64,
    pub code: String,
    pub engine_label: &'static str,
}

fn lamp(buf: &mut PixelBuffer, x: i32, y: i32, label: &str, on: bool, color: Rgba, blink: bool, t: f64) {
    let lit = on && (!blink || (t * 4.0) as i64 % 2 == 0);
    let bg = if lit { color } else { PANEL_DK };
    buf.rect(x, y, label.len() as i32 * 5 + 1, 8, bg);
    buf.text(x + 1, y + 1, label, if lit { BLACK } else { [70, 70, 78, 255] }, 1);
}

/// 地平仪：边长 36 的圆形仪表。
pub fn draw_attitude(buf: &mut PixelBuffer, cx: i32, cy: i32, pitch: f64, roll: f64) {
    let r = 18;
    let (sr, cr) = (roll.to_radians().sin(), roll.to_radians().cos());
    let ppd = 0.9;
    for dy in -r..r {
        for dx in -r..r {
            if dx * dx + dy * dy > r * r {
                continue;
            }
            let (fx, fy) = (dx as f64 + 0.5, dy as f64 + 0.5);
            let d = fy * cr + fx * sr;
            let col = if d > pitch * ppd { GROUND } else { SKY };
            buf.set(cx + dx, cy + dy, col);
        }
    }
    // 俯仰刻度（每 10°）
    for k in [-20i32, -10, 10, 20] {
        let off = (k as f64 - pitch) * -ppd; // 屏幕 y（下为正）：+俯仰在地平线上方
        let (hx, hy) = (-sr * off, cr * off);
        let half = if k.abs() == 10 { 5.0 } else { 8.0 };
        let (ax, ay) = (hx - cr * half, hy + sr * half * -1.0 * -1.0);
        let (bx, by) = (hx + cr * half, hy - sr * half * -1.0 * -1.0);
        let _ = (ax, ay, bx, by);
        let (x0, y0) = (hx - cr * half, hy + sr * half);
        let (x1, y1) = (hx + cr * half, hy - sr * half);
        if hx * hx + hy * hy < (r - 3) as f64 * (r - 3) as f64 {
            buf.line(cx + x0.round() as i32, cy + y0.round() as i32, cx + x1.round() as i32, cy + y1.round() as i32, WHITE);
        }
    }
    // 地平线
    let (hx, hy) = (-sr * (pitch * ppd), cr * (pitch * ppd));
    buf.line(
        cx + (hx - cr * 17.0).round() as i32,
        cy + (hy + sr * 17.0).round() as i32,
        cx + (hx + cr * 17.0).round() as i32,
        cy + (hy - sr * 17.0).round() as i32,
        WHITE,
    );
    // 外圈 + 坡度指针
    for a in 0..72 {
        let t = a as f64 / 72.0 * std::f64::consts::TAU;
        buf.set(cx + (t.cos() * r as f64).round() as i32, cy + (t.sin() * r as f64).round() as i32, GRAY);
    }
    let bank_x = cx as f64 + (r as f64 - 3.0) * (roll.to_radians() - std::f64::consts::FRAC_PI_2).cos() * -1.0 * -1.0;
    let _ = bank_x;
    let a = -std::f64::consts::FRAC_PI_2 - roll.to_radians();
    buf.rect(cx + ((r - 3) as f64 * a.cos()).round() as i32, cy + ((r - 3) as f64 * a.sin()).round() as i32, 2, 2, YELLOW);
    // 固定机翼符号
    buf.rect(cx - 9, cy, 6, 1, YELLOW);
    buf.rect(cx + 4, cy, 6, 1, YELLOW);
    buf.rect(cx - 1, cy - 1, 3, 3, YELLOW);
}

fn draw_cdi(buf: &mut PixelBuffer, x: i32, y: i32, ils: &IlsHud) {
    // 28×28 十字指针：竖针 = 航向道，横针 = 下滑道
    buf.rect(x, y, 30, 30, PANEL_DK);
    buf.frame(x, y, 30, 30, GRAY);
    for k in [-2, -1, 1, 2] {
        buf.set(x + 15 + k * 5, y + 15, GRAY);
        buf.set(x + 15, y + 15 + k * 5, GRAY);
    }
    buf.set(x + 15, y + 15, WHITE);
    if ils.loc_valid {
        // 满偏 ±2.5°（2 点）；指针指向“需要飞向的一侧”：飞机在右侧(+) → 指针偏左
        let px = (-(ils.loc_deg / 2.5) * 10.0).clamp(-13.0, 13.0).round() as i32;
        buf.rect(x + 15 + px, y + 2, 1, 26, MAGENTA);
    } else {
        buf.text(x + 2, y + 2, "LOC", RED, 1);
    }
    if ils.gs_valid {
        // 飞机在下滑道上方(+) → 指针偏下（需要下降）
        let py = ((ils.gs_deg / 0.7) * 10.0).clamp(-13.0, 13.0).round() as i32;
        buf.rect(x + 2, y + 15 + py, 26, 1, MAGENTA);
    } else {
        buf.text(x + 2, y + 23, "GS", RED, 1);
    }
}

fn draw_compass(buf: &mut PixelBuffer, cx: i32, y: i32, heading: f64) {
    buf.rect(cx - 33, y, 66, 10, PANEL_DK);
    buf.frame(cx - 33, y, 66, 10, GRAY);
    for d in -30..=30 {
        let h = (heading + d as f64 * 1.0).rem_euclid(360.0);
        let hi = h.round() as i32 % 360;
        if hi % 10 == 0 {
            buf.rect(cx + d, y + 8, 1, 2, WHITE);
            if hi % 30 == 0 {
                let label = match hi {
                    0 => "N".to_string(),
                    90 => "E".to_string(),
                    180 => "S".to_string(),
                    270 => "W".to_string(),
                    _ => format!("{}", hi / 10),
                };
                let w = PixelBuffer::text_width(&label, 1);
                buf.text(cx + d - w / 2, y + 2, &label, CYAN, 1);
            }
        }
    }
    buf.rect(cx, y, 1, 3, YELLOW);
}

/// 驾驶舱下部 320×48 仪表盘。
fn draw_panel(buf: &mut PixelBuffer, h: &FlightHud) {
    let y0 = COCKPIT_VIEW_H as i32;
    let ph = (H - COCKPIT_VIEW_H) as i32;
    buf.rect(0, y0, W as i32, ph, PANEL);
    buf.rect(0, y0, W as i32, 1, BLACK);

    // 左：油门/引擎/燃油/襟翼/起落架/配平/刹车
    buf.rect(2, y0 + 3, 5, 42, PANEL_DK);
    let th = (h.throttle.clamp(0.0, 1.0) * 40.0).round() as i32;
    buf.rect(3, y0 + 4 + (40 - th), 3, th, GREEN);
    buf.text(10, y0 + 3, &format!("THR {:3.0}", h.throttle * 100.0), WHITE, 1);
    buf.text(10, y0 + 10, &format!("{} {:4.0}", h.engine_label, h.rpm), WHITE, 1);
    buf.text(10, y0 + 17, &format!("FUEL {:3.0}", h.fuel_kg), if h.fuel_kg < 15.0 { RED } else { WHITE }, 1);
    buf.text(10, y0 + 24, &format!("FLP {:2.0}", h.flaps_deg), WHITE, 1);
    buf.text(10, y0 + 31, &format!("TRM {:+.1}", h.trim), WHITE, 1);
    buf.text(10, y0 + 38, if h.parking { "PARK BRK" } else if h.brake { "BRAKE" } else { "GEAR" }, if h.parking || h.brake { AMBER } else { WHITE }, 1);
    if !h.parking && !h.brake {
        buf.rect(30, y0 + 38, 5, 6, if h.gear_down { GREEN } else { RED });
    }

    // 中左：表速 IAS（2× 大字）、AoA、G 条
    buf.rect(64, y0 + 3, 70, 42, PANEL_DK);
    buf.frame(64, y0 + 3, 70, 42, GRAY);
    buf.text(67, y0 + 6, "IAS KT", GRAY, 1);
    if h.ias_failed {
        buf.text(70, y0 + 14, "FAIL", RED, 2);
    } else {
        buf.text(70, y0 + 14, &format!("{:3.0}", h.ias_kt), WHITE, 2);
    }
    buf.text(102, y0 + 6, &format!("GS{:3.0}", h.gs_kt), GRAY, 1);
    buf.text(67, y0 + 30, &format!("AOA{:5.1}", h.aoa), if h.stall { RED } else { WHITE }, 1);
    // G 条：-2..+5
    let g = h.nz.clamp(-2.0, 5.0);
    buf.rect(67, y0 + 38, 64, 5, BLACK);
    buf.rect(67 + 18, y0 + 38, 1, 5, WHITE);
    let gx = 67 + ((g + 2.0) / 7.0 * 63.0).round() as i32;
    buf.rect(gx - 1, y0 + 38, 3, 5, if h.glim { RED } else { GREEN });
    buf.text(112, y0 + 30, &format!("G{:4.1}", h.nz), if h.glim { RED } else { WHITE }, 1);

    // 中：地平仪
    draw_attitude(buf, 160, y0 + 24, h.pitch, h.roll);

    // 中右：ALT、VSI、AGL
    buf.rect(186, y0 + 3, 66, 42, PANEL_DK);
    buf.frame(186, y0 + 3, 66, 42, GRAY);
    buf.text(189, y0 + 6, "ALT FT", GRAY, 1);
    buf.text(189, y0 + 14, &format!("{:5.0}", h.alt_ft), WHITE, 2);
    buf.text(189, y0 + 30, &format!("VS{:+5.0}", h.vs_fpm), if h.vs_fpm < -1500.0 { AMBER } else { WHITE }, 1);
    buf.text(189, y0 + 38, &format!("AGL{:5.0}", h.agl_ft.max(0.0)), if h.agl_ft < 200.0 { AMBER } else { WHITE }, 1);

    // 右：罗盘、ILS 十字、告警灯
    draw_compass(buf, 286, y0 + 3, h.heading);
    buf.text(256, y0 + 15, &format!("HDG {:03.0}", h.heading), WHITE, 1);
    match &h.ils {
        Some(i) => {
            draw_cdi(buf, 256, y0 + 15 + 7 + 3 - 7 + 12 - 12 + 0, i);
            buf.text(290, y0 + 17, &i.ident, MAGENTA, 1);
            buf.text(290, y0 + 24, &format!("{:4.1}K", i.dist_km), WHITE, 1);
        }
        None => {
            buf.text(256, y0 + 24, &h.nav_text, GRAY, 1);
        }
    }
    let t = h.time;
    lamp(buf, 290, y0 + 31, "STALL", h.stall, RED, true, t);
    lamp(buf, 256, y0 + 33, "ICE", h.ice > 0.02, AMBER, h.ice > 0.4, t);
    lamp(buf, 276 - 4, y0 + 33, "OVSPD", h.ovspd, RED, true, t);
    lamp(buf, 256, y0 + 41, "G-LIM", h.glim, RED, true, t);
    lamp(buf, 290, y0 + 40, "GEAR", h.gear_warn, RED, true, t);
}

/// 窗外画面叠加：风挡框、机鼻、结霜、雨线（驾驶舱视角）。
fn draw_windshield(buf: &mut PixelBuffer, h: &FlightHud) {
    let vh = COCKPIT_VIEW_H as i32;
    let frame = [34, 36, 30, 255];
    // 上沿与两侧立柱
    buf.rect(0, 0, W as i32, 3, frame);
    for y in 0..vh {
        let w = 8 - (y * 8 / vh);
        buf.rect(0, y, w.max(2), 1, frame);
        buf.rect(W as i32 - w.max(2), y, w.max(2), 1, frame);
    }
    // 中立柱（细）
    buf.rect(159, 0, 2, 4, frame);
    // 机鼻罩（底部梯形）
    for y in (vh - 9)..vh {
        let k = y - (vh - 9);
        buf.rect(100 - k * 6, y, 120 + k * 12, 1, [52, 56, 48, 255]);
    }
    buf.rect(0, vh - 1, W as i32, 1, BLACK);
    // 结霜：从边缘向内生长
    if h.frost > 0.01 {
        let n = (h.frost * 700.0) as u32;
        let mut s = 0x1234_5678u32;
        for _ in 0..n {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let rx = (s >> 8) % W;
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let ry = (s >> 8) % COCKPIT_VIEW_H;
            // 靠近边缘的概率更大
            let ex = (rx as f64 - 160.0).abs() / 160.0;
            let ey = (ry as f64 - 56.0).abs() / 56.0;
            if ex.max(ey) > 1.0 - h.frost * 0.9 {
                buf.set(rx as i32, ry as i32, [225, 238, 255, 230]);
            }
        }
    }
    // 雨线
    if h.rain > 0.02 {
        let n = (h.rain * 60.0) as i32;
        let mut s = ((h.time * 20.0) as u32).wrapping_mul(2654435761);
        for _ in 0..n {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let rx = ((s >> 8) % W) as i32;
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let ry = ((s >> 8) % (COCKPIT_VIEW_H - 14)) as i32;
            buf.line(rx, ry, rx - 2, ry + 5, [200, 220, 255, 190]);
        }
    }
}

/// 极简 HUD：荧光绿速度带/高度带/FPV/俯仰梯。
fn draw_hud(buf: &mut PixelBuffer, h: &FlightHud, c: Rgba) {
    let cx = 160;
    let cy = 80;
    // 俯仰梯：3.2 px/度（垂直视场 50°），随滚转旋转
    let ppd = 3.2;
    let (sr, cr) = (h.roll.to_radians().sin(), h.roll.to_radians().cos());
    for k in (-30i32..=30).step_by(5) {
        if k == 0 {
            continue;
        }
        let off = (k as f64 - h.pitch) * -ppd;
        let (px, py) = (-sr * off, cr * off);
        if py.abs() > 62.0 {
            continue;
        }
        let half = if k % 10 == 0 { 14.0 } else { 7.0 };
        let (x0, y0) = (px - cr * half, py + sr * half);
        let (x1, y1) = (px + cr * half, py - sr * half);
        buf.line(cx + x0.round() as i32, cy + y0.round() as i32, cx + x1.round() as i32, cy + y1.round() as i32, c);
        if k % 10 == 0 {
            buf.text(cx + x1.round() as i32 + 3, cy + y1.round() as i32 - 3, &format!("{}", k.abs()), c, 1);
        }
    }
    // 地平线
    let off = h.pitch * ppd;
    let (px, py) = (-sr * off, cr * off);
    buf.line(cx + (px - cr * 60.0).round() as i32, cy + (py + sr * 60.0).round() as i32, cx + (px + cr * 60.0).round() as i32, cy + (py - sr * 60.0).round() as i32, c);
    // FPV（飞行航迹矢量）
    let fx = cx + (h.fpv.0 * ppd).clamp(-60.0, 60.0).round() as i32;
    let fy = cy - (h.fpv.1 * ppd).clamp(-50.0, 50.0).round() as i32;
    for a in 0..16 {
        let t = a as f64 / 16.0 * std::f64::consts::TAU;
        buf.set(fx + (t.cos() * 3.5).round() as i32, fy + (t.sin() * 3.5).round() as i32, c);
    }
    buf.rect(fx - 8, fy, 4, 1, c);
    buf.rect(fx + 5, fy, 4, 1, c);
    buf.rect(fx, fy - 7, 1, 3, c);
    // 速度带（左）
    buf.frame(6, cy - 8, 26, 14, c);
    buf.text(9, cy - 4, &if h.ias_failed { "FAIL".to_string() } else { format!("{:3.0}", h.ias_kt) }, c, 1);
    for d in -5..=5 {
        let v = (h.ias_kt / 10.0).round() as i32 * 10 + d * 10;
        let y = cy + ((h.ias_kt - v as f64) * 1.0) as i32 * 2 / 2;
        if v >= 0 && (y - cy).abs() < 54 && (y - cy).abs() > 9 {
            buf.rect(2, y, 3, 1, c);
            buf.text(33, y - 3, &format!("{}", v), c, 1);
        }
    }
    // 高度带（右）
    buf.frame(W as i32 - 38, cy - 8, 32, 14, c);
    buf.text(W as i32 - 36, cy - 4, &format!("{:5.0}", h.alt_ft), c, 1);
    buf.text(W as i32 - 38, cy + 12, &format!("VS{:+5.0}", h.vs_fpm), c, 1);
    buf.text(W as i32 - 38, cy + 20, &format!("R{:5.0}", h.agl_ft.max(0.0)), c, 1);
    // 航向（上）
    buf.text_center(cx, 4, &format!("{:03.0}", h.heading), c, 1);
    buf.rect(cx, 11, 1, 3, c);
    buf.text(6, 4, &format!("G{:3.1}", h.nz), c, 1);
    buf.text(6, 148, &format!("THR{:3.0} FLP{:2.0}", h.throttle * 100.0, h.flaps_deg), c, 1);
    buf.text(W as i32 - 74, 148, &format!("AOA{:5.1}", h.aoa), c, 1);
    if let Some(i) = &h.ils {
        if i.loc_valid {
            let px = cx + (-(i.loc_deg / 2.5) * 20.0).clamp(-60.0, 60.0).round() as i32;
            buf.rect(px, cy + 40, 1, 12, MAGENTA);
        }
        if i.gs_valid {
            let py = cy + ((i.gs_deg / 0.7) * 12.0).clamp(-36.0, 36.0).round() as i32;
            buf.rect(cx + 70, py, 12, 1, MAGENTA);
        }
    }
}

fn draw_alerts(buf: &mut PixelBuffer, h: &FlightHud, y: i32) {
    let blink = (h.time * 3.0) as i64 % 2 == 0;
    let mut msg: Option<(&str, Rgba)> = None;
    if h.damage.as_deref() == Some("CRASH") {
        msg = Some(("CRASH", RED));
    } else if h.stall && blink {
        msg = Some(("STALL", RED));
    } else if h.ovspd && blink {
        msg = Some(("OVERSPEED", RED));
    } else if h.glim && blink {
        msg = Some(("G LIMIT", RED));
    } else if h.gear_warn && blink {
        msg = Some(("GEAR", AMBER));
    }
    if let Some((m, c)) = msg {
        buf.text_center(160, y, m, c, 2);
    }
    if let Some(d) = &h.damage {
        if d != "CRASH" {
            buf.text_center(160, y + 14, d, RED, 1);
        }
    }
}

/// 绘制整个飞行界面到（透明）缓冲。
pub fn draw_flight(buf: &mut PixelBuffer, h: &FlightHud, view: ViewMode) {
    buf.clear(TRANSPARENT);
    match view {
        ViewMode::Cockpit => {
            draw_windshield(buf, h);
            draw_panel(buf, h);
            draw_alerts(buf, h, 30);
        }
        ViewMode::Hud => {
            draw_hud(buf, h, GREEN);
            draw_alerts(buf, h, 30);
            if h.frost > 0.2 {
                draw_windshield_frost_only(buf, h);
            }
        }
        ViewMode::Chase | ViewMode::Flyby => {
            let c = if view == ViewMode::Chase { "CHASE" } else { "FLYBY" };
            buf.text(4, 4, c, WHITE, 1);
            buf.text(4, 12, &format!("IAS {:3.0} ALT {:5.0}", h.ias_kt, h.alt_ft), WHITE, 1);
            buf.text(4, 20, &format!("HDG {:03.0} VS {:+.0}", h.heading, h.vs_fpm), WHITE, 1);
            draw_alerts(buf, h, 30);
        }
    }
    buf.text(W as i32 - 4 - PixelBuffer::text_width(&h.code, 1), 4, &h.code, GRAY, 1);
}

fn draw_windshield_frost_only(buf: &mut PixelBuffer, h: &FlightHud) {
    let mut s = 0x9e37_79b9u32;
    for _ in 0..(h.frost * 500.0) as u32 {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        let rx = (s >> 8) % W;
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        let ry = (s >> 8) % H;
        let ex = (rx as f64 - 160.0).abs() / 160.0;
        let ey = (ry as f64 - 80.0).abs() / 80.0;
        if ex.max(ey) > 1.0 - h.frost * 0.9 {
            buf.set(rx as i32, ry as i32, [225, 238, 255, 230]);
        }
    }
}

// ---------------------------------------------------------------------------
// 航图 / 大地图（按 M 切换）
// ---------------------------------------------------------------------------

pub const MAP_H: i32 = 140;

#[derive(Clone)]
pub struct MapMarker {
    pub px: i32,
    pub py: i32,
    /// 0 = 土路，1 = 支线，2 = 枢纽。
    pub kind: u8,
}

pub struct MapView<'a> {
    /// 320×MAP_H 的 RGB 地图。
    pub terrain: Option<&'a [[u8; 3]]>,
    pub markers: &'a [MapMarker],
    pub selected: Option<usize>,
    pub cursor: (i32, i32),
    pub aircraft: Option<(i32, i32, f64)>,
    pub status: [String; 2],
    pub time: f64,
}

pub fn draw_map(buf: &mut PixelBuffer, m: &MapView) {
    buf.clear(BLACK);
    match m.terrain {
        Some(t) => {
            for y in 0..MAP_H {
                for x in 0..W as i32 {
                    let c = t[(y as u32 * W + x as u32) as usize];
                    buf.set(x, y, [c[0], c[1], c[2], 255]);
                }
            }
        }
        None => {
            buf.rect(0, 0, W as i32, MAP_H, [10, 20, 40, 255]);
            buf.text_center(160, 66, "GENERATING CHART...", CYAN, 1);
        }
    }
    // 经纬网格
    for lon in (0..W as i32).step_by(40) {
        for y in (0..MAP_H).step_by(2) {
            buf.set(lon, y, [255, 255, 255, 60]);
        }
    }
    for lat in (0..MAP_H).step_by(35) {
        for x in (0..W as i32).step_by(2) {
            buf.set(x, lat, [255, 255, 255, 60]);
        }
    }
    for (i, mk) in m.markers.iter().enumerate() {
        let c = match mk.kind {
            2 => WHITE,
            1 => YELLOW,
            _ => AMBER,
        };
        if Some(i) == m.selected {
            if (m.time * 3.0) as i64 % 2 == 0 {
                buf.frame(mk.px - 3, mk.py - 3, 7, 7, RED);
            }
            buf.rect(mk.px - 1, mk.py - 1, 3, 3, RED);
        } else if mk.kind == 2 {
            buf.rect(mk.px - 1, mk.py - 1, 3, 3, c);
        } else {
            buf.set(mk.px, mk.py, c);
            buf.set(mk.px + 1, mk.py, c);
        }
    }
    if let Some((ax, ay, hdg)) = m.aircraft {
        let (dx, dy) = (hdg.to_radians().sin(), -hdg.to_radians().cos());
        buf.line(ax, ay, ax + (dx * 6.0) as i32, ay + (dy * 6.0) as i32, CYAN);
        buf.disc(ax, ay, 1, CYAN);
    }
    // 光标
    let (cx, cy) = m.cursor;
    buf.rect(cx - 4, cy, 3, 1, WHITE);
    buf.rect(cx + 2, cy, 3, 1, WHITE);
    buf.rect(cx, cy - 4, 1, 3, WHITE);
    buf.rect(cx, cy + 2, 1, 3, WHITE);
    // 底部信息栏
    buf.rect(0, MAP_H, W as i32, H as i32 - MAP_H, PANEL_DK);
    buf.text(3, MAP_H + 3, &m.status[0], WHITE, 1);
    buf.text(3, MAP_H + 11, &m.status[1], CYAN, 1);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn font_covers_all_needed_chars() {
        for c in "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-+.:/%<>*^=#? ".chars() {
            assert!(glyph(c).is_some(), "missing glyph {c}");
        }
        assert!(glyph('a').is_some());
    }

    #[test]
    fn text_draws_pixels_within_buffer() {
        let mut b = PixelBuffer::new();
        let end = b.text(2, 2, "IAS 100", WHITE, 1);
        assert_eq!(end, 2 + 7 * 5);
        assert!(b.data.chunks_exact(4).any(|p| p[3] == 255));
    }

    #[test]
    fn attitude_indicator_level_is_half_sky_half_ground() {
        let mut b = PixelBuffer::new();
        draw_attitude(&mut b, 160, 136, 0.0, 0.0);
        assert_eq!(b.get(160, 126), SKY);
        assert_eq!(b.get(160, 146), GROUND);
        // 俯仰 +10°：地平线下移 → 中心上方更多天空，中心偏下仍是天空
        let mut b = PixelBuffer::new();
        draw_attitude(&mut b, 160, 136, 10.0, 0.0);
        assert_eq!(b.get(150, 141), SKY);
        // 右坡度：右侧地平线抬高 → 右侧中间偏上更可能是地面
        let mut b = PixelBuffer::new();
        draw_attitude(&mut b, 160, 136, 0.0, 30.0);
        assert_eq!(b.get(160 + 12, 136 - 3), GROUND);
        assert_eq!(b.get(160 - 12, 136 + 3), SKY);
    }

    #[test]
    fn full_cockpit_renders_without_panic() {
        let mut b = PixelBuffer::new();
        let h = FlightHud { ias_kt: 90.0, alt_ft: 4500.0, stall: true, ice: 0.5, frost: 0.5, rain: 0.5, ils: Some(IlsHud { ident: "IABC".into(), loc_valid: true, gs_valid: true, ..Default::default() }), code: "SKY-172".into(), engine_label: "RPM", ..Default::default() };
        for v in [ViewMode::Cockpit, ViewMode::Hud, ViewMode::Chase, ViewMode::Flyby] {
            draw_flight(&mut b, &h, v);
        }
        draw_flight(&mut b, &h, ViewMode::Cockpit);
        // 下部 48 像素为不透明仪表盘；上部窗外区域大部分透明
        assert_eq!(b.get(160, 150)[3], 255);
        assert_eq!(b.get(160, 40)[3], 0);
    }
}
