//! 轻量确定性 3D 梯度噪声（Perlin 风格），用于乱流/阵风。无外部依赖。

#[inline]
fn hash(mut x: u32, y: i32, z: i32, w: i32) -> u32 {
    x ^= (y as u32).wrapping_mul(0x85eb_ca6b);
    x = x.rotate_left(13).wrapping_mul(5).wrapping_add(0xe654_6b64);
    x ^= (z as u32).wrapping_mul(0xc2b2_ae35);
    x = x.rotate_left(15).wrapping_mul(0x27d4_eb2f);
    x ^= (w as u32).wrapping_mul(0x1656_67b1);
    x ^= x >> 15;
    x = x.wrapping_mul(0x2c1b_3c6d);
    x ^= x >> 12;
    x = x.wrapping_mul(0x297a_2d39);
    x ^= x >> 15;
    x
}

#[inline]
fn grad(h: u32, x: f64, y: f64, z: f64) -> f64 {
    match h % 12 {
        0 => x + y,
        1 => -x + y,
        2 => x - y,
        3 => -x - y,
        4 => x + z,
        5 => -x + z,
        6 => x - z,
        7 => -x - z,
        8 => y + z,
        9 => -y + z,
        10 => y - z,
        _ => -y - z,
    }
}

#[inline]
fn fade(t: f64) -> f64 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// 3D 梯度噪声，输出 `[-1, 1]`。
pub fn noise3(seed: u32, x: f64, y: f64, z: f64) -> f64 {
    let (xf0, yf0, zf0) = (x.floor(), y.floor(), z.floor());
    let (xf, yf, zf) = (x - xf0, y - yf0, z - zf0);
    let (xi, yi, zi) = (xf0 as i32, yf0 as i32, zf0 as i32);
    let (u, v, w) = (fade(xf), fade(yf), fade(zf));
    let g = |dx: i32, dy: i32, dz: i32| {
        grad(hash(seed, xi + dx, yi + dy, zi + dz), xf - dx as f64, yf - dy as f64, zf - dz as f64)
    };
    let l = |a: f64, b: f64, t: f64| a + (b - a) * t;
    let x00 = l(g(0, 0, 0), g(1, 0, 0), u);
    let x10 = l(g(0, 1, 0), g(1, 1, 0), u);
    let x01 = l(g(0, 0, 1), g(1, 0, 1), u);
    let x11 = l(g(0, 1, 1), g(1, 1, 1), u);
    let y0 = l(x00, x10, v);
    let y1 = l(x01, x11, v);
    (l(y0, y1, w) * 0.9).clamp(-1.0, 1.0)
}

/// 分形布朗运动（fBm），输出约 `[-1, 1]`。
pub fn fbm3(seed: u32, x: f64, y: f64, z: f64, octaves: u32) -> f64 {
    let (mut amp, mut freq, mut sum, mut norm) = (1.0, 1.0, 0.0, 0.0);
    for o in 0..octaves {
        sum += amp * noise3(seed.wrapping_add(o * 101), x * freq, y * freq, z * freq);
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn deterministic_and_bounded() {
        assert_eq!(noise3(7, 1.3, 2.1, 9.7), noise3(7, 1.3, 2.1, 9.7));
        for i in 0..1000 {
            let v = noise3(3, i as f64 * 0.37, i as f64 * 0.11, i as f64 * 0.73);
            assert!(v.abs() <= 1.0);
        }
    }
}

/// 脊状多重分形（Ridged Multifractal），输出 `[0, 1]`，用于构造山脉。
pub fn ridged3(seed: u32, x: f64, y: f64, z: f64, octaves: u32) -> f64 {
    let (mut amp, mut freq, mut sum, mut norm, mut weight) = (1.0, 1.0, 0.0, 0.0, 1.0);
    for o in 0..octaves {
        let n = noise3(seed.wrapping_add(o * 977), x * freq, y * freq, z * freq);
        let mut r = 1.0 - n.abs();
        r *= r;
        r *= weight;
        weight = (r * 1.6).clamp(0.0, 1.0);
        sum += r * amp;
        norm += amp;
        amp *= 0.5;
        freq *= 2.03;
    }
    (sum / norm).clamp(0.0, 1.0)
}

/// 64-bit 种子折叠为 32-bit 噪声种子。
pub fn fold_seed(seed: u64, salt: u32) -> u32 {
    let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15u64.wrapping_mul(salt as u64 + 1));
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    (z ^ (z >> 31)) as u32
}

/// 确定性哈希 → [0,1)。
pub fn hash01(seed: u64, a: i64, b: i64, c: i64) -> f64 {
    let mut z = seed
        ^ (a as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (b as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
        ^ (c as u64).wrapping_mul(0x1656_67B1_9E37_79F9);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    (z >> 11) as f64 / (1u64 << 53) as f64
}

#[inline]
pub fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
