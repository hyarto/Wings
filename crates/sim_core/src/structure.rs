//! 结构过载、超速、部件损坏状态机。
//!
//! * 超过限制载荷 / V_NE：触发警报、机身震颤，并累积 `stress`（0..1）；
//! * `stress` 达到 1 或载荷超过极限载荷（Limit × ultimate_factor）：不可逆结构故障。

#[derive(Clone, Copy, Debug)]
pub struct StructureLimits {
    pub limit_load_pos: f64,
    pub limit_load_neg: f64,
    /// 极限载荷 = 限制载荷 × ultimate_factor（通常 1.5）。
    pub ultimate_factor: f64,
    /// 不可逾越速度（IAS, m/s）。
    pub v_ne: f64,
    /// 襟翼放速。
    pub v_fe: f64,
    /// 起落架放速（固定起落架可设很大）。
    pub v_le: f64,
    /// 允许的最大接地下沉率（m/s），超过则损伤。
    pub max_touchdown_sink: f64,
    /// 超过此下沉率起落架坍塌/机体解体。
    pub gear_collapse_sink: f64,
}

#[derive(Clone, Debug, Default)]
pub struct DamageState {
    pub left_wing_failed: bool,
    pub right_wing_failed: bool,
    /// 襟翼卡死在某个位置。
    pub flaps_jammed: Option<f64>,
    pub gear_collapsed: bool,
    /// 重着陆损伤（起落架受损，但未坍塌）。
    pub hard_landing: bool,
    pub crashed: bool,
    /// 累积应力 0..1。
    pub stress: f64,
}

impl DamageState {
    pub fn any(&self) -> bool {
        self.left_wing_failed
            || self.right_wing_failed
            || self.flaps_jammed.is_some()
            || self.gear_collapsed
            || self.hard_landing
            || self.crashed
    }
}

/// 每步产生的结构告警标志（驱动 HUD 与机身震颤）。
#[derive(Clone, Copy, Debug, Default)]
pub struct StructureWarnings {
    pub g_limit: bool,
    pub overspeed: bool,
    pub flap_overspeed: bool,
    pub gear_overspeed: bool,
    /// 震颤强度 0..1。
    pub buffet: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct StructureInput {
    /// 法向过载（机体 +Y 方向，地面静止为 +1）。
    pub nz: f64,
    pub ias: f64,
    pub flaps: f64,
    pub gear_pos: f64,
    /// 滚转角速度符号用于决定折断哪一侧机翼（+ = 右滚）。
    pub roll_rate: f64,
    pub dt: f64,
}

/// 更新结构状态。返回当步告警。
pub fn update(limits: &StructureLimits, dmg: &mut DamageState, inp: &StructureInput) -> StructureWarnings {
    let mut w = StructureWarnings::default();
    if dmg.crashed {
        return w;
    }
    let both_failed = dmg.left_wing_failed && dmg.right_wing_failed;

    // --- G 过载 ---
    let nz = inp.nz;
    let ratio = if nz >= 0.0 { nz / limits.limit_load_pos } else { -nz / limits.limit_load_neg.abs() };
    if ratio > 1.0 {
        w.g_limit = true;
        w.buffet = w.buffet.max(((ratio - 1.0) * 2.0).clamp(0.2, 1.0));
        dmg.stress += inp.dt * (ratio - 1.0) * 1.5;
    }
    if ratio > limits.ultimate_factor && !both_failed {
        break_wing(dmg, inp.roll_rate, nz);
    }

    // --- 超速 V_NE ---
    let vr = inp.ias / limits.v_ne;
    if vr > 1.0 {
        w.overspeed = true;
        w.buffet = w.buffet.max(((vr - 1.0) * 4.0).clamp(0.1, 1.0));
        dmg.stress += inp.dt * (vr * vr - 1.0);
        if vr > 1.5 {
            break_wing(dmg, inp.roll_rate, nz);
        }
    } else if vr < 0.98 && !w.g_limit {
        // 应力缓慢“恢复”仅用于告警，不代表结构修复（材料疲劳不可逆；这里仅防止噪声累积）。
    }
    if dmg.stress >= 1.0 && !both_failed {
        break_wing(dmg, inp.roll_rate, nz);
        dmg.stress = dmg.stress.min(1.0);
    }

    // --- V_FE：襟翼 ---
    if inp.flaps > 0.05 && inp.ias > limits.v_fe {
        w.flap_overspeed = true;
        if inp.ias > limits.v_fe * 1.25 && dmg.flaps_jammed.is_none() {
            dmg.flaps_jammed = Some(inp.flaps);
        }
    }
    // --- V_LE：起落架 ---
    if inp.gear_pos > 0.05 && inp.ias > limits.v_le {
        w.gear_overspeed = true;
        if inp.ias > limits.v_le * 1.25 && inp.gear_pos < 0.99 {
            dmg.gear_collapsed = true;
        }
    }
    w
}

fn break_wing(dmg: &mut DamageState, roll_rate: f64, nz: f64) {
    if dmg.left_wing_failed || dmg.right_wing_failed {
        // 一侧已折断，另一侧随之承载失衡也折断
        dmg.left_wing_failed = true;
        dmg.right_wing_failed = true;
        return;
    }
    // 负过载时方向反转；默认折断“滚转进入方向”相反一侧的外翼承载更高的一侧
    let side_right = (roll_rate * nz.signum()) < 0.0;
    if side_right {
        dmg.right_wing_failed = true;
    } else {
        dmg.left_wing_failed = true;
    }
}

/// 着陆接地评估：返回 (是否损伤, 是否坍塌)。
pub fn evaluate_touchdown(limits: &StructureLimits, dmg: &mut DamageState, sink_rate: f64) {
    if sink_rate > limits.gear_collapse_sink {
        dmg.gear_collapsed = true;
        dmg.hard_landing = true;
    } else if sink_rate > limits.max_touchdown_sink {
        dmg.hard_landing = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn limits() -> StructureLimits {
        StructureLimits {
            limit_load_pos: 3.8,
            limit_load_neg: -1.52,
            ultimate_factor: 1.5,
            v_ne: 84.0,
            v_fe: 44.0,
            v_le: 1000.0,
            max_touchdown_sink: 3.0,
            gear_collapse_sink: 4.6,
        }
    }
    fn inp(nz: f64, ias: f64) -> StructureInput {
        StructureInput { nz, ias, flaps: 0.0, gear_pos: 1.0, roll_rate: 0.1, dt: 1.0 / 120.0 }
    }
    #[test]
    fn ultimate_load_breaks_wing() {
        let mut d = DamageState::default();
        update(&limits(), &mut d, &inp(4.0, 50.0));
        assert!(!d.left_wing_failed && !d.right_wing_failed);
        update(&limits(), &mut d, &inp(6.0, 50.0));
        assert!(d.left_wing_failed || d.right_wing_failed);
    }
    #[test]
    fn sustained_overspeed_breaks() {
        let mut d = DamageState::default();
        let w = update(&limits(), &mut d, &inp(1.0, 90.0));
        assert!(w.overspeed);
        for _ in 0..1200 {
            update(&limits(), &mut d, &inp(1.0, 100.0));
        }
        assert!(d.left_wing_failed || d.right_wing_failed);
    }
    #[test]
    fn hard_landing() {
        let mut d = DamageState::default();
        evaluate_touchdown(&limits(), &mut d, 1.0);
        assert!(!d.any());
        evaluate_touchdown(&limits(), &mut d, 3.5);
        assert!(d.hard_landing && !d.gear_collapsed);
        evaluate_touchdown(&limits(), &mut d, 5.5);
        assert!(d.gear_collapsed);
    }
}
