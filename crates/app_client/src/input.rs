//! 键盘 / 手柄统一输入映射（俯仰/滚转/偏航/油门/襟翼/起落架/刹车/配平/视角）。
//!
//! 键盘：W/S 或 ↑/↓ 俯仰（S/↓ 拉杆）、A/D 或 ←/→ 滚转、Q/E 方向舵、Shift/Ctrl 油门↑↓（Z 全油门 X 慢车）、
//! F/V 襟翼放/收、G 起落架、Space 刹车、B 驻车刹车、T/Y 配平、H 皮托加温、I 除冰、J 减速板、K 反推、
//! N 地形雷达图（关/小/大）、[ ] 雷达量程、1-4 视角（驾驶舱/HUD/追尾/掠过）、M 航图、Tab 开始菜单、R 黑匣子回放（←/→ 快退/快进，F5 导出 CSV）、F9 失效一台发动机、F10 顺桨、C 切换天气、P 暂停、Backspace 重置。

use crate::cockpit_ui::ViewMode;
use crate::{GameState, Screen};
use bevy::input::gamepad::{Gamepad, GamepadAxis, GamepadButton};
use bevy::prelude::*;

pub fn read_input(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    mut game: ResMut<GameState>,
    time: Res<Time>,
    menu: Option<ResMut<crate::menu::MenuState>>,
    touch: Option<Res<crate::touch_ui::TouchAxes>>,
) {
    let dt = time.delta_secs_f64();
    // Tab：开始菜单（选机型 / 机场 / 天气 / 季节 / 种子）
    if keys.just_pressed(KeyCode::Tab) {
        if game.screen == Screen::Menu {
            game.screen = Screen::Flight;
        } else {
            game.screen = Screen::Menu;
            if let Some(mut m) = menu {
                *m = crate::menu::MenuState::from_game(&game);
            }
        }
        return;
    }
    if game.screen == Screen::Menu {
        return;
    }
    if keys.just_pressed(KeyCode::KeyM) {
        game.screen = if game.screen == Screen::Flight { Screen::Map } else { Screen::Flight };
    }
    if game.screen != Screen::Flight {
        return;
    }
    // 黑匣子：R 回放/退出回放；回放中 ←/→ 快退/快进 5 s；F5 导出 CSV
    if keys.just_pressed(KeyCode::KeyR) {
        if game.replay.is_some() {
            game.stop_replay();
        } else {
            game.start_replay();
        }
    }
    if keys.just_pressed(KeyCode::F5) {
        let _ = std::fs::write(export_path("blackbox.csv"), game.blackbox.to_csv());
    }
    if game.replay.is_some() {
        if keys.just_pressed(KeyCode::ArrowLeft) {
            game.replay_tick(-5.0);
        }
        if keys.just_pressed(KeyCode::ArrowRight) {
            game.replay_tick(5.0);
        }
        if keys.just_pressed(KeyCode::KeyP) {
            game.paused = !game.paused;
        }
        return;
    }
    // 单发失效训练：F9 使下一台发动机失效；F10 顺桨
    if keys.just_pressed(KeyCode::F9) {
        if let Some(i) = game.sim.state.engines.iter().position(|e| !e.failed) {
            game.sim.fail_engine(i);
        }
    }
    if keys.just_pressed(KeyCode::F10) {
        game.sim.controls.feather = !game.sim.controls.feather;
    }
    let ax = |neg: KeyCode, pos: KeyCode| (keys.pressed(pos) as i32 - keys.pressed(neg) as i32) as f64;
    // 键盘：S/↓ = 拉杆（抬头）
    let mut pitch = ax(KeyCode::KeyW, KeyCode::KeyS) + ax(KeyCode::ArrowUp, KeyCode::ArrowDown);
    let mut roll = ax(KeyCode::KeyA, KeyCode::KeyD) + ax(KeyCode::ArrowLeft, KeyCode::ArrowRight);
    let mut yaw = ax(KeyCode::KeyQ, KeyCode::KeyE);

    // 触控虚拟摇杆（模拟量）覆盖键盘数字量（无头测试里没有该资源）
    let touch = touch.map(|t| *t).unwrap_or_default();
    if touch.pitch != 0.0 {
        pitch = touch.pitch;
    }
    if touch.roll != 0.0 {
        roll = touch.roll;
    }

    let mut throttle_delta = 0.0;
    if keys.pressed(KeyCode::ShiftLeft) {
        throttle_delta += 0.5 * dt;
    }
    if keys.pressed(KeyCode::ControlLeft) {
        throttle_delta -= 0.5 * dt;
    }
    let mut gear_toggle = keys.just_pressed(KeyCode::KeyG);
    let mut flaps_down = keys.just_pressed(KeyCode::KeyF);
    let mut flaps_up = keys.just_pressed(KeyCode::KeyV);
    let mut brake = keys.pressed(KeyCode::Space);

    for pad in &pads {
        let (lx, ly) = (pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0), pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0));
        let rx = pad.get(GamepadAxis::RightStickX).unwrap_or(0.0);
        let dz = |v: f32| if v.abs() < 0.08 { 0.0 } else { v as f64 };
        if dz(lx) != 0.0 {
            roll = dz(lx);
        }
        if dz(ly) != 0.0 {
            pitch = -dz(ly); // 向后拉（摇杆负 Y）= 抬头
        }
        if dz(rx) != 0.0 {
            yaw = dz(rx);
        }
        let rt = pad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) as f64;
        let lt = pad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) as f64;
        throttle_delta += (rt - lt) * 0.6 * dt;
        gear_toggle |= pad.just_pressed(GamepadButton::South);
        flaps_down |= pad.just_pressed(GamepadButton::East);
        flaps_up |= pad.just_pressed(GamepadButton::West);
        brake |= pad.pressed(GamepadButton::LeftTrigger);
    }

    let n_detents = game.sim.config.flap_detents.len();
    if flaps_down && game.flap_idx + 1 < n_detents {
        game.flap_idx += 1;
    }
    if flaps_up && game.flap_idx > 0 {
        game.flap_idx -= 1;
    }
    let flap_target = game.sim.config.flap_detents[game.flap_idx];

    if keys.just_pressed(KeyCode::KeyZ) {
        game.sim.controls.throttle = 1.0;
    }
    if keys.just_pressed(KeyCode::KeyX) {
        game.sim.controls.throttle = 0.0;
    }
    let (has_pitot_heat, has_deice) = (game.sim.config.has_pitot_heat, game.sim.config.has_deice);
    let engine_running = game.sim.state.engines.iter().any(|e| e.running);
    let c = &mut game.sim.controls;
    c.pitch = pitch.clamp(-1.0, 1.0);
    c.roll = roll.clamp(-1.0, 1.0);
    c.yaw = yaw.clamp(-1.0, 1.0);
    c.throttle = (c.throttle + throttle_delta).clamp(0.0, 1.0);
    if let Some(t) = touch.throttle {
        c.throttle = t; // 触控油门滑杆（绝对值）
    }
    c.flaps = flap_target;
    if gear_toggle {
        c.gear_down = !c.gear_down;
    }
    let b = if brake { 1.0 } else { 0.0 };
    c.brake_left = b;
    c.brake_right = b;
    if keys.just_pressed(KeyCode::KeyB) {
        c.parking_brake = !c.parking_brake;
    }
    let trim = (keys.pressed(KeyCode::KeyT) as i32 - keys.pressed(KeyCode::KeyY) as i32) as f64;
    c.pitch_trim = (c.pitch_trim + trim * 0.25 * dt).clamp(-1.0, 1.0);
    let mut toast: Option<String> = None;
    if keys.just_pressed(KeyCode::KeyH) {
        c.pitot_heat = !c.pitot_heat;
        toast = Some(if !has_pitot_heat {
            "NO PITOT HEAT FITTED".to_string()
        } else if c.pitot_heat {
            "PITOT HEAT ON".to_string()
        } else {
            "PITOT HEAT OFF".to_string()
        });
    }
    if keys.just_pressed(KeyCode::KeyI) {
        c.deice = !c.deice;
        toast = Some(if !has_deice {
            "NO DE-ICE SYSTEM ON THIS AIRCRAFT".to_string()
        } else if !c.deice {
            "DE-ICE OFF".to_string()
        } else if !engine_running {
            "DE-ICE ON - NO ENGINE POWER".to_string()
        } else {
            "DE-ICE ON".to_string()
        });
    }
    c.speedbrake = if keys.pressed(KeyCode::KeyJ) { 1.0 } else { 0.0 };
    c.reverse = keys.pressed(KeyCode::KeyK);
    if let Some(t) = toast {
        game.toast = Some((t, 2.0));
    }

    for (k, v) in [(KeyCode::Digit1, ViewMode::Cockpit), (KeyCode::Digit2, ViewMode::Hud), (KeyCode::Digit3, ViewMode::Chase), (KeyCode::Digit4, ViewMode::Flyby)] {
        if keys.just_pressed(k) {
            game.view = v;
        }
    }
    if keys.just_pressed(KeyCode::KeyP) {
        game.paused = !game.paused;
    }
    if keys.just_pressed(KeyCode::KeyC) {
        game.cycle_weather();
    }
    if keys.just_pressed(KeyCode::Backspace) {
        game.respawn();
    }
}

/// 导出文件路径：Android 上工作目录不可写，改用应用私有目录；桌面保持当前目录。
fn export_path(name: &str) -> std::path::PathBuf {
    #[cfg(target_os = "android")]
    {
        if let Some(app) = bevy::android::ANDROID_APP.get() {
            if let Some(dir) = app.external_data_path().or_else(|| app.internal_data_path()) {
                return dir.join(name);
            }
        }
    }
    std::path::PathBuf::from(name)
}
