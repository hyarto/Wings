//! 键盘 / 手柄统一输入映射（俯仰/滚转/偏航/油门/襟翼/起落架/刹车/配平/视角）。
//!
//! 键盘：W/S 或 ↑/↓ 俯仰（S/↓ 拉杆）、A/D 或 ←/→ 滚转、Q/E 方向舵、Shift/Ctrl 油门↑↓（Z 全油门 X 慢车）、
//! F/V 襟翼放/收、G 起落架、Space 刹车、B 驻车刹车、T/Y 配平、H 皮托加温、I 除冰、J 减速板、K 反推、
//! 1-4 视角（驾驶舱/HUD/追尾/掠过）、M 航图、C 切换天气、P 暂停、Backspace 重置。

use crate::cockpit_ui::ViewMode;
use crate::{GameState, Screen};
use bevy::input::gamepad::{Gamepad, GamepadAxis, GamepadButton};
use bevy::prelude::*;

pub fn read_input(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, mut game: ResMut<GameState>, time: Res<Time>) {
    let dt = time.delta_secs_f64();
    if keys.just_pressed(KeyCode::KeyM) {
        game.screen = if game.screen == Screen::Flight { Screen::Map } else { Screen::Flight };
    }
    if game.screen != Screen::Flight {
        return;
    }
    let ax = |neg: KeyCode, pos: KeyCode| (keys.pressed(pos) as i32 - keys.pressed(neg) as i32) as f64;
    // 键盘：S/↓ = 拉杆（抬头）
    let mut pitch = ax(KeyCode::KeyW, KeyCode::KeyS) + ax(KeyCode::ArrowUp, KeyCode::ArrowDown);
    let mut roll = ax(KeyCode::KeyA, KeyCode::KeyD) + ax(KeyCode::ArrowLeft, KeyCode::ArrowRight);
    let mut yaw = ax(KeyCode::KeyQ, KeyCode::KeyE);

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
    let c = &mut game.sim.controls;
    c.pitch = pitch.clamp(-1.0, 1.0);
    c.roll = roll.clamp(-1.0, 1.0);
    c.yaw = yaw.clamp(-1.0, 1.0);
    c.throttle = (c.throttle + throttle_delta).clamp(0.0, 1.0);
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
    if keys.just_pressed(KeyCode::KeyH) {
        c.pitot_heat = !c.pitot_heat;
    }
    if keys.just_pressed(KeyCode::KeyI) {
        c.deice = !c.deice;
    }
    c.speedbrake = if keys.pressed(KeyCode::KeyJ) { 1.0 } else { 0.0 };
    c.reverse = keys.pressed(KeyCode::KeyK);

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
