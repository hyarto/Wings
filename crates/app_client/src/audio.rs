//! 程序化音效合成（无外部音频资源）：活塞/涡桨/涡扇引擎、风噪、失速警报、接地音效。
//! 采样在内存中合成为 16-bit WAV，再交给 bevy_audio 播放；运行时通过 `AudioSink` 调节音调/音量。

use crate::GameState;
use bevy::audio::{AudioPlayer, AudioSink, AudioSinkPlayback, AudioSource, PlaybackSettings, Volume};
use bevy::prelude::*;
use sim_core::aircraft::EngineKind;
use std::f32::consts::TAU;

const RATE: u32 = 22_050;

pub fn wav_bytes(samples: &[f32]) -> Vec<u8> {
    let data_len = (samples.len() * 2) as u32;
    let mut v = Vec::with_capacity(44 + data_len as usize);
    v.extend_from_slice(b"RIFF");
    v.extend_from_slice(&(36 + data_len).to_le_bytes());
    v.extend_from_slice(b"WAVEfmt ");
    v.extend_from_slice(&16u32.to_le_bytes());
    v.extend_from_slice(&1u16.to_le_bytes()); // PCM
    v.extend_from_slice(&1u16.to_le_bytes()); // mono
    v.extend_from_slice(&RATE.to_le_bytes());
    v.extend_from_slice(&(RATE * 2).to_le_bytes());
    v.extend_from_slice(&2u16.to_le_bytes());
    v.extend_from_slice(&16u16.to_le_bytes());
    v.extend_from_slice(b"data");
    v.extend_from_slice(&data_len.to_le_bytes());
    for s in samples {
        v.extend_from_slice(&((s.clamp(-1.0, 1.0) * 30_000.0) as i16).to_le_bytes());
    }
    v
}

struct Lcg(u32);
impl Lcg {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        (self.0 >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
    }
}

/// 把头部淡入、尾部淡出叠加到一起，使循环无爆音。
fn make_loopable(mut s: Vec<f32>, fade: usize) -> Vec<f32> {
    let n = s.len();
    for i in 0..fade {
        let t = i as f32 / fade as f32;
        let tail = s[n - fade + i];
        s[i] = s[i] * t + tail * (1.0 - t);
    }
    s.truncate(n - fade);
    s
}

/// 活塞发动机：基频 + 谐波 + 点火脉动，整数周期 1 秒循环。
pub fn synth_piston() -> Vec<f32> {
    let f0 = 45.0f32;
    let mut n = Lcg(7);
    (0..RATE)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            let mut s = 0.0;
            for k in 1..=14 {
                s += (TAU * f0 * k as f32 * t).sin() / (k as f32).powf(1.15);
            }
            let pulse = 0.65 + 0.35 * (TAU * f0 * 0.5 * t).sin();
            (s * pulse * 0.32 + n.next() * 0.04).clamp(-1.0, 1.0)
        })
        .collect()
}

/// 涡桨：低频螺旋桨拍频 + 高频压气机啸叫。
pub fn synth_turboprop() -> Vec<f32> {
    let mut n = Lcg(11);
    (0..RATE)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            let prop = (TAU * 68.0 * t).sin() * 0.35 + (TAU * 136.0 * t).sin() * 0.2 + (TAU * 204.0 * t).sin() * 0.1;
            let whine = (TAU * 1200.0 * t).sin() * 0.12 + (TAU * 2400.0 * t).sin() * 0.05;
            prop + whine + n.next() * 0.08
        })
        .collect()
}

/// 涡扇：宽带噪声 + 风扇啸叫。
pub fn synth_turbofan() -> Vec<f32> {
    let mut n = Lcg(23);
    let mut lp = 0.0f32;
    (0..RATE)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            lp += (n.next() - lp) * 0.35;
            lp * 0.5 + (TAU * 1500.0 * t).sin() * 0.14 + (TAU * 3000.0 * t).sin() * 0.06 + (TAU * 90.0 * t).sin() * 0.12
        })
        .collect()
}

pub fn synth_wind() -> Vec<f32> {
    let mut n = Lcg(99);
    let mut lp = 0.0f32;
    let raw: Vec<f32> = (0..RATE * 2)
        .map(|_| {
            lp += (n.next() - lp) * 0.08;
            lp * 1.6
        })
        .collect();
    make_loopable(raw, 2000)
}

pub fn synth_stall_horn() -> Vec<f32> {
    (0..RATE)
        .map(|i| {
            let t = i as f32 / RATE as f32;
            let gate = if (t * 2.0).fract() < 0.55 { 1.0 } else { 0.0 };
            let sq = (TAU * 480.0 * t).sin().signum() * 0.5 + (TAU * 960.0 * t).sin() * 0.15;
            sq * gate * 0.5
        })
        .collect()
}

pub fn synth_thump() -> Vec<f32> {
    let mut n = Lcg(5);
    (0..(RATE / 2))
        .map(|i| {
            let t = i as f32 / RATE as f32;
            (TAU * 62.0 * t).sin() * (-t * 14.0).exp() * 0.9 + n.next() * (-t * 38.0).exp() * 0.6
        })
        .collect()
}

#[derive(Component)]
pub enum Sound {
    Engine,
    Wind,
    Stall,
}

#[derive(Resource)]
pub struct SoundBank {
    thump: Handle<AudioSource>,
}

fn setup_audio(mut commands: Commands, mut sources: ResMut<Assets<AudioSource>>, game: Res<GameState>) {
    let mut add = |s: Vec<f32>| sources.add(AudioSource { bytes: wav_bytes(&s).into() });
    let engine = match game.sim.config.engines.first().map(|e| e.kind) {
        Some(EngineKind::Turbofan { .. }) => add(synth_turbofan()),
        Some(EngineKind::Turboprop { .. }) => add(synth_turboprop()),
        _ => add(synth_piston()),
    };
    let wind = add(synth_wind());
    let stall = add(synth_stall_horn());
    let thump = add(synth_thump());
    let quiet = PlaybackSettings::LOOP.with_volume(Volume::Linear(0.0));
    commands.spawn((AudioPlayer::new(engine), quiet, Sound::Engine));
    commands.spawn((AudioPlayer::new(wind), quiet, Sound::Wind));
    commands.spawn((AudioPlayer::new(stall), quiet, Sound::Stall));
    commands.insert_resource(SoundBank { thump });
}

fn update_audio(mut commands: Commands, game: Res<GameState>, bank: Option<Res<SoundBank>>, mut q: Query<(&mut AudioSink, &Sound)>) {
    let active = game.screen == crate::Screen::Flight && !game.paused && !game.sim.state.damage.crashed;
    let d = &game.sim.diag;
    let e = game.sim.state.engines.first().copied().unwrap_or_default();
    let max_rpm = match game.sim.config.engines.first().map(|c| c.kind) {
        Some(EngineKind::Piston { max_rpm, .. }) | Some(EngineKind::Turboprop { max_rpm, .. }) => max_rpm as f32,
        _ => 100.0,
    };
    let eng_norm = (e.rpm as f32 / max_rpm).clamp(0.0, 1.2);
    let cockpit_damp = if game.view == crate::cockpit_ui::ViewMode::Cockpit { 0.7 } else { 1.0 };
    for (mut sink, kind) in &mut q {
        match kind {
            Sound::Engine => {
                sink.set_speed(0.35 + eng_norm * 0.95);
                let v = if active && e.running { (0.12 + 0.45 * eng_norm) * cockpit_damp } else { 0.0 };
                sink.set_volume(Volume::Linear(v));
            }
            Sound::Wind => {
                let v = if active { ((d.tas as f32) / 90.0).clamp(0.0, 1.0).powf(1.5) * 0.45 } else { 0.0 };
                sink.set_speed(0.7 + (d.tas as f32 / 120.0).clamp(0.0, 1.0) * 0.8);
                sink.set_volume(Volume::Linear(v));
            }
            Sound::Stall => {
                let on = active && d.stall_warning;
                sink.set_volume(Volume::Linear(if on { 0.45 } else { 0.0 }));
            }
        }
    }
    if let (Some(td), Some(bank)) = (d.touchdown, bank) {
        commands.spawn((AudioPlayer::new(bank.thump.clone()), PlaybackSettings::DESPAWN.with_volume(Volume::Linear((0.35 + td as f32 * 0.25).min(1.0)))));
    }
}

pub struct AudioPlugin;
impl Plugin for AudioPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostStartup, setup_audio).add_systems(Update, update_audio);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn synthesized_audio_is_valid_wav_and_bounded() {
        for s in [synth_piston(), synth_turboprop(), synth_turbofan(), synth_wind(), synth_stall_horn(), synth_thump()] {
            assert!(!s.is_empty());
            let w = wav_bytes(&s);
            assert_eq!(&w[0..4], b"RIFF");
            assert_eq!(&w[8..12], b"WAVE");
            assert_eq!(w.len(), 44 + s.len() * 2);
            assert!(s.iter().any(|x| x.abs() > 0.05), "silent buffer");
        }
    }
}
