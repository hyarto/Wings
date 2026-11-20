//! Floating Origin：物理/世界坐标全部为 f64；仅在渲染层减去飞机基准原点后转 f32 提交给 Bevy，
//! 彻底消除大世界（>100 km）下的浮点抖动。

use bevy::math::DVec3;
use bevy::prelude::*;

/// 当前渲染原点（f64 世界坐标，等于飞机位置）。
#[derive(Resource, Default, Clone, Copy)]
pub struct FloatingOrigin(pub DVec3);

/// 挂在任何“世界固定”实体上：渲染位置 = anchor - origin。
#[derive(Component, Clone, Copy)]
pub struct WorldAnchor(pub DVec3);

#[inline]
pub fn to_render(origin: DVec3, p: DVec3) -> Vec3 {
    (p - origin).as_vec3()
}

pub fn sync_anchors(origin: Res<FloatingOrigin>, mut q: Query<(&WorldAnchor, &mut Transform)>) {
    for (a, mut t) in &mut q {
        t.translation = to_render(origin.0, a.0);
    }
}

pub struct OriginPlugin;
impl Plugin for OriginPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FloatingOrigin>();
    }
}
