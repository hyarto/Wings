#import bevy_sprite::mesh2d_vertex_output::VertexOutput

// 3D 场景（320x160 离屏纹理，sRGB 编码值）与 2D 像素仪表层（Rgba8Unorm）
@group(#{MATERIAL_BIND_GROUP}) @binding(0) var scene_tex: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(1) var scene_samp: sampler;
@group(#{MATERIAL_BIND_GROUP}) @binding(2) var hud_tex: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(3) var hud_samp: sampler;

// 4x4 Bayer 有序抖动矩阵（0..15）
fn bayer4(p: vec2<i32>) -> f32 {
    var m = array<f32, 16>(
        0.0, 8.0, 2.0, 10.0,
        12.0, 4.0, 14.0, 6.0,
        3.0, 11.0, 1.0, 9.0,
        15.0, 7.0, 13.0, 5.0
    );
    let x = p.x & 3;
    let y = p.y & 3;
    return (m[y * 4 + x] + 0.5) / 16.0;
}

@fragment
fn fragment(mesh: VertexOutput) -> @location(0) vec4<f32> {
    let uv = mesh.uv;
    let scene = textureSample(scene_tex, scene_samp, uv).rgb;
    let hud = textureSample(hud_tex, hud_samp, uv);
    // HUD 层叠加（alpha=0 处透出 3D 画面）
    var c = mix(scene, hud.rgb, hud.a);
    // 色彩量化：RGB555（每通道 32 级）+ Bayer 4x4 抖动
    let px = vec2<i32>(floor(uv * vec2<f32>(320.0, 160.0)));
    let t = bayer4(px);
    let levels = 31.0;
    c = floor(c * levels + t) / levels;
    // 显示面板为 sRGB 输出：把 sRGB 编码值还原为线性
    return vec4<f32>(pow(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(2.2)), 1.0);
}
