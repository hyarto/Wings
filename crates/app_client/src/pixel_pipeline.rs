//! 320×160 离屏渲染管线：
//! 主 3D 摄像机 → 320×160 离屏纹理（`Msaa::Off`），再由全屏后处理材质
//! 执行 Bayer 4×4 有序抖动 + RGB555 色彩量化，以 Nearest 采样整数倍放大到窗口（黑边信箱，严格 2:1）。

use bevy::asset::{embedded_asset, RenderAssetUsages};
use bevy::camera::visibility::RenderLayers;
use bevy::camera::RenderTarget;
use bevy::image::ImageSampler;
use bevy::prelude::*;
use bevy::reflect::TypePath;
use bevy::render::render_resource::{AsBindGroup, TextureFormat};
use bevy::shader::ShaderRef;
use bevy::sprite_render::{Material2d, Material2dPlugin};

pub const W: u32 = 320;
pub const H: u32 = 160;
/// 驾驶舱视角下 3D 窗外画面高度（上部 320×112）。
pub const COCKPIT_VIEW_H: u32 = 112;
/// 2D 像素合成层的渲染层。
pub const POST_LAYER: usize = 1;

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
pub struct PixelPostMaterial {
    #[texture(0)]
    #[sampler(1)]
    pub scene: Handle<Image>,
    #[texture(2)]
    #[sampler(3)]
    pub hud: Handle<Image>,
}

impl Material2d for PixelPostMaterial {
    fn fragment_shader() -> ShaderRef {
        "embedded://app_client/pixel_post.wgsl".into()
    }
}

#[derive(Resource)]
pub struct PixelTargets {
    pub scene: Handle<Image>,
    pub hud: Handle<Image>,
}

#[derive(Component)]
pub struct SceneCamera;
#[derive(Component)]
pub struct PostCamera;
#[derive(Component)]
pub struct PostQuad;

pub struct PixelPipelinePlugin;

impl Plugin for PixelPipelinePlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "pixel_post.wgsl");
        app.add_plugins(Material2dPlugin::<PixelPostMaterial>::default())
            .add_systems(Startup, setup_pipeline)
            .add_systems(Update, (fit_quad_to_window, upload_hud));
    }
}

fn nearest(mut img: Image) -> Image {
    img.sampler = ImageSampler::nearest();
    img
}

fn setup_pipeline(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<PixelPostMaterial>>,
) {
    // 3D 离屏目标（320×160；RGBA8，sRGB 视图写入）
    let scene = nearest(Image::new_target_texture(W, H, TextureFormat::Rgba8Unorm, Some(TextureFormat::Rgba8UnormSrgb)));
    let scene = images.add(scene);

    // 2D 像素仪表层（CPU 缓冲，原生 320×160，透明）
    let hud = nearest(Image::new_fill(
        bevy::render::render_resource::Extent3d { width: W, height: H, depth_or_array_layers: 1 },
        bevy::render::render_resource::TextureDimension::D2,
        &[0, 0, 0, 0],
        TextureFormat::Rgba8Unorm,
        RenderAssetUsages::default(),
    ));
    let hud = images.add(hud);

    // 3D 主摄像机：渲染到离屏纹理，Msaa 关闭
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, clear_color: ClearColorConfig::Custom(Color::srgb(0.45, 0.65, 0.9)), ..default() },
        RenderTarget::Image(scene.clone().into()),
        Msaa::Off,
        Projection::Perspective(PerspectiveProjection { fov: 50f32.to_radians(), near: 0.3, far: 900_000.0, ..default() }),
        Transform::default(),
        SceneCamera,
    ));

    // 全屏后处理四边形 + 2D 摄像机（独立渲染层）
    let layer = RenderLayers::layer(POST_LAYER);
    commands.spawn((Camera2d, Camera { order: 0, clear_color: ClearColorConfig::Custom(Color::BLACK), ..default() }, Msaa::Off, layer.clone(), PostCamera));
    commands.spawn((
        Mesh2d(meshes.add(Rectangle::new(1.0, 1.0))),
        MeshMaterial2d(materials.add(PixelPostMaterial { scene: scene.clone(), hud: hud.clone() })),
        Transform::from_scale(Vec3::new(W as f32 * 3.0, H as f32 * 3.0, 1.0)),
        layer,
        PostQuad,
    ));
    commands.insert_resource(PixelTargets { scene, hud });
}

/// 整数倍缩放：窗口物理像素 / (320×160) 向下取整，剩余区域为黑边（严格 2:1）。
fn fit_quad_to_window(windows: Query<&Window>, mut quad: Query<&mut Transform, With<PostQuad>>) {
    let (Ok(win), Ok(mut tf)) = (windows.single(), quad.single_mut()) else { return };
    let (pw, ph) = (win.physical_width().max(1), win.physical_height().max(1));
    let s = (pw / W).min(ph / H).max(1) as f32;
    let sf = win.scale_factor().max(0.1);
    tf.scale = Vec3::new(W as f32 * s / sf, H as f32 * s / sf, 1.0);
}

/// 把 CPU 像素缓冲上传到 GPU。
#[derive(Resource)]
pub struct HudBuffer(pub crate::cockpit_ui::PixelBuffer);

fn upload_hud(
    buf: Option<Res<HudBuffer>>,
    targets: Option<Res<PixelTargets>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<PixelPostMaterial>>,
    quads: Query<&MeshMaterial2d<PixelPostMaterial>>,
) {
    let (Some(buf), Some(t)) = (buf, targets) else { return };
    if let Some(img) = images.get_mut(&t.hud) {
        img.data = Some(buf.0.data.clone());
    }
    // 触碰材质资源 → 强制重建 bind group，使其绑定到刚重新上传的 HUD 纹理。
    for m in &quads {
        let _ = materials.get_mut(&m.0);
    }
}
