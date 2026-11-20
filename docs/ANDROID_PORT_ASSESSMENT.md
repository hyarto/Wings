# STRATO-32 → Android（触控 UI 外壳 + 现有后端）移植评估

> 评估日期：2026-11（基于 commit `9cccf64`，Bevy 0.18.1 / wgpu 27 / winit 0.30 / android-activity 0.6.1）
> 方法：不只是读代码——在隔离副本中**真的用 NDK r27c 交叉编译**了 `aarch64-linux-android`，下文区分「已验证」与「估算」。

## 结论（TL;DR）

**难度：中低。** 后端几乎零成本复用，主要工作量在「触控 UI」和「Android 工程打包」，而不是引擎移植。

| 项目 | 难度 | 依据 |
| --- | --- | --- |
| `sim_core` / `world_gen`（物理、世界生成） | ⭐ 几乎为 0 | 纯 Rust、无 Bevy；已有 166 个无头测试 |
| `app_client` 能否编译到 Android | ⭐ **已验证** | 仅加 `android-game-activity` feature，`cargo check --target aarch64-linux-android` 331 个 crate 0 错误 |
| 产出可加载的 `.so` | ⭐⭐ **已验证** | 拆出 `lib.rs`(cdylib) + `#[bevy_main]` 后 release 链接成功，导出 `android_main` / `GameActivity_onCreate` |
| 触控 UI（虚拟摇杆/按钮/菜单/航图） | ⭐⭐⭐ 主要工作量 | 键盘输入集中在 3 个文件，可以不改旧代码 |
| Gradle/Kotlin 壳、签名、打包 | ⭐⭐ | 模板化，约 100 行 Kotlin/Gradle |
| 真机适配（GPU 驱动、性能、生命周期） | ⭐⭐⭐ 最大不确定性 | 沙盒没有设备/模拟器，无法验证 |

**工作量估算（估算，非实测）**：熟悉 Rust+Bevy 的开发者，
「真机上启动并用触控完整飞行」≈ **3–5 个工作日**；「打磨到可发布（多机型适配、设置页、刘海适配、签名 AAB）」≈ **2–3 周**。

---

## 1. 已验证的事实

### 1.1 编译
```bash
rustup target add aarch64-linux-android
# NDK r27c；CC/AR/linker 指向 NDK 的 aarch64-linux-android30-clang
cargo check --target aarch64-linux-android -p app_client      # 通过，0 error（仅 6 个原有 warning）
```
`Cargo.toml` 唯一改动：`bevy = { version = "0.18", features = ["android-game-activity"] }`。

### 1.2 产出 `.so`
把 `main.rs` 重命名为 `lib.rs`，`fn main` 前加 `#[bevy_main]` 并改为 `pub`，新增 3 行 `main.rs` 转发，`Cargo.toml` 增加：
```toml
[lib]
crate-type = ["cdylib", "rlib"]
```
`cargo build --release --lib --target aarch64-linux-android -p app_client` → **2 核机器上 10 分 41 秒成功**：

* `libapp_client.so`：125 MB（带符号）→ strip 后 **68 MB** → gzip 后约 **21 MB**（APK 内压缩后的量级）
* 导出符号：`android_main`、`GameActivity_onCreate`、`Java_com_google_androidgamesdk_GameActivity_initializeNativeCode`
* 动态依赖：`libandroid / liblog / libOpenSLES / libc++_shared`（`libc++_shared.so` 需从 NDK 拷进 `jniLibs`）
* 音频链路 `bevy_audio → cpal → oboe` 在 NDK 下编译通过（游戏音效是内存合成 WAV，无外部资源）

### 1.3 代码层面的天然优势（读代码得出）
* **零外部资源**：模型、地形、仪表、音效全部程序化 → 无需处理 Android assets。
* **渲染分辨率固定 320×160**：GPU 填充率压力极小（瓶颈在顶点数/CPU，见 §2.4）。
* **屏幕比例友好**：主流手机 20:9≈2.22:1，与游戏 2:1 接近，整数缩放后两侧的黑边正好放拇指操作区（如 2400×1080 → 6× = 1920×960，左右各 240 px）。
* **dt 限幅**：`step_sim` 使用 `delta.min(0.1)`，切后台再恢复不会爆步。
* **异步任务**：地形 Chunk / 航图用 `AsyncComputeTaskPool`，Android 上可直接工作。

---

## 2. 需要做的改动清单

### 2.1 工程结构（小）
1. `app_client` 拆 `lib.rs`（cdylib+rlib）+ 薄 `main.rs`（桌面入口不变）。`#[bevy_main]` 同时兼容桌面 `main` 与 Android `android_main`。
2. 平台差异收口：
   * `std::env::args()` — Android 上为空，现有默认值（seed 32，进开始菜单）即可，无需改。
   * `std::fs::write("blackbox.csv")`（`input.rs` 的 F5 导出）— Android 工作目录不可写，改写应用私有目录（`AndroidApp::internal_data_path()`）或隐藏该功能。约 10 行。
   * 窗口：Android 忽略 `resolution`；建议全屏 + 沉浸模式。
3. Android 工程（`android/`）：Gradle + Kotlin `MainActivity : GameActivity`，`cargo ndk -t arm64-v8a -o app/src/main/jniLibs build --release`。参考 Bevy 官方 `examples/mobile`。需 Android SDK + JDK 21 + NDK。

### 2.2 触控 UI（主要工作量）
现状：键盘输入集中在 `input.rs`（42 处引用）、`menu.rs`（9）、`map_screen.rs`（10），其余模块不碰键盘。因此有一个很干净的方案：

**推荐：「虚拟键盘/虚拟手柄」注入层，不改旧逻辑。**
* 新增 `touch_ui.rs`：读取 `Touches`，在 `PreUpdate` 中向 `ButtonInput<KeyCode>` 调用 `press()/release()`。`read_input` / `menu_system` / `map_system` 无需修改即可工作。
* 模拟量：键盘是 ±1 数字量；手柄分支已有模拟量处理（`dz()` 死区）。给 `read_input` 增加 `TouchAxes { pitch, roll, yaw, throttle }` 资源并并入手柄分支即可（约 30 行）。
* 控件布局（利用两侧黑边）：
  * 左：浮动虚拟摇杆（俯仰/滚转，带死区与非线性）；方向舵用底部滑条或摇杆扭转。
  * 右：竖直油门滑杆（绝对值，对应 `controls.throttle`）。
  * 按钮：起落架、襟翼 ±、刹车(按住)、驻车、暂停、视角循环、地图、菜单、反推/减速板(按住)、配平(按住)、皮托加热、除冰、天气、黑匣子回放、复位；低频项收进「更多」抽屉。
  * 可选：重力感应倾斜操纵（需 JNI 读传感器，非必需）。
* 渲染方式：**bevy_ui 覆盖层**（推荐，清晰、不受 320×160 量化/抖动影响、命中检测简单）；画进 `HudBuffer` 风格统一但按钮太小（不推荐作为主要触点）。
* **菜单/航图触控**：它们是「行选择 + ←→ 改值」的键盘交互，需要：
  * 触点坐标换算到 320×160 像素空间（`fit_quad_to_window` 已有整数缩放，反算简单）：点某行=选中，点值两侧 ◀ ▶ 或水平拖动=改值；
  * 航图：单指拖动平移（现为方向键）、双指捏合缩放（现为 `[` `]`）、长按选机场（现为 Enter）。
  * 估算每个屏幕 0.5–1 天。

### 2.3 渲染/兼容性（不确定性所在）
* `pixel_pipeline` 创建 `Rgba8Unorm` 离屏目标并声明 `Rgba8UnormSrgb` 视图格式。**Vulkan 没问题**；若设备只能走 GLES（老机型），该 view-format 可能不支持 → 需降级路径（直接用 Srgb 目标并去掉 WGSL 里手动 gamma）。
* 使用 `StandardMaterial`（PBR）+ 方向光 + 雾，`far = 900 km`。PBR 在低端 GPU 上偏重；游戏本身 320×160 + RGB555 量化，PBR 收益很低，Android 可用 feature 切到 `unlit` 顶点色。
* 社区反馈 wgpu/Bevy 在部分 Mali 芯片上有闪烁/崩溃（Bevy PR #23491 讨论提到需回移植 wgpu 补丁）。只能真机验证。
* GameActivity 触控：社区报告过 Bevy + `android-activity` 某些版本组合收不到触摸事件（兜底：Java 层 `onTouchEvent` + JNI 转发，或升级 `games-activity` 4.0 + 补丁版 android-activity）。**第一天就应验证的风险点。**

### 2.4 生命周期与性能
* 切后台：Surface 会被销毁重建，Bevy 基本自动处理；建议在 `AppLifecycle::Suspended` 时自动 `paused = true` 并静音。
* 物理固定步长 120 Hz，`dt<=0.1s` ⇒ 最多 12 步/帧；`sim_core` 的 22 个 `m4_fleet` 飞行测试 release 下合计 12.7 s（含大量长时间仿真），手机 CPU 足够。
* 地形：4 级 LOD、异步网格生成。中低端机建议降低 Chunk 网格分辨率/视距（常量在 `terrain_mesh.rs`）并限制每帧上传 Chunk 数。
* 内存：未实测，程序化世界无大纹理，预计 < 500 MB。

---

## 3. 风险排序
1. **真机 GPU/驱动兼容（高）**：Mali/Adreno/PowerVR 上的 wgpu 稳定性、view-format、深度精度（near 0.3 / far 900 000）。需 3–5 台不同 SoC 的真机回归。
2. **GameActivity 触控事件（中）**：见 §2.3。
3. **构建环境（低-中）**：Android release 编译 ~11 分钟（2 核）；CI 用 `cargo-ndk` + GitHub Actions。
4. **触控手感（中）**：6-DOF 硬核模拟用虚拟摇杆很难精细控制，需要灵敏度曲线、自动配平/辅助（可做「入门模式」）与按住/切换式按钮的反复调参——**产品层面而非技术层面的工作量**。
5. **包体（低）**：strip 后 68 MB `.so`；`opt-level="s"`、`lto="thin"`、裁剪 Bevy 默认 feature 可降到约 30–40 MB。

## 4. 建议里程碑
| 阶段 | 内容 | 估算 |
| --- | --- | --- |
| A1 | 拆 lib/bin、Gradle 壳、`.so` 打包，启动并显示菜单（验证 GameActivity 触控） | 1 天 |
| A2 | `touch_ui.rs` 注入层 + 虚拟摇杆/油门/核心按钮（可起飞、巡航、降落） | 1–2 天 |
| A3 | 菜单与航图触控化、沉浸/刘海适配、生命周期暂停 | 1 天 |
| A4 | 渲染降级路径（GLES / unlit 材质）、性能与视距档位 | 1–2 天 |
| A5 | CI（cargo-ndk + Gradle + 签名）、多机型回归、体验调参 | 3–7 天 |

## 5. 本次评估没有做、也无法做的事
* **没有在真机或模拟器上运行**（沙盒无 GPU/KVM），因此「能编译、能链接」≠「能跑」，性能与兼容性结论为推断。
* 没有构建 APK（未安装 Android SDK/Gradle）。
* 没有改动主仓库的任何游戏代码；lib 拆分只在临时副本中验证。
