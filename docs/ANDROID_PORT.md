# STRATO-32 Android 移植（arm64-v8a）

按 [ANDROID_PORT_ASSESSMENT.md](ANDROID_PORT_ASSESSMENT.md) 的方案 A1–A3 落地：**后端零改动复用**，新增「触控虚拟键盘注入层」与 Gradle/GameActivity 外壳。

## 做了什么

| 评估项 | 实现 |
| --- | --- |
| A1 工程结构 | `app_client` 拆为 `lib.rs`（`cdylib`+`rlib`，`#[bevy_main] pub fn main`）+ 薄 `main.rs`（桌面入口不变）。`android-game-activity` feature 只在 `cfg(target_os="android")` 下启用，桌面构建不受影响 |
| A1 Android 外壳 | `android/`：Gradle 8.6 + AGP 8.4 + `androidx.games:games-activity:2.0.2`，`MainActivity extends GameActivity`（沉浸式、常亮、刘海 `shortEdges`、`sensorLandscape`），`minSdk 28 / targetSdk 34`，仅 `arm64-v8a` |
| A2 触控 UI | `crates/app_client/src/touch_ui.rs`：向 `ButtonInput<KeyCode>` 注入按键，`read_input` / `menu_system` / `map_system` 逻辑未改；模拟量（摇杆、油门）经 `TouchAxes` 并入 `read_input`（+14 行） |
| A3 菜单/航图触控 | 开始菜单：直接点行选中、点值区左半/右半 = 减/加、点 `FLY!` 行出发；航图：浮动摇杆平移光标 + 方向键/日期/时间/天气按钮 |
| A3 生命周期 | 切后台（`AppLifecycle::Suspended`）自动暂停并暂停所有音频；回前台恢复音频，飞行保持暂停由玩家点 PAUSE 继续 |
| A3 文件导出 | 黑匣子 CSV 在 Android 上写入应用外部私有目录（`Android/data/com.hyarto.wings/files/`），桌面不变 |
| 测试 | `touch_ui` 内 6 个无头测试：布局不越界/不重叠（5 种分辨率）、真实 `TouchInput` 消息驱动起落架/刹车/油门/摇杆/MORE 面板 |

## 触控操作

飞行界面（横屏）：

* **左半屏任意位置按下拖动** = 浮动虚拟摇杆（向右 = 右滚，**向下 = 拉杆抬头**），带死区与指数曲线；抬手回中。
* **右侧竖条** = 油门滑杆（绝对值，抬手保持）。
* 右侧按钮列：`GEAR` 起落架、`FLAP+/FLAP-` 襟翼、`BRAKE` 按住刹车；右下 `RUD<` `RUD>` 方向舵（按住）。
* 左上：`MENU` 开始菜单、`MAP` 航图、`VIEW` 循环视角、`PAUSE`、`MORE` 展开次级面板：
  `PARK` 驻车刹车 · `PITOT` 皮托加温 · `DEICE` 除冰 · `WX` 切换天气 · `SPBRK` 减速板(按住) · `REV` 反推(按住) ·
  `TRIM+/-`(按住) · `REPLY` 黑匣子回放 · `RESET` 重置 · `ENG-` 失效发动机 · `FEATH` 顺桨 · `FULL/IDLE` 全油门/慢车 · `CSV` 导出黑匣子。
* 黑匣子回放时：`EXIT` / `<<5S` / `5S>>` / `PAUSE`。

桌面也可调试触控 UI：`cargo run -p app_client -- --touch`（或有触摸屏时首次触摸自动启用）。

## 构建 APK

```bash
# 依赖：Rust + `rustup target add aarch64-linux-android` + `cargo install cargo-ndk`
#       Android SDK(platform-34, build-tools-34) + NDK r27 + JDK 17/21
export ANDROID_HOME=... ANDROID_NDK_HOME=$ANDROID_HOME/ndk/27.2.12479018 JAVA_HOME=...
# 可选：签名（不入库）。没有 android/keystore.properties 时回退到 debug 签名
keytool -genkeypair -keystore android/wings-release.jks -alias wings -keyalg RSA -keysize 2048 -validity 10000
printf 'storeFile=wings-release.jks\nstorePassword=...\nkeyAlias=wings\nkeyPassword=...\n' > android/keystore.properties

android/build.sh        # JOBS=2 ./android/build.sh 可降低内存占用
# → android/app/build/outputs/apk/release/app-release.apk
```

构建实测（4 核 / 4 GB，`--profile android` = release + strip）：Rust 首次 ≈ 10 分钟；`libapp_client.so` 68 MB（strip），APK 29 MB。

## 已验证 / 未验证

已验证（沙盒内）：
* `aarch64-linux-android` 下 release 构建通过；`.so` 导出 `android_main` / `GameActivity_onCreate`；
* Gradle 打包、v1/v2/v3 签名通过 `apksigner verify`；`aapt2 dump badging` 显示 `native-code: arm64-v8a`、`sdkVersion 28`；
* 触控层 6 个无头测试 + 原有桌面代码路径编译通过。

**未验证（沙盒没有设备/模拟器/GPU）——首次真机运行请重点关注：**
1. GPU 兼容：`pixel_pipeline` 使用 `Rgba8Unorm` + `Rgba8UnormSrgb` view-format，Vulkan 没问题；仅 GLES 的老机型可能需要降级路径（评估文档 §2.3）。
2. GameActivity 触摸事件：`android-activity 0.6.1` + Bevy 0.18 理论上可用；如果收不到触摸，参考评估文档的兜底方案。
3. 性能与发热：PBR + 地形 LOD，中低端机可能需要降低视距（`terrain_mesh.rs` 常量）。
4. 触控手感（灵敏度曲线、按钮尺寸）需要真机调参。

## 后续（评估文档里程碑 A4/A5，未做）

渲染降级路径（GLES / unlit）、视距档位、CI（cargo-ndk + Gradle + 签名）、AAB、多机型回归。
