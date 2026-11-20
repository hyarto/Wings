# STRATO-32

320×160 像素风、1:32 过程化星球、硬核 6-DOF 气动的复古飞行模拟器（Rust workspace + Bevy 0.18）。
详见 [Spec.md](Spec.md)。当前进度：**M1–M4 完成并有测试覆盖**（5 种机型全部可飞）。

![cockpit](docs/screenshot_cockpit.png)

## 下载

预编译版本见 [Releases](https://github.com/hyarto/Wings/releases)：
[Windows x86_64 (`v0.1.0-windows`)](https://github.com/hyarto/Wings/releases/tag/v0.1.0-windows) ·
[Linux x86_64 (`v0.1.0-linux`)](https://github.com/hyarto/Wings/releases/tag/v0.1.0-linux)

Android：移植可行性评估 [docs/ANDROID_PORT_ASSESSMENT.md](docs/ANDROID_PORT_ASSESSMENT.md) · **移植实现与构建说明 [docs/ANDROID_PORT.md](docs/ANDROID_PORT.md)**（arm64-v8a APK 见 Releases `v0.1.0-android`，构建：`android/build.sh`）

## 运行

```bash
# Linux 需要: pkg-config libasound2-dev libudev-dev libwayland-dev libxkbcommon-dev libx11-dev ...
cargo run -p app_client --release -- [--seed N] [--icao XXXX] [--aircraft CODE] [--day D] [--hour H] [--fly]
cargo run -p app_client --features dev      # 开发期动态链接加速
```

按键：W/S 俯仰、A/D 滚转、Q/E 方向舵、Shift/Ctrl 油门、Z/X 全油门/慢车、F/V 襟翼、G 起落架、
Space 刹车、B 驻车刹车、T/Y 配平、H 皮托加温、I 除冰、J 减速板、K 反推、1–4 视角、
M 航图、C 切换天气预设、P 暂停、Backspace 重置。
**M5 新增**：N 地形雷达图（关 → 小图 → 大图+前方地形剖面）、`[` `]` 雷达量程（5/10/20/40 km）。
**M4 新增**：Tab 开始菜单（选机型 / 出发机场 / 水面或跑道 / 天气 / 日期 / 时间 / 世界种子，↑↓ 选行，←→ 修改，Shift 大步长，Enter 出发）、
R 黑匣子回放（←/→ 快退/快进 5 s，再按 R 回到实时飞行，F5 导出 `blackbox.csv`）、F9 依次失效发动机、F10 顺桨。
启动默认进入开始菜单；`--fly` 直接起飞。

## 指示灯、地形告警与地形雷达（M5）

* **指示灯条**（驾驶舱视图在风挡上沿；HUD/追尾视图在画面底部）：`STALL OVSPD G-LIM GEAR TERR ICE PITOT DEICE`。
  * `PITOT`：绿 = 加热开；琥珀闪 = 处于结冰条件但加热没开；**红闪 = 皮托管已被冰堵塞**（此时空速表冻结，IAS 读数变琥珀、标签变 `IAS FZ`）；灰色划线 = 本机型没有此系统。
  * `DEICE`：绿 = 除冰工作；琥珀闪 = 开着但发动机没转（无动力）或已积冰而开关在关位；灰色划线 = **本机型没有除冰系统**。
    只有 `TWIN-6 / LINER-73 / SEA-415` 装备除冰；`SKY-172 / JET-39` 没有（按 I 会提示 `NO DE-ICE SYSTEM ON THIS AIRCRAFT`）。
  * `ICE`：琥珀 = 处于结冰条件或已有积冰，闪 = 积冰 > 40 %。
  * 按 H / I 会在屏幕上显示 `PITOT HEAT ON/OFF`、`DE-ICE ON/OFF` 等提示。
* **地形告警（简化 TAWS/GPWS）**，`sim_core::taws`：沿航迹/爬升率外推 40 s 的前视地形（含两侧扇形）+ 下沉率包线 + 低空检查。
  `TERRAIN`（黄，净空 < 150 m 且前方地形明显更高）、`SINK RATE`（黄）、`TOO LOW TERRAIN`（黄，< 120 m AGL 且不在机场附近）、
  **`PULL UP`（红，18 s 内净空 < 20 m）** + 对应音效（黄：双短鸣，红：警笛）。放轮且距机场 < 8 km 时抑制前视（正常进近不误报），两栖机水上起降忽略水面。
* **地形雷达图**，`sim_core::terrain_map` + `app_client::terrain_radar`：航向向上的**等高线地形图**，等高线间隔随量程自适应（25/50/100/200 m），每 5 条为白色计曲线；
  地形颜色相对本机海拔分级（红 ≥ 本机高度 −30 m、琥珀 −30…−300 m、绿 −300…−800 m、暗绿更低、蓝 = 水面）。大图模式额外显示前方**地形纵剖面**（含外推航迹、MSL/雷达高度读数）。
  出现告警而雷达图关闭时会自动弹出小图，告警解除后 10 s 收回。

![annunciators](docs/screenshot_annunciators.png)
![terrain radar](docs/screenshot_terrain_radar.png)

## 机型（M4）

| 代号 | 原型 | 动力 | 实测（无头试飞） |
| --- | --- | --- | --- |
| `SKY-172` | Cessna 172 | 单发活塞 | 离地 ≈58 kt，升限 ≈4.2 km |
| `TWIN-6` | DHC-6 | 双发涡桨 STOL（反桨、除冰、单发失效/顺桨） | 离地 ≈65 kt / 滑跑 ≈200 m，巡航 ≈156 kt，升限 ≈7.0–7.4 km |
| `JET-39` | L-39 | 单发涡扇（+8/-4 G、翼尖油箱、减速板、可收放架） | 离地 ≈104 kt / 滑跑 ≈490 m，巡航 ≈370 kt，升限 ≈10.4–11.4 km，滚转 >110°/s |
| `LINER-73` | B737-300 | 双发涡扇（多级襟翼、扰流板、反推、马赫阻力/抖振） | 离地 ≈147 kt / 滑跑 ≈1.06 km，巡航 M0.77 ≈445 kt，升限 ≈12 km |
| `SEA-415` | CL-415 | 双发涡桨水陆两栖（船体 + 翼尖浮筒，浮力/水动阻力/滑水） | 陆地离地 ≈85 kt；水上滑跑 ≈950 m；海面降落并重新起飞；升限 ≈6.1–6.3 km |

调参/测量工具（`crates/sim_core/examples`）：`cargo run --release -p sim_core --example trim`（巡航点平尾配平）、
`cargo run --release -p sim_core --example perf -- CODE [takeoff|stall|roll|land|climb|cruise|water|all]`。

## 测试（全部无头，无需 GPU）

```bash
cargo test --workspace
```

| crate | 内容 |
| --- | --- |
| `sim_core` | 单元测试 + `tests/sky172_flight.rs`（M1：起飞 55–65 kt、失速、升限自然衰减、进近刹停；M3：结冰失速速度、超速/超 G 断翼）+ `tests/m3_weather_structure.rs`（风/风切变/乱流、能见度、湿滑跑道、结冰链路、皮托管加热、除冰、V_FE 卡襟翼、重着陆、折翼后果…） |
| `world_gen` | `tests/world_tests.rs`（确定性、海洋占比、跑道平坦、季节反转）+ `tests/m2_extra.rs`（±180° 跨界无撕裂、100 km f64 精度、机场/ILS/PAPI 几何、`airport_near`、气候单调性…） |
| `sim_core` (M4) | `tests/m4_fleet.rs`：5 机型独立起飞/失速/升限/滚转/着陆测试、机型间差异（滑跑、失速、升限、巡航、滚转率排序，升限与规格 ±20%）、TWIN-6 反桨与单发失效/顺桨、JET-39 +8/−4 G 与超速断翼/减速板/收轮、LINER-73 反推/扰流板/多级襟翼/马赫阻力/“棺材角”、SEA-415 浮力平衡/载重吃水/水上起飞/**海面降落并重新起飞**/放轮入水翻覆；`blackbox` 单元测试 |
| `app_client` (M4) | `src/m4_tests.rs`：5 机型程序化模型（翼展/螺旋桨/起落架/舵面）、开始菜单、机型/机场/天气/季节/种子应用、5 机型在生成世界跑道上稳定停放与起飞、水面出发、黑匣子记录/回放/恢复、Tab/F9/F10/R 输入 |
| `sim_core` (M5) | `tests/m5_systems.rs`：皮托/除冰面板状态（未装备/关/开/无动力）、装备除冰的机型实测积冰对照；`taws.rs` / `terrain_map.rs` 单元测试（前视告警、下沉率包线、进近不误报、水面抑制、监视器保持、网格双线性/等高线/航向向上/剖面） |
| `app_client` (M5) | `src/m5_tests.rs`：各指示灯状态与渲染、H/I 键提示、**真实生成世界里直飞山体时 TAWS 的提前量**、雷达图自动弹出/网格构建/等高线像素、N/[ ] 键 |
| `app_client` | `src/headless_tests.rs`：浮动原点精度、网格/地形 Chunk 无裂缝、驾驶舱像素渲染、航图、输入系统、`WorldGround`、**飞越 ±180°**、**雾/雨/雪中仅凭 ILS 盲降** |

## 里程碑状态

* M1 ✅ 物理核心 + 最小可飞原型
* M2 ✅ 1:32 星球、季节气候、机场网络、Floating Origin、LOD 地形、航图
* M3 ✅ 天气/结冰/结构损坏、ILS 仪表、三视角、程序化音效
* M4 ✅ 5 种机型全量（气动/专属引擎/水面浮力/程序化模型）、开始菜单、黑匣子回放、手感调优
