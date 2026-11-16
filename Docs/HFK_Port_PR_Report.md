# 移植 HFK 巡线/舵机/环岛/十字/逆透视：修改报告

> 分支：`other`
> 目标仓库：`another-bbzj/smart-car-vision`
> 基线：`main`
> 规模：8 个提交、57 个文件、+3201 / −631（其中 3 个提交为本次功能移植，5 个为前置注释整理）

## 1. 摘要

本 PR 把 `D:\NEUQCAR\HFK_STC32G144K`（第 21 届智能车室内镜头组“雁过留痕”）中的
纯跟踪、舵机模型、增量式 PID、环岛、十字、灰度巡线和逆透视 LUT 移植到本仓库，
并按本仓库的实际架构做了适配：

- 本仓库是 **STM32F407 差速底盘 + 香橙派 OpenCV 视觉**；
- HFK 是 **STC32G144K 单片摄像头 + 阿克曼舵机**。

因此本次不是“复制粘贴”，而是把算法拆成两层：

| 层 | 落点 | 内容 |
|---|---|---|
| 图像/几何 | 香橙派 C++ | 动态阈值、边线爬行、角点、逆透视 LUT、路径导出 |
| 车辆控制 | STM32F407 | 虚拟舵机、纯跟踪、环岛/十字状态机、增量式速度环 |

这是一个**大范围变更**，涉及固件、视觉、串口协议、测试和文档。默认行为尽量保持
向后兼容：不配置路径帧时，`Race_Step` 仍按原来的视觉曲率行驶；增量式 PID 默认关闭。

## 2. 背景与目标

HFK 公开代码中有几处实现比本仓库更完整或更简洁：

1. **纯跟踪**：在预瞄点几何上直接计算 `d = atan2(2·D·dy, dx²+dy²)`，并按速度分段
   调整前瞻（`DTarget()`），直道和弯道的转向量比单纯的曲率跟随更平滑。
2. **舵机模型**：`setServoDuty()` 把转角命令乘增益、限幅，再叠加中位输出 PWM；
   控制链清晰，便于把“转向角”和“执行器”解耦。
3. **增量式速度环**：`motorPIDCalculate()` 用增量式 PID 驱动双电机，抗积分饱和
   和换向行为与位置式不同，适合对比调参。
4. **环岛/十字状态机**：四相位环岛、两相位十字的划分清楚，便于按里程和元素扩展。
5. **灰度巡线**：局部动态阈值 + 8 邻域爬线 + 角点检测，对光照变化更鲁棒。
6. **逆透视**：查表把原图坐标映射到鸟瞰坐标，再在中线上做纯跟踪。

目标是把这些思想完整落到本仓库，而不是引入第二套并行架构。

## 3. 总体设计

### 3.1 数据流

```
摄像头 640x480 BGR
  └─ 香橙派 OpenCV
       ├─ HSV/Gray 前端 → 边线 → 鸟瞰 → 中线
       ├─ 环岛/十字状态 → selectedCenter
       ├─ TYPE=1 道路观测（曲率/元素/置信度/距离）
       └─ TYPE=3 路径（车辆坐标系 0.05~2 m，≤16 点）
                 │ USART3 115200
                 ▼
STM32F407
  ├─ Car_Vision：解析 TYPE=1 / TYPE=3（独立序号）
  ├─ Car_Race：
  │    ├─ 环岛/十字状态机（元素 + 里程 + 置信度 + 帧去重）
  │    ├─ 路径纯跟踪（Car_Pursuit + Car_Steer）
  │    └─ 视觉曲率回退
  ├─ Car_Core：位置式/增量式速度环
  └─ PWM + 方向 → 双轮
```

### 3.2 控制优先级

```
十字直行 > 环岛曲率偏置 > 路径纯跟踪 > 视觉曲率
```

- 十字：`curve = 0`，强制直行；
- 环岛：在视觉/路径曲率上叠加 `±1/R`，偏置随 EXIT 相位线性衰减；
- 路径：`CarPursuit_Wheels()` 用虚拟舵机转角和自行车模型换算左右轮 rpm；
- 回退：路径无效或跟踪失败时，恢复原来的 `drive(speed, speed*curvature)`。

### 3.3 舵机到差速的适配

HFK 的转向执行器是舵机，本车是双轮差速。适配链：

```
纯跟踪曲率 κ
  → 目标转角 δ = atan(L·κ)
  → 虚拟舵机：乘增益、限幅、按 rate 逼近
  → 横摆角速度 ω = v·tan(δ)/L
  → v_left = v − 0.5·B·ω, v_right = v + 0.5·B·ω
  → rpm = v·60/(2πR)
```

`Car_Steer_PwmUs()` 仍保留舵机脉宽输出，未来改阿克曼底盘时只需接执行器，
上层不用改。

## 4. 固件变更明细

### 4.1 新增模块

| 文件 | 行数 | 说明 |
|---|---:|---|
| `CODE/Car_Steer.h/.c` | 276 | 虚拟舵机：增益、限幅、速率、脉宽、横摆角速度 |
| `CODE/Car_Pursuit.h/.c` | 296 | 纯跟踪：动态预瞄、曲率、限速、左右轮 rpm |
| `CODE/Car_Ring.h/.c` | 256 | 环岛四相位状态机 |
| `CODE/Car_Cross.h/.c` | 170 | 十字两相位状态机 |

### 4.2 `Car_Config.h`

新增两组参数，均带中文注释：

- 纯跟踪/舵机：轴距、轮距、轮半径、预瞄最小/增益/最大、转角上限、
  转角速率、横向加速度上限、绝对限速、虚拟舵机脉宽；
- 环岛/十字：确认帧数、置信度门限、各相位里程、环岛半径、速度缩放、超时。

新增 `CAR_SPEED_PID_INCREMENTAL`（默认 0），用于切换速度环实现。

### 4.3 `Car_Core`

- `Car_Core` 新增 `Car_PidInc_Struct pid_inc[2]`；
- `Car_Init`、`Car_SetPid`、`reset_motion` 同时维护位置式和增量式对象；
- `Car_TickDt` 在 `CAR_SPEED_PID_INCREMENTAL=1` 时改用
  `Car_PidInc_update_dt()`，故障检查也切换到对应对象；
- 默认值 0，主线行为不变。

### 4.4 `Car_Vision`

- `CAR_VISION_MAX_PAYLOAD` 从 32 扩到 68，最大帧 84 字节；
- 新增 `CarVisionPath`、`CarVision_GetPath()`；
- 新增 TYPE=3 解析：点数、flags、建议速度、最多 16 个车辆坐标系点；
- PATH 使用独立序号 `last_path_sequence`，避免与 TYPE=1 同帧互相判重；
- HELLO 和接收异常会同时作废旧路径。

### 4.5 `Car_Race`

- `Race_Step()` 新增 `const CarVisionPath *path` 参数（可为 NULL）；
- `CarRace` 内嵌 `CarPursuit`、`CarRing`、`CarCross`；
- 环岛/十字状态机只在 `RACE_TRACK` 运行，其他相位重置；
- 有路径时用纯跟踪输出；环岛/十字相位跳过纯跟踪，避免转角被覆盖；
- 输出前统一按 `CAR_RPM_LIMIT` 等比例限幅。

### 4.6 `Car_Hardware`

- `race_tick()` 每周期通过 `CarVision_GetPath()` 取路径快照；
- 视觉故障或路径过期时传 NULL，自动回退到视觉曲率。

### 4.7 Keil 工程

`MDK-ARM/*.uvprojx` 增加 4 个新源文件。固件编译结果：

```
0 Error(s), 0 Warning(s)
Program Size: Code=48604 RO-data=10212 RW-data=292 ZI-data=20252
```

## 5. 视觉变更明细

### 5.1 新增 `gray_track.hpp`

移植 HFK `Image.c`：

- `getPointThreshold()`：`block×block` 局部均值减 `cap` 的动态阈值；
- `getLStart()/getRStart()`：从底部向上扫描，取黑色边线内侧的白色起点；
- `getLBorder()/getRBorder()`：8 邻域方向状态机沿边线爬行；
- `findCorner()`：前后向量夹角找折点。

适配点：

- 原图 120×188 → 本工程 640×480；
- 用积分图一次算完整幅局部均值，避免逐点重复求和；
- 输出统一为本仓库的 `cv::Point(col, row)` 约定。

该模块当前是独立算法模块，未替换主处理链；接入前需要用实车录像校准
`block/cap`。

### 5.2 新增 `inverse_lut.hpp`

- 把现有 `calibration/UndistInverseMapW/H[480][640]` 打包成连续 LUT；
- 提供 `lookup()`、`mapPoints()`、`warpGray()`；
- 对应 HFK 的 `reflection` 表、`getGodRoadLine()`、`displayGodImage()`；
- 没有复用 HFK 的 120×188 LUT，因为相机和分辨率不同。

### 5.3 路径导出

`img_handle()` 在选中中线后：

1. 把鸟瞰点转换成车辆坐标系：`x=(px-300)·mpp`、`y=(460-py)·mpp`；
2. 过滤前方 0.05~2 m；
3. 按 4 cm 间隔去重，抽稀到最多 16 点；
4. 写入 `FrameResult::pathMeters` 和 `pathFlags`。

`main.cpp` 每帧在 TYPE=1 之后发送 TYPE=3；路径帧不影响原有 CSV 和调试输出。

## 6. 协议变更：TYPE=3 PATH

帧格式不变：

```
A5 5A | version:u8 | type:u8(=3) | length:u16 | seq:u16 | session:u32
      | age:u16 | payload | CRC16-CCITT-FALSE
```

负载：

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | count u8 | 1~16 |
| 1 | flags u8 | bit0 左、bit1 右、bit2 环岛、bit3 十字 |
| 2 | speed_mmps u16 | 0 表示不提供建议速度 |
| 4 | points | N × (x_mm i16, y_mm i16) |

- 坐标：车辆坐标系，x 右、y 前，原点为鸟瞰参考点 `(300,460)`；
- 长度必须等于 `4+4N`，坐标范围 ±20000 mm；
- PATH 在 F407 侧使用独立序号；
- 时效与 TYPE=1 相同：发送端年龄 + 本机接收后时间 ≤ 200 ms；
- 向后兼容：旧视觉不发送 TYPE=3，F407 自动回退。

## 7. 测试与验证

### 7.1 已执行

| 项目 | 结果 |
|---|---|
| Keil 固件编译（UV4 -b） | 0 Error / 0 Warning |
| Python 协议测试（19 项） | 全部通过 |
| `python Tools/vision_link.py --hex path` | 输出合法 TYPE=3 帧 |
| `git diff --check` | 通过 |

新增测试：

- `Tests/test_car_pursuit.c`：直道/左右弯、动态预瞄、限速、舵机速率、非法输入；
- `Tests/test_car_ring.c`：环岛方向/相位/超时、十字直行；
- `Tests/test_car_incremental.c`：`-DCAR_SPEED_PID_INCREMENTAL=1` 接线；
- `Tests/test_car_vision.c`：TYPE=3 解析、时效、长度错误；
- `vision_inspection/vision/test_gray_track.cpp`：动态阈值、爬线、角点、逆透视 LUT。

### 7.2 未执行

- **视觉 C++ 未在本机编译**：当前 Windows 环境没有 OpenCV 和主机 C++ 编译器。
  需要在香橙派/Linux 上运行 `bash Tools/check_race.sh`。
- **未上实车验证**：纯跟踪预瞄、环岛/十字里程、舵机速率等参数需要实车标定。
- **未运行主机 C 单元测试**：本机没有 clang/gcc；测试代码已写好，
  但需要在有主机编译器的环境执行 `bash Tools/check_race.sh`。

## 8. 兼容性与行为影响

| 项 | 影响 |
|---|---|
| 默认速度环 | 不变（位置式） |
| 无 TYPE=3 的视觉 | 不变，回退视觉曲率 |
| `Race_Step()` 签名 | 源级不兼容：新增 `path` 参数；仓库内调用和测试已更新 |
| `Car_Core` 结构体 | 增大，新增增量式 PID 对象 |
| `CarVision` 缓冲区 | 48 → 84 字节，Flash/RAM 略增 |
| 串口带宽 | 每帧最多多 84 字节；25 fps 下约 +2.1 KB/s |
| 视觉输出 | TYPE=1 内容不变，新增 TYPE=3 |

## 9. 实车迁移步骤

1. 先只烧固件、不发送 PATH，确认原巡线行为不变；
2. 香橙派运行新视觉，确认 TYPE=1 仍正常；
3. 打开 `--vision-uart`，观察 TYPE=3 帧和 `pathMeters`；
4. 低速验证纯跟踪方向：直道、左右弯、S 弯；
5. 标定 `CAR_PURSUIT_LOOKAHEAD_*`、`CAR_PURSUIT_STEER_RATE_RAD_S`；
6. 单独验证环岛：确认 `CAR_RING_RADIUS_M`、各相位里程；
7. 单独验证十字：确认 `CAR_CROSS_HOLD_M`；
8. 最后再切换 `CAR_SPEED_PID_INCREMENTAL=1` 做对比。

## 10. 已知限制与风险

1. **舵机是虚拟的**：本车没有舵机，`Car_Steer_PwmUs()` 未接执行器。
2. **环岛/十字为行为适配**：HFK 原实现依赖图像边线几何；本实现依赖视觉元素
   和里程，相位参数必须按实车赛道重新标定。
3. **逆透视未替换主链**：`inverse_lut.hpp` 是独立模块，主处理链仍使用
   `bird::transformPoints`。
4. **路径帧带宽**：当前每次最多 16 点、每帧发送一次；若串口压力大，可降频
   或减少点数。
5. **未实车验证**：所有参数默认值只是移植时的起点，不代表最终调参结果。

## 11. 审阅建议

建议重点审阅：

1. `Car_Pursuit.c` 的曲率符号和左右轮分解是否符合本车约定；
2. `Car_Race.c` 中环岛/十字与纯跟踪的优先级；
3. `Car_Vision.c` 中 PATH 独立序号的回绕和 HELLO 清理；
4. `Car_Core.c` 中增量式 PID 的故障与复位路径；
5. 香橙派侧 `img_handle.hpp` 的路径点顺序（近 → 远）和坐标符号；
6. TYPE=3 的协议字段是否满足后续扩展需求。

## 12. 文件清单

新增：

```
CODE/Car_Pursuit.h/.c
CODE/Car_Steer.h/.c
CODE/Car_Ring.h/.c
CODE/Car_Cross.h/.c
Docs/HFK_Port_PR_Report.md
Docs/Other_Branch_Port.md
Tests/test_car_pursuit.c
Tests/test_car_ring.c
Tests/test_car_incremental.c
vision_inspection/vision/gray_track.hpp
vision_inspection/vision/inverse_lut.hpp
vision_inspection/vision/test_gray_track.cpp
```

修改：

```
CODE/Car_Config.h
CODE/Car_Core.h/.c
CODE/Car_Hardware.c
CODE/Car_Race.h/.c
CODE/Car_Vision.h/.c
MDK-ARM/NEUQ_STM32F407VET6_Opensource_Library.uvprojx
Tests/test_car_race.c
Tests/test_car_vision.c
Tests/test_vision_link.py
Tools/check_car.ps1
Tools/check_race.sh
Tools/vision_link.py
vision_inspection/vision/build.sh
vision_inspection/vision/img_handle.hpp
vision_inspection/vision/main.cpp
vision_inspection/vision/test_race_camera.cpp
vision_inspection/vision/test_vision_link.cpp
vision_inspection/vision/vision_link.hpp
vision_inspection/vision/vision_serial.hpp
Docs/Vision_Interface.md
```

## 13. 验证命令

```bash
# 固件 + 主机测试（需要 clang/gcc，Linux/macOS）
bash Tools/check_race.sh

# 仅 Python 协议
python -m unittest discover -s Tests -p 'test_*.py' -v

# 视觉离线测试（需要 OpenCV）
cd vision_inspection/vision
bash build.sh test_gray_track
./test_gray_track

# Keil 编译（Windows）
cd MDK-ARM
UV4.exe -b NEUQ_STM32F407VET6_Opensource_Library.uvprojx -o build.log
```
