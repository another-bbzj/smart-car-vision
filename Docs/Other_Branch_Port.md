# `other` 分支：HFK_STC32G144K 元素移植说明

本分支把 `D:\NEUQCAR\HFK_STC32G144K` 的纯跟踪、舵机模型、增量式 PID、
环岛、十字、灰度巡线和逆透视算法移植到 W 车，并按 W 的实际架构（STM32F407
差速底盘 + 香橙派视觉）做适配。所有改动只存在于 `other` 分支。

## 1. 架构差异与适配原则

| HFK | W | 适配方式 |
|---|---|---|
| STC32G 单片摄像头，120×188 灰度 | 香橙派 OpenCV，640×480 BGR | 灰度算法在香橙派 C++ 侧实现 |
| 舵机转向（阿克曼） | 双轮差速，无舵机 | `Car_Steer` 保留舵机模型，输出转角后经自行车模型换算横摆角速度，再分解成左右轮 rpm |
| 图像坐标纯跟踪 | 视觉只发曲率/元素 | 新增 TYPE=3 路径帧，F407 用车辆坐标系路径做纯跟踪 |
| 速度环增量式 PID | 位置式 PID | 增加 `CAR_SPEED_PID_INCREMENTAL` 开关，两种实现都保留 |
| 环岛/十字靠图像边线几何 | 视觉已输出 RING/CROSS 元素 | F407 侧状态机改为“元素 + 里程 + 置信度 + 去重序号”驱动 |

## 2. 固件新增模块

| 文件 | 功能 |
|---|---|
| `CODE/Car_Steer.h/.c` | 虚拟舵机：增益、限幅、速率限制、脉宽输出、横摆角速度换算 |
| `CODE/Car_Pursuit.h/.c` | 纯跟踪：动态预瞄、`κ = 2y/(x²+y²)`、曲率限速、左右轮 rpm |
| `CODE/Car_Ring.h/.c` | 环岛四相位（ENTER/INSIDE/EXIT），输出曲率偏置 |
| `CODE/Car_Cross.h/.c` | 十字两相位，路口内强制直行 |
| `CODE/Car_Vision.h/.c` | 新增 TYPE=3 PATH 帧解析与独立序号缓存 |
| `CODE/Car_Core.h/.c` | 增加增量式速度环对象与编译期选择 |
| `CODE/Car_Race.c` | 路径优先于视觉曲率；叠加环岛/十字输出 |

`Car_Config.h` 新增全部默认参数，换底盘或重新标定后须整组复核。

### 2.1 控制优先级

```
十字直行 > 环岛偏置 > 路径纯跟踪 > 视觉曲率
```

有有效路径时，`Race_Step` 用 `CarPursuit_Wheels()` 输出左右轮 rpm；
路径丢失或无效时退回原来的 `drive(speed, speed * curvature)`。

### 2.2 增量式速度环

`CAR_SPEED_PID_INCREMENTAL 0`（默认）保持位置式，行为与主线一致；
改为 1 后使用 `Car_PidInc_update_dt()`。两种对象的增益在 `Car_Init`、
`Car_SetPid` 和 Flash 参数应用时保持同步。

## 3. 视觉串口协议扩展：TYPE=3 PATH

帧格式仍为：

```
A5 5A | version:u8 | type:u8(=3) | length:u16 | seq:u16 | session:u32
      | age:u16 | payload | CRC16-CCITT-FALSE
```

负载：

```
count:u8 | flags:u8 | speed_mmps:u16 | count × (x_mm:i16, y_mm:i16)
```

- 坐标：车辆坐标系，x 向右、y 向前，原点为视觉参考点（鸟瞰 `(300,460)`）；
- `count`：1~16；
- `flags`：bit0 左侧、bit1 右侧、bit2 环岛、bit3 十字；
- `speed_mmps`：0 表示不提供建议速度；
- PATH 使用独立序号，避免与 TYPE=1 同帧发送时互相判重；
- 时效与 TYPE=1 相同：发送端年龄 + 本机接收后时间不超过 200 ms。

## 4. 视觉侧新增文件

| 文件 | 内容 |
|---|---|
| `vision_inspection/vision/gray_track.hpp` | HFK 动态局部阈值、起点搜索、8 邻域边线爬行、角点检测 |
| `vision_inspection/vision/inverse_lut.hpp` | 把现有 `UndistInverseMapW/H` 打包成逆透视 LUT，提供逐点/整图映射 |
| `vision_inspection/vision/vision_link.hpp` | 新增 `encodePath()` 与 `PathObservation` |
| `vision_inspection/vision/vision_serial.hpp` | 新增 `sendPath()` |
| `vision_inspection/vision/img_handle.hpp` | 选中中线转成车辆坐标系路径，写入 `FrameResult::pathMeters` |
| `vision_inspection/vision/main.cpp` | 每帧在 TYPE=1 之后发送 TYPE=3 |

`gray_track.hpp` 与 `inverse_lut.hpp` 是独立算法模块，当前未替换主处理链；
接入前应先用实车录像校准 `block/cap` 和逆透视表。

## 5. 测试

固件（需要主机 clang/gcc）：

```bash
bash Tools/check_race.sh
```

新增测试：

- `Tests/test_car_pursuit.c`：直道/左右弯、动态预瞄、限速、舵机速率、非法输入；
- `Tests/test_car_ring.c`：环岛方向与相位、超时、十字直行；
- `Tests/test_car_incremental.c`：`-DCAR_SPEED_PID_INCREMENTAL=1` 下的速度环接线；
- `Tests/test_car_vision.c`：TYPE=3 PATH 帧解析与时效。

视觉侧：

```bash
cd vision_inspection/vision
bash build.sh test_gray_track
./test_gray_track
```

协议参考：

```bash
python Tools/vision_link.py --hex path
python -m unittest discover -s Tests -p 'test_*.py'
```

## 6. 已知限制

1. W 车没有舵机，`Car_Steer_PwmUs()` 只提供接口，未接执行器；实车仍是差速。
2. 环岛/十字的触发条件已从图像边线几何改为视觉元素，行为不等价于 HFK 原车；
   相位里程/超时参数必须按实车赛道重新标定。
3. 逆透视使用 W 现有标定表，未复用 HFK 的 `reflection` 表（相机和分辨率不同）。
4. 视觉侧 C++ 改动未在本地编译验证：当前 Windows 环境没有 OpenCV 和主机
   C++ 编译器；请在香橙派或 Linux 环境运行 `Tools/check_race.sh`。
5. 固件已用 Keil (`UV4 -b`) 编译通过，0 error / 0 warning，但未上实车验证。
