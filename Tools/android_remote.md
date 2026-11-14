# 安卓手机蓝牙遥控

手机安装 RoboRemo Demo，在系统蓝牙中配对 JDY-31-SPP（默认 PIN 为 1234）。先在电脑调试台断开串口，然后在手机 App 的 Menu → Connect 里选择 Bluetooth RFCOMM 和已配对的小车。免费版每个面板支持五个控件，正好可以放一个使能按钮和四个方向按钮。[官方版本说明](https://roboremo.app/pricing)

进入 Menu → Edit UI，添加五个 Button，把动作类型设为 **Local**。按下和松开的内容可从下表直接复制。方向键开启 Repeat，只重复 Press，首次延迟和重复周期都设为 **100 ms**。使能键关闭 Repeat，面板无需添加独立心跳。[官方操作手册，第 11–12 页](https://roboremo.app/roboremo_app_manual.pdf)

| 按钮 | Press（按下） | Release（松开） |
|---|---|---|
| 清故障并使能 | `sendhex C33C02002AC33C010015` | 留空 |
| 前进 | `sendhex C33C0304002800283E` | `sendhex C33C02002A` |
| 后退 | `sendhex C33C0304FFD8FFD88A` | `sendhex C33C02002A` |
| 原地左转 | `sendhex C33C0304FFD8002883` | `sendhex C33C02002A` |
| 原地右转 | `sendhex C33C03040028FFD837` | `sendhex C33C02002A` |

退出编辑模式，首次架空车轮。点一次“清故障并使能”，随即按住一个方向键（500 ms 内）。预设轮速为 40 rpm。松开方向键会发送 STOP 并取消使能，下次运动前再点使能。如果两个按键之间超过 500 ms，固件会因通信超时停车，重新点使能即可。遇到堵转或方向异常，先检查车轮和接线再清故障。

使能键的两个连续帧依次执行 STOP 和 ARM，目标从零开始。按键数据由本项目的 `car_host.py` 编码器生成。此面板用于触控驾驶，电脑调试台提供遥测曲线和 PID 调参。
