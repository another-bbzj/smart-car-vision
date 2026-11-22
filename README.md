# smart-car-vision



*东北大学秦皇岛分校智能车实验室 · NEUQ Smartcar*

当前固件支持 **车载按键自主比赛和蓝牙手动调试**。香橙派视觉程序在 `vision_inspection/vision`，STM32 执行左右轮速度闭环和赛项动作。编译、发车、回放、实测参数回填及现场未公布项目见 [比赛交接文档](Docs/Race_Handover.md)。

与图像组联调请先阅读 [vision1.2 视觉组交接文档](Docs/Vision12_Handover.md)，包含四个 3×3 数独子块的编号、串口协议、运行入口和验收步骤。

控制开发人员协作请阅读 [下位机控制代码开发交接](Docs/Control_Development_Handover.md)，包含代码地图、调度与接口、硬件资源、比赛状态机、测试和后续任务。左右轮 PID 已支持菜单手动保存、重启自动恢复，操作见 [PID 掉电保存](Docs/PID_Storage.md)。

接线、编译烧录、蓝牙配对和首次校准见 [蓝牙小车使用说明](Docs/Bluetooth_Car.md)。启动电脑驾驶工具：

```powershell
python -m pip install -r Tools/requirements-car.txt
python Tools/car_host.py
```

底盘硬件与 PID 参数在 `CODE/Board_Config.h`，轮径、轮距、赛道里程、绕障方向及练习终点在 `CODE/task/Task_Fsm.h`。

也可双击 `Tools/car_console.bat` 使用网页调试台（Python 3.8+、桌面版 Edge/Chrome，无需额外 Python 依赖）。启动器默认用 Chrome 打开本机页面（未安装时回退 Edge）；在 Windows 配对蓝牙后选择出站 SPP 串口，保持默认9600并等待停止回传，再勾选安全确认、使能。支持实时曲线、内外环分区 PID 调整（数字框+滑块）、内环自动调参和 CSV 导出。自动调参通过命令 6 做开环 PRC 并拟合 FOPDT，只在车轮架空时使用；外环尚未开放，详见[内环自动调参](Docs/Auto_Tuning.md)。网页的蓝牙连接独立于 BAT 服务，结束时先在网页停止并断开。用法见 [网页调试台说明](Tools/car_console_README.md)。

**在线版（无需下载、无需 Python）：** <https://another-bbzj.github.io/smart-car-bluetooth/> —— 桌面版 Chrome/Edge 117+ 打开，串口权限在各自电脑上授权。

独立的 USART3 视觉串口使用 PB10/PB11，115200。比赛模式消费道路曲率、赛道元素、路障距离及四份数独子块。帧格式和 Python 参考发送器见 [图像通信接口](Docs/Vision_Interface.md)。

速度控制已加入实际采样时间、测量微分滤波、抗积分饱和、可标定前馈和双轮共享进度的加速度/加加速度受限参考，详见[速度控制升级](Docs/Speed_Control_Upgrade.md)。尚未进行实车高速过弯验证。

## 项目贡献者

- [邹俊（another-bbzj）](https://github.com/another-bbzj)

依赖库原有版权与许可声明保留在对应源码中。
