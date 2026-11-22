当前目录只包含蓝牙双电机差速车的业务代码，按下列 7 类模块组织，平台入口与板级配置单独列出。

Board_Config.h                    板级基准：控制周期、串口分配、转速/PWM 满量程与 90% 安全限幅
menu/Menu_Task.c/.h               菜单：三级导航、调参候选值、发车/保存动作请求、逐行格式化
driver/Drv_Motor.c/.h             电机驱动：PWM、方向与输出提交
driver/Drv_Encoder.c/.h           编码器驱动：正交计数、方向符号与速度采样
driver/Drv_Key.c/.h               按键驱动：消抖与按键事件
driver/Drv_Lcd.c/.h               LCD 驱动：缓存与逐字符/逐行刷新
driver/Drv_Gyro.c/.h              陀螺仪驱动：MPU6050 初始化、Z 轴零偏与角速度
driver/Drv_Clock.c/.h             时基驱动：TIM5 微秒计时与 5 ms 控制节拍
driver/Drv_Uart.c/.h              串口驱动：蓝牙/视觉串口与 RX FIFO 接入
protocol/Proto_Bluetooth.c/.h     蓝牙协议：帧解析、轮速命令与状态应答
protocol/Proto_Vision.c/.h        视觉协议：观测/元素/数独子块接收、校验、会话与过期检查
protocol/Proto_Vofa.c/.h          JustFloat 遥测：16 通道回传缓冲与分次发送
pid/Pid.c/.h                      PID 算法：位置式/增量式、微分滤波、抗饱和与前馈
pid/Pid_Storage.c/.h              PID 存储：记录格式、CRC、Flash 读写与上电恢复
sudoku/Sudoku.c/.h                数独：四块拼接、6×6 唯一解检查
task/Task_Fsm.c/.h                任务状态机：整车状态、目标来源(motion)、保护/故障与终点收尾
track/Track_Fsm.c/.h              巡线状态机：曲率跟踪、赛段切换（断路/绕障/红灯/完赛）与速度规划
platform/Platform_Main.c/.h       平台入口：外设初始化、控制节拍、前台任务与共享快照

入口链：User/NEUQ_Main.c -> Platform_Main -> Car_Main
操作与接线：Docs/Bluetooth_Car.md
图像接口：Docs/Vision_Interface.md；尚未实现图像算法或自主状态机