# 校赛香橙派视觉程序

本目录保留已有道路曲率和路障距离发送，合入本次提供的 vision1.2.rar 数独模板识别、四块拼接及求解模块。车载启动、引脚、标定表和验收步骤见[项目交接文档](../../Docs/Race_Handover.md)。

视觉与控制双方的开发分工、协议和联调步骤见 [vision1.2 视觉组交接文档](../../Docs/Vision12_Handover.md)。

在香橙派上安装含 aruco 的 OpenCV 4 开发库后，在本目录编译。本地历史可执行文件可能来自其他机器，部署时重新构建。

```bash
bash build.sh vision
bash build.sh test_sudoku
./test_sudoku
./vision --video test_fixtures/sequence.avi
./vision --image test_fixtures/straight.png --output straight-inspection
```

实时模式使用 640×480 摄像头；将实际串口设备传给 `--vision-uart`，F407 停车时加 `--hello` 建立新会话。几何在鸟瞰坐标中先按向右为正计算，再在串口边界转成向左为正。近端蓝底缺路只构成候选，路线位置、过断路距离、航向和回线确认由 F407 判断。

`--show` 打开本地预览，`--tcp` 启用诊断图服务器。无显示器的香橙派用 `--http`（默认端口 8080）启动浏览器查看：程序运行后在同一局域网的电脑打开 `http://<板子IP>:8080/`，页面同时显示 Camera 原始画面、鸟瞰/预瞄、巡线、白色掩膜、蓝底掩膜和路障图，约 10 fps；`/snapshot?name=Camera` 可取单张 JPEG，端口冲突时用 `--http-port N`。CSV 中的 `gap_phase` 与 `obstacle_phase` 属于 V1.1 的视觉侧策略模型；实车动作由 F407 的 `Race_Step` 执行。运行工程根目录的 `bash Tools/check_race.sh` 可重建并检查视觉、固件算法、伪串口和录像回放。

数独每隔一帧处理一次，读取 Tag 0/1、2/3、4/5、6/7 对应的左上、右上、左下、右下九格。利用网格外框和 Tag 对角顺序矫正，标签外边距不会被纳入九格。字形模板在 Reader 初始化时缓存，各块分别连续两次一致才确认，同帧可确认多块。低置信度与非法识别结果会阻止该块发送。实际赛题字体的识别率仍需实拍样片验证。

摄像头模式下发送四个原始 3×3 子块，F407 完赛停车后拼接求解并显示。图像侧的 `sudoku::solve()` 可用于离线验证，检查给定值与唯一性并限制搜索量。指定 `--output` 后额外保存 `sudoku.csv`，每行包含来源帧、块号、九格值、处理年龄和是否发送。离线回放只记录识别结果，`sent` 为 0。
