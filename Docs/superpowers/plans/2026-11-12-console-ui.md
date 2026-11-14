# 调试台界面与曲线选择实施计划

目标：按用户选择的浅色实验台风格，调整白灰底、留白与信息层级；默认打开 Chrome；支持六条曲线独立显示。

实现范围：保留单页 HTML、Canvas 与 Web Serial，保持原有车辆协议和安全逻辑。Chrome 查找覆盖 PATH 与常见安装目录，找不到时回退 Edge。

- [x] 用测试验证 Chrome 优先级，再修改 Tools/car_console_server.py。
- [x] 在 Tests/test_car_console.cjs 增加显示选择、隐藏后缩放、全部隐藏和采集不受影响测试。
- [x] 修改 Tools/car_console.html：浅色双栏布局、六路选择与四个快捷预设；坐标范围仅计算可见曲线。
- [x] 更新使用说明，运行 JS/Python 回归，在 Chrome 渲染桌面和窄屏截图检查布局。

曲线：左目标、左实测、右目标、右实测、左 PWM、右 PWM。轮速与 PWM 保留各自坐标；默认全部显示；空选择明确提示。选择仅影响绘图，CSV 始终包含全部通道。
