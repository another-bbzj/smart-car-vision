# PID 掉电保存

当前版本在左右轮 PID 子菜单中均提供第四项 `Save parameters`。任一入口都会保存左右两轮全部 6 个已生效参数，不需要分别保存两次。

## 调车操作

1. 停车，在 `Left wheel PID` / `Right wheel PID` 的 `Kp`、`Ki`、`Kd` 页面调值，按 PE7（OK）应用。`Active` 是当前值，`Edit` 是待应用值；PE9 返回会放弃尚未应用的编辑。
2. 调好两轮后，保持停车且车轮静止，进入任一 PID 子菜单的 `Save parameters`，再按 PE7 保存。
3. 等待 `Saved / reboot ready`。重启后程序会自动加载，保存页显示 `Loaded from Flash`，各 PID 页的 `Active` 显示恢复的数值。

PE10 上，PE8 下，PE7 确认，PE9 返回。运行中 PE10 仍用于停车。保存页仅接受单次确认，不因长按重复写入。保存条件为 STOP、左右测得轮速均小于 2 RPM、没有待完成的比赛结果处理。

| 显示 | 含义 |
|---|---|
| Using defaults / No saved parameters | 首次使用或没有有效记录，使用 Board_Config.h 编译默认值 |
| Loaded from Flash | 本次启动已恢复记录 |
| Unsaved changes | 当前已生效 PID 与上一份保存值不同 |
| Saving... keep power on | 正在写入，等待结果 |
| Saved / reboot ready | 写入和读回校验成功 |
| Already saved | 与已保存值一致，本次没有写入 |
| Save/read failed: retry | 存储读取、擦除、编程或校验失败，尚未确认本次保存成功 |
| Invalid PID values | 参数非法，拒绝写入 |
| Stop / finish result first | 先停车并等待车轮静止、结果处理完成 |

通过蓝牙上位机调好的 PID 也可用该菜单保存。蓝牙写 PID 命令仍只应用到 RAM，没有新增串口保存命令。只保存左右轮 Kp/Ki/Kd；轮径、减速比、限速、IMU 零偏等不在本次保存范围内。重启恢复参数后车辆仍保持 STOP。

## 存储实现和维护

- 外部 W25Q 的 SPI1 引脚与当前电机/编码器复用，因此采用 STM32 内部 Flash 的 S6/S7：`0x08040000..0x0807FFFF`，每区 128 KiB。扇区地址依据 ST [RM0090 参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf)。
- 当前 Keil 工程采用自动生成的链接布局，IROM/OCR_RVCT4 均限制为前 256 KiB；备用 `stm32f407_512k_app.sct` 也已排除 S6/S7。`Tools/check_pid_hex.py` 验证工程边界和最终 HEX 均未覆盖参数区。
- 每条记录 64 字节：标识、版本、递增序号、六个 float、CRC32、最后写入的提交标记。每区可追加 2048 条。写满后切换另一扇区；擦除目标区时，上一份有效记录所在区保持不动。
- 不完整记录和校验错误记录跳过；启动加载最新有效记录。没有有效记录时保留默认值。参数没有改变时不写 Flash，减少擦写。
- Flash 操作在主循环执行；保存期间控制中断保持 PWM 为零，不处理启动命令。保存结束清理接收积压、失效道路/陀螺仪快照，并丢弃第一份控制时差及编码器采样，随后重新接收新数据。
- 断电/复位可以保留已确认保存的参数；烧录时若选择整片擦除，也会擦掉参数。只擦除程序占用扇区可保留参数区。记录格式未来不兼容变更时应更新版本号。

## 验证范围

已加入 Flash 模型测试：首次保存中断、两轮六参数重启恢复、无变化不写、追加写入各阶段断电、CRC 损坏回退、双区满后切换、目标区擦除中断、非法数值及读取失败；菜单测试覆盖第四项、保存条件和长按不重复触发。

`Tools/check_car.ps1` 执行主机回归、Keil 全量编译和 HEX 地址检查。尚未连接实车烧录，也未在物理 Flash 上做断电测试。
