<!-- 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9 -->
# RoBoBo V3.9 — ATOM S3R

V3.9 packages the working four-mode robot firmware and low-latency Mac MuJoCo pose mirror. Pose return is fixed at a **200 Hz target**; the switch is off on every boot. Models and motion control are unchanged from V3.8. No retraining is claimed.

作者：**同济子豪兄 2026-10-26**。原始机器人设计与项目：**HomeMadeGarbage**。此署名指本目录的新增策略、改编代码和版本集成，不替代原作者及依赖库的署名。

模型 ZIP 内含 `AUTHOR.txt`，权重张量未改动；继承的 STL 用相邻署名文件保留原作者信息。

## 包含内容

- `firmware/robo03_atom_s3r_v39/`：完整 Arduino 固件、导出的策略头文件、网页和四套随机配色图标。
- `models/`：起身 PPO、Walk 残差 PPO、侧翻 BC/PPO 权重，以及固定步态参数。
- `pc_viewer/`：MuJoCo XML、全部网格、黑地面网格样式、实时查看 Python 和 Mac 启动器。
- `build/`：可烧录二进制和 flash 地址说明；不含编译缓存。
- `manifest.json`：文件 SHA-256；`validation.json`：模型与 C 头文件推理一致性结果。

## Mac 实时查看

首次安装（建议使用 Python 3.14，本发布验证版本）：

```sh
cd zihao_s3r_v39/pc_viewer
sh setup_viewer.sh
```

之后每次只需要：

1. 在网页 `http://192.168.42.1/` 打开“姿态同步回传电脑”。
2. Mac 连接 `RoBoBo` 热点。
3. 双击 `pc_mujoco_viewer.command`，或运行 `sh pc_mujoco_viewer.command`。

安装依赖需要网络，建议在连接机器人热点之前完成。Mac 若询问本地网络权限，请允许。关闭旧查看器或占用 UDP 4242 的测速程序再启动。

Python 入口是 `live_robot_viewer.py`；macOS 图形窗口需通过 `.venv/bin/mjpython live_robot_viewer.py` 启动。Linux 可以使用 `.venv/bin/python live_robot_viewer.py`。可用 `--check` 无窗口检查模型和数据格式，`--no-smoothing` 关闭轻微平滑。

查看器自动每秒发送订阅，固件向最近订阅者单播；无订阅时兼容广播。独立线程收包，仅保留最新有效数据；窗口目标 60Hz，平滑时间常数 5ms，无历史动画缓存。断流一秒显示 STALE 并冻结。只读显示，不发送运动控制。一次使用一个查看器。

## 频率与显示含义

| 通道 | 设置 |
|---|---|
| Wi-Fi UDP 姿态回传 | 固定目标 200Hz；实际速率受循环和网络影响 |
| 网页 `/data` 获取姿态及舵机目标 | 100ms 一次，最多约 10Hz；控制请求忙时跳过 |
| 网页曲线重绘 | 约 25Hz，使用平滑值，并非 25Hz 新测量 |
| 策略控制 | 约 50Hz |
| Mac 画面更新 | 目标 60Hz，实际取决于显示和系统负载 |

在之前相同单播传输方案的 Jetson 20秒测试中，200Hz 设置加 10Hz 网页取数，实收约175Hz，无序号缺口。这不是 Mac 端到端延迟测量，也不是长期零丢包保证。用户反馈 Mac 联动画面可用。

姿态来自 IMU 四元数，关节来自**目标角**，不是实际角反馈。XY 固定，贴地高度仅为显示；不反映真实平移、跳跃高度或舵机滞后。Yaw 是相对估计，会漂移。

## 硬件与模式

M5Stack ATOM S3R；GPIO1：下方靠近底板舵机 ±90°；GPIO2：上方舵机 ±120°。沿用已验证的 IMU 轴序和负向 PWM 映射。舵机使用独立合适的 5V 供电并共地；角度与脉宽尚未实测标定。

- 雄起机：默认模式，43°触发、300ms等待，站稳后保留回零顿挫。
- 不倒翁：V3.1 起身策略持续控制，无角度触发等待。
- 蛄蛹者：步态加残差策略及侧翻选择器，七按键控制；不是所有姿态和方向都可靠。
- 动作：点头、鞠躬等预设动作。

短按屏幕：慢慢回零暂停；再按回雄起机。长按切换雄起机/不倒翁。暂停红屏。开机随机红、蓝、绿、金色网页图标。

## 编译和烧录

验证依赖：m5stack ESP32 core 3.3.8，M5Unified 0.2.23，M5GFX 0.2.30，ESP32Servo 3.2.1，Kalman Filter Library 1.0.2。

```sh
arduino-cli compile --fqbn m5stack:esp32:m5stack_atoms3 --build-path /tmp/robobo-v39 firmware/robo03_atom_s3r_v39
arduino-cli upload --fqbn m5stack:esp32:m5stack_atoms3 --port YOUR_SERIAL_PORT --input-dir /tmp/robobo-v39 firmware/robo03_atom_s3r_v39
```

从本目录执行。更新固件后刷新网页，避免旧页面缓存。启动会进入雄起机，烧录前给机器人留出活动空间。

## 模型来源

- `recovery_ppo.zip`：`ppo_scratch_long_seed11/best.zip`，4→64→64→2，V3/V3.1 原起身策略。
- `walk_residual_ppo.zip`：`multipose_conservative/final.zip`，16→64→64→2；固定步态加局部残差。
- `roll_bc.zip`：`roll_bc/bc.zip`，13→64→64→2。
- `roll_sde.zip`：`roll_sde/final.zip`，13→64→64→2。

侧翻调用分派及固定步态在头文件中保留。模型 ZIP 用于离线研究；烧录实际嵌入 C 头文件中的 actor 权重，ZIP 不会以文件系统形式存入设备。未发布训练缓存或无关实验模型。
