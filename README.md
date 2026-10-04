# MuJoCo 二维小车倒立摆仿真

基于 MuJoCo 的水平面二维移动小车 + 空间倒立摆仿真程序，带 GLFW 实时可视化窗口，内置 **PID** 与 **LQR** 两种控制器，用于平衡控制算法的学习与验证。

- 小车可在 X、Y 两个水平方向独立平移（无车轮、高度固定、无转动）
- 摆杆通过两个相交铰链形成万向铰，可向任意方向倾倒
- 控制器根据摆杆倾角与角速度输出两个水平推力，使摆杆保持直立

## 目录结构

```
mujoco/
├── CMakeLists.txt        # 构建脚本
├── cart_pole.xml         # MuJoCo 模型：几何、动力学、执行器、传感器、关键帧
├── include/
│   ├── controller.hpp    # Controller 基类 + PIDcontroller / LQRcontroller 声明
│   └── simulation.hpp    # Simulation 类声明（模型加载、步进、渲染、事件回调）
├── src/
│   ├── main.cpp          # 主循环：初始化控制器、200 Hz 控制、60 Hz 渲染
│   ├── controller.cpp    # PID 控制律与 LQR（ZOH 离散化 + DARE 迭代求解）
│   └── simulation.cpp    # MuJoCo / GLFW / 鼠标键盘交互实现
└── build/                # CMake 构建目录（Ninja）
```

## 依赖

| 依赖 | 说明 |
| --- | --- |
| CMake ≥ 3.16 | 构建工具 |
| C++17 编译器 | 如 GCC / Clang |
| MuJoCo 3.14.0 | 默认取 `~/miniconda3/envs/robot/lib/python3.12/site-packages/mujoco/`，可通过 `-DMUJOCO_ROOT=<路径>` 覆盖 |
| GLFW3 | 窗口与输入（`find_package(glfw3)`） |
| Eigen3 | 矩阵运算（`find_package(Eigen3)`） |

## 构建

```bash
cmake -S . -B build -G Ninja
cmake --build build
```

可执行文件输出到项目根目录 `./simu_cartPole`。

若 MuJoCo 安装在其他位置：

```bash
cmake -S . -B build -G Ninja -DMUJOCO_ROOT=/path/to/mujoco/
```

## 运行

**必须在项目根目录下运行**（`MODEL_PATH` 为相对路径 `cart_pole.xml`）：

```bash
./simu_cartPole          # 默认 PID 控制器
./simu_cartPole --PID    # 显式指定 PID
./simu_cartPole --LQR    # 使用 LQR 控制器
```

| 操作 | 功能 |
| --- | --- |
| `Backspace` | 重置仿真到 `initial_tilt` 关键帧，并清空控制器积分状态 |
| 鼠标左键拖动 | 旋转视角 |
| 鼠标右键拖动 | 平移视角（按住 `Shift` 切换水平/竖直方向） |
| 鼠标中键拖动 / 滚轮 | 缩放 |

运行中每个控制周期会在终端打印一行当前状态与控制量：
`[PID] state=[...] control=[...]`（LQR 同理）。

## 模型说明（cart_pole.xml）

- 自由度：`nq = nv = 4`，`qpos = [x, y, alpha, beta]`，`qvel = [vx, vy, alpha_dot, beta_dot]`
- 物理步长 1 ms（1000 Hz），积分器 `implicitfast`，无碰撞、无关节摩擦
- 摆杆为均匀圆柱：长 1 m、质量 0.1 kg，质心距铰链 0.5 m；小车质量 1 kg
- 执行器：`force_x`、`force_y` 两个直驱推力，各自限幅 ±20 N
- 传感器（`get_state()` 使用前 8 项）：
  `[x, y, vx, vy, alpha, beta, alpha_dot, beta_dot]`，另含摆杆方向与世界角速度（`sensordata[8:14]`）

关键帧：

| 名称 | 说明 |
| --- | --- |
| `upright` | 小车在原点，摆杆精确直立 |
| `initial_tilt` | **程序启动与重置使用的关键帧**：小车位于 `(-2, -2)` m，alpha = beta = 25°（总倾角约 34.8°） |
| `tilt_x` | 仅向 +X 倾斜 5° |
| `tilt_y` | 仅向 +Y 倾斜 5° |

## 控制器

控制周期为 5 ms（200 Hz，即每 5 个物理步更新一次推力），参数在 `src/main.cpp` 中配置。

### PID（默认）

控制律（`src/controller.cpp`）：

```
u[i] = kp * angle[i] + ki * ∫angle[i] dt + kd * angle_dot[i]
```

- 反馈量：摆杆两轴倾角 `alpha, beta` 与角速度 `alpha_dot, beta_dot`
- 积分限幅 ±20，输出限幅 ±20 N
- 当前整定参数（`src/main.cpp`）：**KP = 40.0，KI = 10.0，KD = 6.0**

实测行为：从 `initial_tilt`（25° + 25°）出发，约 2 s 内摆杆恢复直立（残余角 ≈ 0），峰值推力约 17.7 N，未触发限幅。

> 注意：PID 仅反馈摆杆角度，没有小车位置/速度反馈。摆杆稳住后小车会以约 0.2 m/s（Y 向约 0.28 m/s）匀速滑行，PID 参数无法消除该现象，需要位置反馈时请使用 LQR 或扩展控制器结构。

### LQR

- 连续模型线性化后经零阶保持（ZOH）离散化，用迭代法求解离散代数 Riccati 方程（DARE）得到反馈增益 `K`
- 性能指标矩阵在 `src/main.cpp` 中配置，当前为
  `Q = diag(25, 25, 4, 4, 1459, 1459, 1, 1)`，`R = diag(0.0025, 0.0025)`
- 全状态反馈，可在平衡摆杆的同时把小车位置与速度调节回零

