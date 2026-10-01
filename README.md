# 水平二维移动小车与空间倒立摆

`cart_pole.xml` 已改为小车在 XY 水平面内平移、摆杆可向任意方向倾倒的模型，保留逐项中文注释。世界 Z 轴竖直向上。

小车用两个 `slide` 关节约束在固定高度，姿态固定。摆杆通过一个 `ball` 球铰连接小车。球铰允许三个转动自由度，包含两个倾倒方向和绕杆轴的自转，因此模型有 **5 个物理自由度、2 个控制输入**。球铰姿态用单位四元数表示，具体定义见 [MuJoCo 关节文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint)。

| 参数 | 数值 |
| --- | --- |
| 小车质量 / 总尺寸 | 1 kg / 0.40 × 0.40 × 0.12 m |
| 摆杆质量 / 全长 / 半径 | 0.1 kg / 1 m / 0.02 m |
| 球铰到摆杆质心距离 | 0.5 m |
| 球铰初始高度 | 0.20 m |
| 重力加速度 | 9.81 m/s² |
| 物理步长 | 0.001 s，即 1000 Hz |
| 每个方向的推力范围 | -20 至 20 N |

小车是理想 XY 移动平台，没有车轮、碰撞、摩擦、阻尼或行程端挡。地面与坐标轴标记仅用于显示；红色标记表示 +X，绿色标记表示 +Y。摆杆为具有转动惯量的均匀圆柱，倒下时可以穿过地面。

## 查看模型

在项目目录运行：

```bash
conda activate robot
python -m mujoco.viewer --mjcf=cart_pole.xml
```

选择 `overview` 相机观察空间运动；`front`、`side` 分别观察两个竖直投影，`top` 观察 XY 运动。默认状态精确直立且推力为零；选择 `initial_tilt` 关键帧可从朝 +X、+Y 倾斜 5° 的状态开始。`tilt_x`、`tilt_y` 分别提供单方向初始扰动。

## 输入和状态

输入现为两个推力分量，而不是一个标量：

```cpp
d->ctrl[0] = force_x;  // Fx，单位 N，正值沿世界 +X。
d->ctrl[1] = force_y;  // Fy，单位 N，正值沿世界 +Y。
mj_step(m, d);        // 推进 0.001 s。
```

两个分量各自限幅到 ±20 N；合力大小不是限制在 20 N 内。两个方向均通过小车运动间接驱动摆杆。

| 数据 | 内容 |
| --- | --- |
| `qpos[0:2]` | 小车位置 x、y，单位 m |
| `qpos[2:6]` | 球铰四元数 qw、qx、qy、qz |
| `qvel[0:2]` | 小车速度 vx、vy，单位 m/s |
| `qvel[2:5]` | 球铰局部角速度，单位 rad/s |
| `ctrl[0:2]` | 小车推力 Fx、Fy，单位 N |

区间采用 Python 切片记法，右端索引不包含在内。C++ 需要逐项索引。

四元数的四个分量满足单位长度，所以 `nq=6`、`nv=5`，位置数组和速度数组长度不同。旧模型的 `qpos[1]` 现在表示小车 y 位移，不能再作为摆角。旧的单输入控制器也需要改为输出两个推力。

## 传感器

当前传感器均输出理想量，布局如下：

| 名称 | sensordata 区间 | 输出 |
| --- | --- | --- |
| cart_position_x / cart_position_y | [0:2] | x、y |
| cart_velocity_x / cart_velocity_y | [2:4] | vx、vy |
| pole_orientation | [4:8] | qw、qx、qy、qz |
| pole_direction | [8:11] | 杆轴世界方向 nx、ny、nz |
| pole_world_angular_velocity | [11:14] | 世界角速度 wx、wy、wz |

杆方向 `n=[nx, ny, nz]` 从球铰指向杆顶。`n=[0,0,1]` 对应直立；nx 为正表示朝 +X 倾倒，ny 为正表示朝 +Y 倾倒。它不受杆轴自转影响，适合用于平衡控制。

在直立附近，可用投影倾角 `theta_x=atan2(nx,nz)`、`theta_y=atan2(ny,nz)` 表示两个倾倒方向；这两个量用于直立附近控制，不是描述全部三维姿态的一组全局欧拉角。实际球铰姿态仍保留四元数。

C++ 读取示例：

```cpp
mj_resetDataKeyframe(m, d, 1);  // 朝 +X、+Y 的对角方向倾斜 5°。
mj_forward(m, d);

double x = d->sensordata[0];
double y = d->sensordata[1];
double nx = d->sensordata[8];
double ny = d->sensordata[9];
double nz = d->sensordata[10];
// 世界角速度位于 sensordata[11]、[12]、[13]。
// 控制器使用采样后的测量计算 force_x、force_y。
```

## 采样与控制周期

物理仿真为 1000 Hz。当前 C++ 草稿中的 `SENSOR_FREQUNCY=100` 可对应每 10 个物理步采样一次；使用此前建议的 200 Hz 时，每 5 个物理步采样一次。采样时更新测量和控制输出，两次更新之间保持上一组推力。采样时刻按仿真步数安排，显示刷新率可独立设为 60 Hz。

以下 Python 示例演示 100 Hz 采样和推力保持，使用零推力观察自然倒下，尚未加入平衡反馈：

```python
import mujoco

model = mujoco.MjModel.from_xml_path("cart_pole.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 1)
mujoco.mj_forward(model, data)

control_hz = 100  # 可改成 200。
steps_per_control = round(1.0 / (control_hz * model.opt.timestep))
assert abs(steps_per_control * model.opt.timestep - 1.0 / control_hz) < 1e-12

for step in range(1000):  # 1 秒物理时间。
    if step % steps_per_control == 0:
        mujoco.mj_forward(model, data)  # 更新到当前物理状态。
        measured = data.sensordata.copy()
        # 在此加入噪声、量化、延迟及速度估计，再交给控制器。
        # 将下面的零推力替换为控制器输出的 [Fx, Fy]。
        data.ctrl[:] = [0.0, 0.0]
    mujoco.mj_step(model, data)  # 采样间隔内 ctrl 保持不变。

print("小车位置 [x, y]:", data.qpos[:2])
mujoco.mj_forward(model, data)
print("杆的世界方向:", data.sensordata[8:11])
```

若现实设备只测位置或 IMU 原始量，应由这些采样量估计速度和姿态，避免直接把理想 `qvel` 或四元数当成实际传感器输出。采样率、噪声、分辨率和延迟应按实际硬件设定。
