# SixDForceToolkit

一个面向机器人末端六维力/力矩传感器的载荷标定与重力补偿工具。项目通过多姿态静态采样数据，辨识末端负载的质量、质心位置以及传感器零偏，并根据机器人当前姿态从原始六维力数据中扣除载荷重力和重力力矩，得到更接近真实接触作用的力与力矩。

工具的核心计算不依赖特定六维力传感器型号。只要能够提供统一坐标约定下的末端姿态和六维力数据，即可接入现有机器人或传感器采集程序。

> 当前仓库为实验性工程代码，使用前请先核对坐标系、欧拉角顺序、单位和传感器安装方向。

## 功能

- 多姿态载荷参数辨识
  - 载荷质量
  - 传感器坐标系下的负载质心
  - 三轴力零偏与三轴力矩零偏
  - 世界坐标系与机器人基坐标系的姿态偏差参数
- 六维力重力补偿
  - 扣除载荷重力
  - 扣除质心偏置引起的重力力矩
  - 扣除传感器零偏
- 传感器无关的数据接口
  - 输入末端姿态、三轴力和三轴力矩即可完成标定
  - 可嵌入机器人实时控制、柔顺控制或接触力监测程序
- C++核心实现与Python实验脚本
  - C++版本基于 Eigen 和 Orocos KDL
  - Python脚本包含睿尔曼机械臂数据采集与补偿示例

## 原理概述

六维力传感器安装在机器人末端后，原始测量值通常包含以下分量：

```text
原始六维力 = 外部接触作用 + 负载重力/重力矩 + 传感器零偏 + 测量噪声
```

本项目采用多姿态静态标定方法：

1. 控制机器人到达多个差异明显且保持静止的末端姿态。
2. 在每个姿态下记录传感器姿态、三轴力和三轴力矩。
3. 使用最小二乘法辨识负载质心、质量及传感器零偏。
4. 在线运行时，根据当前姿态计算负载在传感器坐标系中的重力与重力矩。
5. 从原始测量值中扣除重力项和零偏，得到补偿后的接触力/力矩。

标定姿态越丰富、姿态差异越明显，参数矩阵通常越容易获得良好的可观测性。建议使用不少于4组静态姿态，并在每个姿态下对多帧数据取平均后再进行辨识。

## 坐标系与单位

使用前必须统一以下约定：

| 数据 | 约定 |
| --- | --- |
| `Pose` | 六维力传感器坐标系相对于机器人基坐标系的RPY姿态 |
| RPY顺序 | `roll, pitch, yaw` |
| 角度单位 | 弧度 `rad` |
| 力 | 牛顿 `N` |
| 力矩 | 牛顿米 `N·m` |
| 质心坐标 | 米 `m` |
| 质量 | 千克 `kg` |

`GetGravityCompensation()`传入的旋转矩阵必须与标定阶段使用相同的坐标方向和旋转约定。若传感器相对末端法兰存在固定安装角，需要先将该固定变换计入传感器姿态。

## 依赖

### C++

- CMake 3.0或更高版本
- 支持C++11的编译器
- [Eigen3](https://eigen.tuxfamily.org/)
- [Orocos KDL](https://orocos.org/kdl.html)

Ubuntu/Debian可安装：

```bash
sudo apt update
sudo apt install build-essential cmake libeigen3-dev liborocos-kdl-dev
```

### Python实验脚本（可选）

- Python 3
- NumPy
- 睿尔曼机械臂Python SDK：`Robotic_Arm.rm_robot_interface`
- `gravityCompensation copy.py`还使用了Pandas和JSON参数保存功能

Python脚本直接连接真实机械臂，运行前必须修改机械臂IP、确认关节目标点和运动速度，并确保急停、限位与现场安全措施有效。

## 编译与运行C++示例

```bash
git clone https://github.com/anonymouseea/SixDForceToolkit.git
cd SixDForceToolkit

cmake -S . -B build
cmake --build build
./build/SixDForceTool
```

`main.cpp`中的数据是离线演示样例，用于展示姿态转换、载荷参数辨识和结果读取流程。它不负责连接具体型号的六维力传感器。

## C++快速使用

### 1. 添加标定数据

```cpp
#include "SixDForceTool.hpp"

SixDForceTool tool;

// Pose: roll, pitch, yaw，单位rad
// SixDForce: Fx, Fy, Fz, Mx, My, Mz，单位N和N·m
tool.addData(
    Pose(roll_0, pitch_0, yaw_0),
    SixDForce(fx_0, fy_0, fz_0, mx_0, my_0, mz_0)
);

tool.addData(
    Pose(roll_1, pitch_1, yaw_1),
    SixDForce(fx_1, fy_1, fz_1, mx_1, my_1, mz_1)
);

// 继续添加其他静态姿态的数据……
```

### 2. 载荷参数辨识

```cpp
const int sample_count = static_cast<int>(tool.poses.size());

if (tool.LoadParameterIdentification(sample_count) != 0) {
    std::cerr << "载荷参数辨识失败，请检查样本数量。" << std::endl;
    return -1;
}

MassResult load = tool.GetMassAndGravity();
SixDForce zero = tool.GetZeroDriftCalibration();
WorldBaseOffset offset = tool.GetWorldBaseOffset();

std::cout << "mass: " << load.mass << " kg\n";
std::cout << "center of mass: "
          << load.massx << ", "
          << load.massy << ", "
          << load.massz << " m\n";
```

### 3. 在线重力补偿

```cpp
// 当前传感器相对于基坐标系的姿态
KDL::Rotation sensor_rotation =
    KDL::Rotation::RPY(current_roll, current_pitch, current_yaw);

// 传感器原始六维力
KDL::Wrench raw_wrench(
    KDL::Vector(raw_fx, raw_fy, raw_fz),
    KDL::Vector(raw_mx, raw_my, raw_mz)
);

KDL::Wrench compensated =
    tool.GetGravityCompensation(sensor_rotation, raw_wrench);

std::cout << "compensated force: "
          << compensated.force.x() << ", "
          << compensated.force.y() << ", "
          << compensated.force.z() << std::endl;
```

## 主要接口

| 接口 | 说明 |
| --- | --- |
| `addData(pose, force)` | 添加一组静态姿态与六维力标定数据 |
| `LoadParameterIdentification(n)` | 使用前`n`组数据辨识载荷参数和传感器零偏 |
| `GetMassAndGravity()` | 获取质量及质心位置 |
| `GetZeroDriftCalibration()` | 获取三轴力和三轴力矩零偏 |
| `GetWorldBaseOffset()` | 获取世界坐标系与基坐标系姿态偏差参数 |
| `GetGravityCompensation(R, wrench)` | 对当前原始六维力进行重力与零偏补偿 |

## Python实验脚本

`gravityCompensation.py`提供了与睿尔曼机械臂连接、自动采集多个姿态数据并进行标定的实验流程。核心类`GravityCompensation`包含：

- `Update_F()` / `Update_M()`：添加标定力与力矩数据
- `Update_R()` / `Update_f()`：添加姿态矩阵与力数据
- `Solve_A()`：求解质心和力矩常量
- `Solve_B()`：求解重力向量和三轴力零偏
- `Solve_Force()` / `Solve_Torque()`：计算补偿后的接触力和接触力矩
- `compensate()`：对单组测量数据执行力/力矩补偿

脚本内目前包含实验用机械臂IP和关节目标点。建议先将`GravityCompensation`类与硬件采集逻辑拆分，再接入自己的机器人程序。

## 项目结构

```text
SixDForceToolkit/
├── SixDForceTool.hpp            # C++载荷辨识与重力补偿工具类
├── main.cpp                     # C++离线数据示例
├── CMakeLists.txt               # CMake构建配置
├── gravityCompensation.py       # 睿尔曼机械臂Python实验脚本
├── gravityCompensation copy.py  # 参数保存与数据记录实验版本
├── LICENSE                      # MIT许可证
└── README.md
```

## 标定建议

1. 标定过程中保持末端静止，避免加速度和外部接触力进入采样数据。
2. 每个姿态采集多帧数据并取均值，减小测量噪声和机械振动影响。
3. 不要只绕单一轴小角度变化，应选择方向差异明显的多个姿态。
4. 确认力、力矩与姿态的时间戳对应，避免使用不同步的数据。
5. 用未参与标定的新姿态验证补偿结果，观察静止状态下的残余力和残余力矩。
6. 若更换末端工具、负载或传感器安装方式，需要重新标定。

## 当前限制

- 目前没有提供通用传感器驱动，数据采集需要由使用者自行接入。
- 标定假设机器人静止且没有外部接触；动态运动中的惯性力不在当前模型内。
- C++头文件中的`SensorDataFilter()`和`ZeroDriftCalibration()`尚未提供独立实现，当前核心流程使用载荷参数辨识结果完成零偏估计和重力补偿。
- 仓库暂未包含自动化测试和标定数据集，接入实际设备前应使用已知负载进行验证。

## License

本项目基于[MIT License](LICENSE)开源。
