# Agilex-piper
松灵piper主从协同+视觉定位

## 1.PiPER Dual-Arm Master-Slave Control on Jetson

### 项目简介

本项目实现了在 Jetson Orin Nano 平台上的 PiPER 双机械臂主从协同控制。系统采用两台 PiPER 六自由度机械臂，其中一台作为主臂，另一台作为从臂。主臂保持示教/手动操作状态，程序只读取主臂关节角和夹爪反馈；从臂进入 CAN 控制模式，实时跟随主臂关节运动，并同步执行夹爪开合动作。

### 当前项目已实现：
- Jetson Orin Nano 上 USB-CAN 驱动适配
- gs_usb.ko 驱动编译与加载
- SocketCAN 接口 can0 / can1 启动
- PiPER SDK 环境搭建
- 主臂关节角读取
- 从臂 CAN 控制
- 主从绝对位置跟随
- 主臂夹爪反馈同步控制从臂夹爪
- 从臂对齐主臂姿态
- 基础相机点击三维定位测试

### 具体步骤

基础工具安装

```bash
sudo apt update

sudo apt install -y \
  python3-venv \
  python3-pip \
  can-utils \
  net-tools \
  ethtool \
  git \
  build-essential \
  bc \
  flex \
  bison \
  libssl-dev \
  libelf-dev \
  v4l-utils
```


