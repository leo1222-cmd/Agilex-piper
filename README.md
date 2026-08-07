# Agilex-piper
松灵piper主从协同+视觉定位

## 1.PiPER Dual-Arm Master-Slave Control on Jetson(在jetson上实现双臂主从控制)

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

### 具体步骤

#### 基础工具安装

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

#### Python 虚拟环境安装以及机械臂 PiPER SDK安装

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/piper_dual

source ~/venvs/piper_dual/bin/activate

python -m pip install --upgrade pip setuptools wheel

pip install piper_sdk
```

#### SDK安装验证

```bash
python - <<'PY'
from piper_sdk import *
print("piper_sdk import OK")
PY
```

#### can口启动与机械臂状态确认

```bash
#1.启动环境
cd ~/piper
source ~/venvs/piper_dual/bin/activate

#2.查看can口
python ms.py find

#3.启动can口（确认主从臂can口型号）
python ms.py start --master-can can0 --slave-can can1

#4.读取双臂状态(确认两边都能读到真实关节角)
python ms.py read --master-can can0 --slave-can can1
```

#### 绝对位置跟随

```bash
#1.摆好主臂位置，让从臂对齐主臂
python ms.py align-slave-to-master \
--master-can can0 \
--slave-can can1 \
--speed 8 \
--rate 20 \
--step-deg 0.8 \
--tol-deg 0.3

#2.启动绝对跟随
python ms.py follow \
--master-can can0 \
--slave-can can1 \
--follow-mode absolute \
--speed 16 \
--rate 30 \
--alpha 0.45 \
--max-step-deg 3.5 \
--cmd-deadband-deg 0.05 \
--sync-gripper \
--gripper-source fb \
--gripper-min 10000 \
--gripper-max 60000 \
--gripper-scale 1.235 \
--gripper-offset -12716 \
--gripper-effort 1000 \
--mirror 1,1,1,1,1,1 \
--joint-offset-deg 0,0,0,0,0,0
```

#### 相对位置跟随
```bash
python ms.py follow \
--master-can can0 \
--slave-can can1 \
--follow-mode relative \
--speed 15 \
--rate 25 \
--scale 1.0 \
--alpha 0.35 \
--max-delta-deg 100 \
--max-step-deg 3.0 \
--master-deadband-deg 0.12 \
--cmd-deadband-deg 0.08 \
--sync-gripper \
--gripper-source fb \
--gripper-min 10000 \
--gripper-max 60000 \
--gripper-scale 1.235 \
--gripper-offset -12716 \
--gripper-effort 1000 \
--gripper-debug \
--mirror 1,1,1,1,1,1
```

## 
