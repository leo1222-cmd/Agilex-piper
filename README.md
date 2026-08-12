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

### 前期准备

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

### 具体步骤

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

## 3、常见文件解决方案

### （1）Jetson 官方 USB-CAN 驱动问题

在 Jetson 上，PiPER 官方 USB-CAN 可能只能通过 lsusb 被识别，但不会自动生成 can1 SocketCAN 接口。典型现象是：lsusb 能看到 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter，但 ip -br link 里只有 Jetson 板载 CAN，没有官方 USB-CAN 对应的接口。这个问题通常是因为 Jetson 定制内核未包含 gs_usb 驱动模块。

#### 本项目使用的系统为：
```bash
cat /etc/nv_tegra_release
uname -r
```

#### 当前环境：
```bash
L4T: R35.3.1
Kernel: 5.10.104-tegra
```

#### 编译 gs_usb.ko

```bash
#1.下载与当前系统版本匹配的 NVIDIA L4T 源码，并解压：
mkdir -p ~/jetson_kernel
cd ~/jetson_kernel

# R35.3.1 对应源码包
wget -O public_sources.tbz2 \
  "https://developer.nvidia.com/downloads/embedded/l4t/r35_release_v3.1/sources/public_sources.tbz2"

tar -xf public_sources.tbz2

cd ~/jetson_kernel/Linux_for_Tegra/source/public
tar -xf kernel_src.tbz2

#2.进入内核源码目录：
cd ~/jetson_kernel/Linux_for_Tegra/source/public/kernel/kernel-5.10

#3.导入当前系统内核配置：
sudo modprobe configs 2>/dev/null || true
zcat /proc/config.gz > .config

#4.设置内核版本后缀：
./scripts/config --set-str LOCALVERSION "-tegra"
./scripts/config --disable LOCALVERSION_AUTO

#5.打开 CAN 相关模块：
./scripts/config --module CAN
./scripts/config --module CAN_RAW
./scripts/config --module CAN_DEV
./scripts/config --module CAN_GS_USB

make olddefconfig

#6.检查配置
grep -E "CONFIG_CAN_GS_USB|CONFIG_CAN_DEV|CONFIG_CAN_RAW|CONFIG_CAN=|CONFIG_LOCALVERSION|CONFIG_LOCALVERSION_AUTO" .config

应看到：
CONFIG_LOCALVERSION="-tegra"
# CONFIG_LOCALVERSION_AUTO is not set
CONFIG_CAN=m
CONFIG_CAN_RAW=m
CONFIG_CAN_DEV=m
CONFIG_CAN_GS_USB=m

#7.编译模块：
make -j$(nproc) modules_prepare
make -j$(nproc) M=drivers/net/can/usb modules

#8.检查模块版本：
modinfo drivers/net/can/usb/gs_usb.ko | grep vermagic
uname -r

两者应一致，例如：
5.10.104-tegra

#9.安装并加载模块：
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb

sudo cp drivers/net/can/usb/gs_usb.ko \
  /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/

sudo depmod -a
sudo modprobe gs_usb

#10.检查：
lsmod | grep gs_usb
modinfo gs_usb | head

#11.设置开机自动加载：
echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf
sudo depmod -a

#12.重新插拔 USB-CAN 后检查：
ip -br link
正常应出现：
can0 DOWN
can1 DOWN
```
## 4、相关参数说明

### 主从控制
```bash
--master-can
主臂 CAN 接口。本项目为 can0。

--slave-can
从臂 CAN 接口。本项目为 can1。

--follow-mode absolute
绝对位置跟随。从臂目标关节角 = 主臂当前关节角 + joint offset。

--speed
PiPER 底层运动速度等级。越大响应越快，初次测试建议 8～15。

--rate
Python 控制循环频率，单位 Hz。

--alpha
平滑系数。越大响应越快，越小越稳。

--max-step-deg
每个控制周期单关节最大变化角度。该参数直接影响跟随速度和安全性。

--cmd-deadband-deg
命令死区，小于该角度变化不发送控制命令，用于减少抖动。

--sync-gripper
开启夹爪同步。

--gripper-source fb
使用主臂夹爪反馈值作为从臂夹爪输入。

--gripper-scale
主臂夹爪反馈到从臂夹爪目标的比例系数。

--gripper-offset
主臂夹爪反馈到从臂夹爪目标的偏置量。

--mirror
六个关节方向映射。1 表示方向一致，-1 表示反向。

--joint-offset-deg
从臂相对主臂的关节偏置。
```
