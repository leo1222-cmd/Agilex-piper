# Agilex-piper主从协同 + ArUco 辅助自动夹取系统

本项目面向医疗/实验室场景中的试剂夹取任务，构建了一套基于 **PiPER 双机械臂主从控制、RealSense 深度相机、ArUco 视觉识别、关节角示教复现与夹爪控制** 的半自动化夹取系统。

系统采用“一臂主控、一臂执行”的工作方式：操作者通过主动臂将从动臂引导至目标附近，相机实时显示 ArUco 识别结果；当识别到指定 ArUco 码后，系统可以根据预先保存的示教数据，自动控制从动臂运行到对应夹取位置，完成夹爪闭合，并回到暂停主从时的位置，再恢复主从控制。

当前版本采用 **固定关节角复现 + ArUco 到位校验** 的方案，暂不使用连续视觉伺服控制。

## Software Environment

```bash
Ubuntu 22.04
Python 3.10
piper_sdk
python-can
pyrealsense2
opencv-contrib-python
numpy
```

## Repository Structure

```bash
piper/
├── piper_operator_panel.py              # 一键运行
├── task_targets.json                    # 任务配置
├── ms.py                                # 主从底层控制
├── auto_grasp_by_joint_replay.py        # 自动夹取执行
├── teach_grasp_full_pose.py             # 完整示教
├── taught_full_grasp_poses.json         # 示教数据
│
├── tools_debug/ 
│   ├── go_to_grasp_joint_test.py
│   ├── camera_view.py
│   ├── aruco_detect_realsense.py
│   ├── inspect_endpose.py
│   ├── piper_slave_cartesian_jog_test.py
│
├── archive_old/
│   ├── visual_servo_align.py
│   ├── teach_marker_pose.py
│   ├── aruco_target_locator.py
```

## Core document explanation：

```bash
piper_operator_panel.py          #总控脚本，负责主从控制、相机显示、ID 选择、自动夹取和恢复主从
task_targets.json                #任务配置文件，管理不同 ArUco ID 对应的示教数据和夹爪参数
ms.py                            #底层主从控制脚本，负责对齐和主从跟随
auto_grasp_by_joint_replay.py    #自动夹取脚本，负责关节角复现、ArUco 校验、夹爪闭合和回位
teach_grasp_full_pose.py         #完整示教脚本，用于保存不同 ID 的夹取示教数据
taught_full_grasp_poses.json     #示教数据文件，保存 ArUco 位姿、关节角、末端位姿等信息
go_to_grasp_joint_test.py        #关节复现测试脚本，用于验证示教数据是否可用
```

## Documentation

- **[前期准备与验证](Documents/README1.md)** - 基础工具安装、Python 虚拟环境创建、PiPER SDK 安装、RealSense 相机依赖安装以及功能验证
- **[机械臂主从控制详情](Documents/README2.md)** - 详细介绍piper机械臂主从控制流程

## Quick Start

### 1、启动 CAN

系统初次开启或重启后 CAN 口未启动，需要先执行：
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000 up
sudo ip link set can1 txqueuelen 1000 up
```

检查 CAN 状态：
```bash
ip -br link
ip -details link show can0
ip -details link show can1
```

### 2、一键启动

```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate
python piper_operator_panel.py
```
如果提示按 Enter 确认安全，则按 Enter 继续。

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
