# 前期准备与验证

本项目运行前需要完成基础工具安装、Python 虚拟环境创建、PiPER SDK 安装、RealSense 相机依赖安装以及功能验证。

## 基础工具安装

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

### 其中

```bash
python3-venv              创建 Python 虚拟环境
python3-pip	              安装 Python 功能包
can-utils	                CAN 口调试，例如 candump、cansend
net-tools/ethtool	        网络与 CAN 设备状态查看
git	                      代码管理
build-essential	          编译基础工具
bc、flex、bison、libssl-dev、libelf-dev	后续如需编译内核模块或驱动时使用
v4l-utils	      查看 USB 相机设备，例如 v4l2-ctl --list-devices
```
## Python 虚拟环境创建以及机械臂 PiPER SDK安装

建议所有 Python 脚本都在独立虚拟环境中运行，避免污染系统环境。

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/piper_dual
source ~/venvs/piper_dual/bin/activate

python -m pip install --upgrade pip setuptools wheel
pip install piper_sdk
```

### SDK安装验证

```bash
python - <<'PY'
from piper_sdk import *
print("piper_sdk import OK")
PY
```

## RealSense 相机与视觉功能包安装

本项目使用 Intel RealSense D435i 深度相机，并使用 OpenCV 的 ArUco 模块进行二维码识别。

```bash
source ~/venvs/piper_dual/bin/activate

pip install numpy
pip install pyrealsense2
pip install opencv-contrib-python
```

### 视觉环境验证

验证 OpenCV、ArUco、numpy 和 RealSense 是否可用：

```bash
source ~/venvs/piper_dual/bin/activate

python - <<'PY'
import cv2
import numpy as np

print("opencv:", cv2.__version__)
print("has aruco:", hasattr(cv2, "aruco"))
print("numpy:", np.__version__)

try:
    import pyrealsense2 as rs
    print("pyrealsense2 OK")
except Exception as e:
    print("pyrealsense2 import failed:", e)
PY
```
正常情况下应看到类似：

```bash
opencv: x.x.x
has aruco: True
numpy: x.x.x
pyrealsense2 OK
```

## 相机画面测试

进入项目目录后运行：
```bash
cd ~/piper/tools_debug
source ~/venvs/piper_dual/bin/activate

python camera_view.py
```
如果能正常弹出 RealSense 实时画面，说明相机读取正常。

## ArUco 识别测试

当前项目使用的 ArUco 参数为：
```bash
字典类型：4x4_50
marker_id：0，后续可扩展为 1、2、3...
黑色编码区域边长：0.032 m
```

运行识别测试：
```bash
cd ~/piper/tools_debug
source ~/venvs/piper_dual/bin/activate

python aruco_detect_realsense.py \
  --marker-size-m 0.032 \
  --dict 4x4_50 \
  --marker-id 0
```
如果相机画面中能识别到 ArUco 码，并显示对应 ID 和坐标轴，说明视觉识别功能正常。
