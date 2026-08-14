# 相机识别定位与 ArUco ID 配置流程

本项目使用 RealSense D435i 深度相机 + OpenCV ArUco 完成目标识别与到位校验。当前系统中，ArUco 的作用主要是：
```bash
1. 在交付脚本中识别当前看到的目标 ID；
2. 根据 ID 查找对应的夹取动作；
3. 自动夹取前进行 ArUco 到位校验；
4. 防止机械臂虽然到达关节角，但目标相对位置偏差过大。
```

具体逻辑：
```bash
ArUco 识别目标 ID
→ 读取该 ID 对应的示教关节角
→ 机械臂复现关节角
→ ArUco 校验相机与目标的相对位置
→ 校验通过后夹取
```

## 前期检查

### 检查视觉环境
```bash
cd ~/piper
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
正常应看到：
```bash
has aruco: True
pyrealsense2 OK
```
如果 has aruco: False，说明没有安装 opencv-contrib-python。

### 相机画面测试
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python tools_debug/camera_view.py
```
如果能看到 RealSense 实时画面，说明相机读取正常。

### ArUco 识别测试

当前项目 ArUco 参数为：
```bash
字典类型：4x4_50
黑色编码区域边长：0.032 m
常用 ID：0、1、2...
```
注意：这里的 0.032 m 是黑色编码区域边长，不是纸张外框边长。

测试 ID=0：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python tools_debug/aruco_detect_realsense.py \
  --marker-size-m 0.032 \
  --dict 4x4_50 \
  --marker-id 0
```

正常效果：
```
1. 相机窗口显示画面；
2. 识别到 ArUco ID；
3. 画面上显示坐标轴；
4. 终端输出 ArUco 相对相机的位置。
```

## 以ID为0时为例，当前 ID=0 的完整流程

当前 ID=0 的逻辑是：
```
识别到 ArUco ID=0
→ task_targets.json 中找到 ID=0
→ pose_name = teach_reagent_grasp_pose_fixed
→ 在 taught_full_grasp_poses.json 中读取该夹取位关节角
→ 机械臂运行到夹取位
→ ArUco 校验
→ 夹爪闭合
→ 执行 post_waypoints，例如 teach_reagent_id0_lift_200
→ 回到按 g 时的位置
```

其中两个 JSON 的关系是：
```
task_targets.json
    负责 ID 与动作的对应关系

taught_full_grasp_poses.json
    负责保存具体动作数据，包括夹取位、拔出位、关节角、ArUco 位姿等
```

## 查看当前有哪些 ID 和动作

### 查看任务配置
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python - <<'PY'
import json
from pathlib import Path

data = json.loads(Path("task_targets.json").read_text())

print("当前 ID 配置：")
for marker_id, cfg in data["targets"].items():
    print(f"\nID={marker_id}")
    print("label:", cfg.get("label"))
    print("pose_name:", cfg.get("pose_name"))
    print("post_waypoints:", cfg.get("post_waypoints"))
    print("enabled:", cfg.get("enabled"))
PY
```

### 查看动作库
```bash
python - <<'PY'
import json
from pathlib import Path

data = json.loads(Path("taught_full_grasp_poses.json").read_text())

print("当前已保存的示教动作：")
for name, rec in data.items():
    print(f"- {name} | pose_type={rec.get('pose_type')} | marker_id={rec.get('marker_id')}")
PY
```

### 修改已有 ID 的对应动作

比如你想让 ID=0 不再执行原来的夹取动作，而是执行另一个动作：
```
旧动作：teach_reagent_grasp_pose_fixed
新动作：teach_reagent_id0_new_fixed
```

修改：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

cp task_targets.json task_targets_backup_change_id0_$(date +%Y%m%d_%H%M%S).json

python - <<'PY'
import json
from pathlib import Path

p = Path("task_targets.json")
data = json.loads(p.read_text())

data["targets"]["0"]["pose_name"] = "teach_reagent_id0_new_fixed"

p.write_text(json.dumps(data, indent=2, ensure_ascii=False))

print("[OK] ID=0 已改为执行 teach_reagent_id0_new_fixed")
PY

python -m json.tool task_targets.json >/dev/null && echo "JSON OK"
```

### 修改已有 ID 的拔出动作

比如你想让 ID=0 夹取后执行新的拔出位：
```
teach_reagent_id0_lift_200
```

修改：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

cp task_targets.json task_targets_backup_change_post_$(date +%Y%m%d_%H%M%S).json

python - <<'PY'
import json
from pathlib import Path

p = Path("task_targets.json")
data = json.loads(p.read_text())

data["targets"]["0"]["lift_mm"] = 0
data["targets"]["0"]["post_waypoints"] = [
    "teach_reagent_id0_lift_200"
]

p.write_text(json.dumps(data, indent=2, ensure_ascii=False))

print("[OK] ID=0 的 post_waypoints 已更新")
PY

python -m json.tool task_targets.json >/dev/null && echo "JSON OK"
```

如果需要多个拔出中间点：
```bash
"post_waypoints": [
  "teach_reagent_id0_lift_80",
  "teach_reagent_id0_lift_200"
]
```

## 新增一个 ArUco ID 的完整流程

假设现在要新增：
```
ArUco ID=1
```

完整流程分为三步：
```
1. 示教 ID=1 的夹取位；
2. 示教 ID=1 的拔出位；
3. 在 task_targets.json 中注册 ID=1。
```

### 1、示教 ID=1 的夹取位

先通过主从控制，把从臂移动到 ID=1 对应试剂的正确夹取位置。要求：
```
1. 相机能稳定识别 ID=1；
2. 夹爪位置适合夹取；
3. 姿态自然；
4. 周围没有干涉；
5. 停止主从后再保存示教。
```

保存 ID=1 的夹取位：
```
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python teach_grasp_full_pose.py \
  --slave-can can1 \
  --pose-name teach_reagent_id1_fixed \
  --marker-size-m 0.032 \
  --dict 4x4_50 \
  --marker-id 1 \
  --samples 80 \
  --out taught_full_grasp_poses.json
```

窗口打开后：
```
按 a：保存示教数据
按 q：退出窗口
```

保存后，taught_full_grasp_poses.json 中会新增：
```
teach_reagent_id1_fixed
```
### 2、示教 ID=1 的拔出位

夹住试剂后，用主从控制带动从臂到拔出后的安全位置，例如向上约 20 cm。保存拔出位：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python teach_joint_pose.py \
  --slave-can can1 \
  --pose-name teach_reagent_id1_lift_200 \
  --out taught_full_grasp_poses.json \
  --note "ID1 夹取后向上拔出约200mm的关节位"
```

保存后，taught_full_grasp_poses.json 中会新增：
```
teach_reagent_id1_lift_200
```

### 3、在 task_targets.json 中注册 ID=1
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

cp task_targets.json task_targets_backup_add_id1_$(date +%Y%m%d_%H%M%S).json

python - <<'PY'
import json
from pathlib import Path

p = Path("task_targets.json")
data = json.loads(p.read_text())

data["targets"]["1"] = {
    "label": "reagent_1",
    "pose_file": "taught_full_grasp_poses.json",
    "pose_name": "teach_reagent_id1_fixed",
    "enabled": True,

    "open_mm": 60,
    "close_mm": 10,
    "gripper_effort": 1000,

    "xy_tol_mm": 7,
    "z_tol_mm": 10,
    "verify_samples": 60,

    "lift_mm": 0,
    "post_waypoints": [
        "teach_reagent_id1_lift_200"
    ]
}

p.write_text(json.dumps(data, indent=2, ensure_ascii=False))

print("[OK] 已新增 ID=1 配置")
PY

python -m json.tool task_targets.json >/dev/null && echo "JSON OK"
```

### 4、检查新增 ID 是否配置正确
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python - <<'PY'
import json
from pathlib import Path

task = json.loads(Path("task_targets.json").read_text())
poses = json.loads(Path("taught_full_grasp_poses.json").read_text())

for marker_id, cfg in task["targets"].items():
    print(f"\nID={marker_id}")
    print("pose_name:", cfg.get("pose_name"))
    print("post_waypoints:", cfg.get("post_waypoints"))

    pose_name = cfg.get("pose_name")
    if pose_name not in poses:
        print("[ERROR] pose_name 不存在:", pose_name)
    else:
        print("[OK] grasp pose 存在")

    for wp in cfg.get("post_waypoints", []):
        if wp not in poses:
            print("[ERROR] post_waypoint 不存在:", wp)
        else:
            print("[OK] post_waypoint 存在:", wp)
PY
```

新增 ID=1 后，应看到：
```
ID=1
pose_name: teach_reagent_id1_fixed
post_waypoints: ['teach_reagent_id1_lift_200']
[OK] grasp pose 存在
[OK] post_waypoint 存在: teach_reagent_id1_lift_200
```

## 测试与检验

### 先单独测试 ID=1 的自动流程
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python auto_grasp_by_joint_replay.py \
  --slave-can can1 \
  --pose-file taught_full_grasp_poses.json \
  --pose-name teach_reagent_id1_fixed \
  --speed 8 \
  --step-deg 1.0 \
  --tol-deg 0.3 \
  --rate 20 \
  --xy-tol-mm 7 \
  --z-tol-mm 10 \
  --verify-samples 60 \
  --open-mm 60 \
  --close-mm 10 \
  --gripper-effort 1000 \
  --post-waypoints teach_reagent_id1_lift_200
```
测试通过后，再用最终脚本。

### 最终脚本中选择 ID
运行：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python piper_operator_panel.py
```

操作方式：
```
1. 主从控制启动；
2. 相机窗口显示 ArUco；
3. 如果看到 ID=0，按 0；
4. 如果看到 ID=1，按 1；
5. 按 g 执行当前选中 ID 对应的自动夹取和拔出流程；
6. 执行完成后，系统回到按 g 时的位置，并恢复主从。
```

## 注

在添加id和动作配置时，我们使用了cp命令进行备份以防止出错，最终调试完成后，可以对备份文件进行删除，具体流程如下：

### 1、先确认当前两个正式 JSON 可用
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python -m json.tool task_targets.json >/dev/null && echo "task_targets.json OK"
python -m json.tool taught_full_grasp_poses.json >/dev/null && echo "taught_full_grasp_poses.json OK"
```
如果都显示 OK，再继续删备份。

### 2、先查看有哪些 JSON 备份
```bash
cd ~/piper

find . -maxdepth 1 -type f -name "*backup*.json" -print
```

如果输出的是类似这些：
```
./task_targets_backup_xxx.json
./taught_full_grasp_poses_backup_xxx.json
```
就说明可以删。

### 3、删除所有 JSON 备份文件
```bash
cd ~/piper

find . -maxdepth 1 -type f -name "*backup*.json" -delete
```

### 4、最后检查
```bash
cd ~/piper

ls -lh *.json
```

最终应该只看到：
```bash
task_targets.json
taught_full_grasp_poses.json
```
