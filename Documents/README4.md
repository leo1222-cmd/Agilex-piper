# 修改各ID对应参数配置（JSON文件）具体步骤——ID0作为示例

## 1、保存 ID0 的固定抓取位

### 姿态摆好以后不要再碰机械臂，执行：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python teach_joint_endpose.py \
  --slave-can can1 \
  --pose-name teach_reagent_id0_grasp \
  --out taught_full_grasp_poses.json
```
从臂为can1，若从臂是 can0，则改为 --slave-can can0
数值没有明显异常，然后输入y进行保存

### 保存以后马上执行下面这个检查：
```bash
python - <<'PY'
import json

with open("taught_full_grasp_poses.json", "r") as f:
    d = json.load(f)

name = "teach_reagent_id0_grasp"

print("\nexists =", name in d)

if name in d:
    r = d[name]

    print("\nslave_joint_raw:")
    print(r.get("slave_joint_raw"))

    print("\nslave_endpose_raw:")
    print(r.get("slave_endpose_raw"))

    if "slave_joint_raw" in r:
        print("\nJoint (deg):")
        print([round(v / 1000.0, 3) for v in r["slave_joint_raw"]])

    if "slave_endpose_raw" in r:
        p = r["slave_endpose_raw"]

        print("\nEndPose:")
        print(
            "XYZ mm =",
            [round(v / 1000.0, 3) for v in p[:3]]
        )
        print(
            "RPY deg =",
            [round(v / 1000.0, 3) for v in p[3:]]
        )
PY
```
正确结果应该同时存在：slave_joint_raw（MOVE_J 先把机械臂送到抓取附近）和slave_endpose_raw（到位以后检查实际 XYZ/RPY
→ 必要时 MOVE_P 自动微调）
