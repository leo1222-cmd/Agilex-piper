# 修改各ID对应参数配置（JSON文件）具体步骤——ID0作为示例
改某个 ID 的夹取位置或放回位置，不用改总控代码，只需要重新示教对应 ID 的 pose，覆盖 JSON 里的记录就行。主要需示教：
- 夹取位置：`pose_name`
- 放回位置：`return_pose_name`

## 1、修改某个 ID 对应夹取位置
比如修改ID0：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python teach_grasp_full_pose.py \
  --id 0 \
  --slave-can can1
```
具体流程：
```
主从把从臂移动到新的 ID0 夹取位
↓
确认夹爪与试剂位置正确
↓
保存
↓
覆盖 teach_reagent_grasp_pose_id0
↓
task_targets.json 仍保持：
pose_name = teach_reagent_grasp_pose_id0
```
按流程走完，就会自动读取新的 ID0 夹取位置。

## 2、修改某个 ID 对应的放回位置
比如修改ID0：
```bash
cd ~/piper
source ~/venvs/piper_dual/bin/activate

python teach_return_full_pose.py \
  --id 0 \
  --slave-can can1
```

具体流程：
```
用主从移动到新的放回预位
↓
试剂对准对应孔
↓
下端距离孔口约 10 mm
↓
按 Enter
↓
检查姿态
↓
输入 SAVE
```
因为脚本默认 pose 名是：
```
teach_reagent_return_pose_id0
```
所以重新 SAVE 后会直接覆盖 ID0 原来的放回位置，不需要再改总控。
