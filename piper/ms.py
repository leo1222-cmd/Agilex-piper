#!/usr/bin/env python3
import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import List

from piper_sdk import *


@dataclass
class Joint:
    j: List[int]

    def copy(self):
        return Joint(list(self.j))

    def readable(self):
        return " | ".join([f"J{i+1}={v / 1000:.2f}deg" for i, v in enumerate(self.j)])


def run(cmd, check=False):
    print("[CMD]", " ".join(cmd))
    ret = subprocess.run(cmd)
    if check and ret.returncode != 0:
        raise RuntimeError(f"命令失败: {' '.join(cmd)}")
    return ret.returncode == 0


def make_piper(can_name):
    try:
        return C_PiperInterface_V2(can_name=can_name, judge_flag=False)
    except Exception:
        return C_PiperInterface(can_name=can_name, judge_flag=False)


def connect_piper(can_name, wait=1.0):
    p = make_piper(can_name)
    p.ConnectPort()
    time.sleep(wait)
    return p


def get_joint(piper) -> Joint:
    msg = piper.GetArmJointMsgs()
    js = msg.joint_state
    return Joint([
        int(js.joint_1),
        int(js.joint_2),
        int(js.joint_3),
        int(js.joint_4),
        int(js.joint_5),
        int(js.joint_6),
    ])


def print_status(name, piper):
    print(f"\n========== {name} ==========")
    print("\n[ArmStatus]")
    try:
        print(piper.GetArmStatus())
    except Exception as e:
        print("[ERROR]", e)

    print("\n[Joint]")
    try:
        print(get_joint(piper).readable())
    except Exception as e:
        print("[ERROR]", e)


def set_standby(piper):
    if hasattr(piper, "ModeCtrl"):
        piper.ModeCtrl(0x00, 0x00, 0, 0x00)
    else:
        piper.MotionCtrl_2(0x00, 0x00, 0, 0x00)
    time.sleep(0.5)


def set_can_j(piper, speed=15):
    speed = max(1, min(int(speed), 20))
    if hasattr(piper, "ModeCtrl"):
        piper.ModeCtrl(0x01, 0x01, speed, 0x00)
    else:
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    time.sleep(0.5)


def enable_arm(piper):
    try:
        piper.EnableArm(7)
        time.sleep(0.3)
        return
    except Exception:
        pass

    for i in range(1, 7):
        try:
            piper.EnableArm(i)
            time.sleep(0.08)
        except Exception:
            pass


def send_joint(piper, joint: Joint):
    piper.JointCtrl(*joint.j)


def extract_gripper_angle(obj):
    if obj is None:
        return None

    if hasattr(obj, "grippers_angle"):
        try:
            return int(getattr(obj, "grippers_angle"))
        except Exception:
            return None

    for name in [
        "gripper_state",
        "gripper_ctrl",
        "arm_gripper_ctrl",
        "arm_gripper_teaching_param_feedback",
    ]:
        sub = getattr(obj, name, None)
        val = extract_gripper_angle(sub)
        if val is not None:
            return val

    return None


def read_gripper_angle(piper, method_name):
    if not hasattr(piper, method_name):
        return None

    try:
        msg = getattr(piper, method_name)()
        return extract_gripper_angle(msg)
    except Exception:
        return None


def get_master_gripper_angle(piper, source="fb"):
    source = source.lower()

    if source == "ctrl":
        return read_gripper_angle(piper, "GetArmGripperCtrl")

    if source == "fb":
        return read_gripper_angle(piper, "GetArmGripperMsgs")

    fb = read_gripper_angle(piper, "GetArmGripperMsgs")
    if fb is not None:
        return fb

    return read_gripper_angle(piper, "GetArmGripperCtrl")


def send_gripper(piper, angle_raw, effort=1000):
    angle_raw = int(angle_raw)
    effort = max(0, min(int(effort), 5000))
    piper.GripperCtrl(angle_raw, effort, 0x01, 0x00)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_mirror(s):
    vals = [int(x.strip()) for x in s.split(",")]
    if len(vals) != 6:
        raise ValueError("--mirror 必须是 6 个数，例如 1,1,1,1,1,1")
    for v in vals:
        if v not in [-1, 1]:
            raise ValueError("--mirror 只能是 1 或 -1")
    return vals


def parse_joint_deg(s):
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 6:
        raise ValueError("关节角必须是 6 个数，例如 0,5,-5,0,0,0")
    return Joint([int(v * 1000) for v in vals])


def parse_joint_offset_deg(s):
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 6:
        raise ValueError("--joint-offset-deg 必须是 6 个数，例如 0,0,0,0,0,0")
    return [int(v * 1000) for v in vals]


def step_joint_towards(current: Joint, target: Joint, step_raw: int) -> Joint:
    out = []
    for c, t in zip(current.j, target.j):
        diff = t - c
        if abs(diff) <= step_raw:
            out.append(t)
        else:
            out.append(c + step_raw if diff > 0 else c - step_raw)
    return Joint(out)


def joint_error_max_deg(current: Joint, target: Joint) -> float:
    return max(abs(c - t) for c, t in zip(current.j, target.j)) / 1000.0


def list_can_drivers():
    result = []
    if not os.path.exists("/sys/class/net"):
        return result

    for name in sorted(os.listdir("/sys/class/net")):
        if not name.startswith("can"):
            continue

        driver_path = f"/sys/class/net/{name}/device/driver"
        if os.path.exists(driver_path):
            driver = os.path.realpath(driver_path)
        else:
            driver = "no-driver"

        result.append((name, driver))

    return result


def setup_can(can_if):
    print(f"\n========== 启动 {can_if} ==========")

    run(["sudo", "modprobe", "gs_usb"], check=False)
    run(["sudo", "ip", "link", "set", can_if, "down"], check=False)

    ok = run(["sudo", "ip", "link", "set", can_if, "type", "can", "bitrate", "1000000"], check=False)
    if not ok:
        print(f"\n[FAIL] {can_if} 设置 bitrate 失败。")
        print("处理方法：拔插这一路 USB-CAN，或者换 USB 口后重新执行 find/start。")
        return False

    run(["sudo", "ip", "link", "set", can_if, "txqueuelen", "1000"], check=False)

    ok = run(["sudo", "ip", "link", "set", can_if, "up"], check=False)
    if not ok:
        print(f"\n[FAIL] {can_if} 启动失败。")
        return False

    print(f"\n[OK] {can_if} 已启动")
    subprocess.run(["ip", "-details", "-statistics", "link", "show", can_if])
    return True


def cmd_find(args):
    run(["sudo", "modprobe", "gs_usb"], check=False)

    print("\n========== ip -br link ==========")
    subprocess.run(["ip", "-br", "link"])

    print("\n========== CAN 口驱动 ==========")
    ports = list_can_drivers()

    if not ports:
        print("[FAIL] 当前没有 can 口。请检查两个 USB-CAN 是否插入。")
        return

    for name, driver in ports:
        print(f"{name} -> {driver}")

    print("\n判断：能看到 can0/can1，且驱动正常，就可以继续 start。")


def cmd_start_one(args):
    setup_can(args.can_if)


def cmd_start(args):
    if args.master_can == args.slave_can:
        print("[ERROR] master-can 和 slave-can 不能相同。")
        return

    ok_master = setup_can(args.master_can)
    ok_slave = setup_can(args.slave_can)

    if ok_master and ok_slave:
        print("\n[OK] 主臂和从臂 CAN 都启动完成。")
    else:
        print("\n[FAIL] 至少有一路 CAN 启动失败。")


def cmd_dump(args):
    print(f"[INFO] candump {args.can_if}，最多显示 30 帧。")
    print("[INFO] 只要能刷出 CAN 数据，就说明底层 CAN 通信正常。")

    try:
        proc = subprocess.Popen(
            ["candump", args.can_if],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        count = 0
        start = time.time()

        while True:
            line = proc.stdout.readline()

            if line:
                print(line, end="")
                count += 1

            if count >= 30:
                print("\n[OK] 已收到 30 帧 CAN 数据，自动停止。")
                break

            if time.time() - start > args.seconds:
                print(f"\n[INFO] 已等待 {args.seconds} 秒，自动停止。")
                break

        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    except FileNotFoundError:
        print("[ERROR] 没有 candump，请先安装：sudo apt install -y can-utils")


def cmd_read_one(args):
    arm = connect_piper(args.can_if)
    print_status(f"机械臂 {args.can_if}", arm)
    print("\n判断：Hz 接近 200、Joint 有真实值，说明通信正常。")


def cmd_read(args):
    master = connect_piper(args.master_can)
    slave = connect_piper(args.slave_can)

    print_status("主臂", master)
    print_status("从臂", slave)

    print("\n判断标准：")
    print("1. 主臂 Hz 应接近 200")
    print("2. 从臂 Hz 应接近 200")
    print("3. Error Code 应为 0")
    print("4. Joint 不应全是 0")


def cmd_watch_master(args):
    master = connect_piper(args.master_can)
    print_status("主臂初始状态", master)

    print("\n现在手动移动主臂。")
    print("如果 J1~J6 数值变化，说明主臂读取正常。")
    print("Ctrl+C 退出。\n")

    try:
        while True:
            print(get_joint(master).readable())
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[INFO] 退出。")


def cmd_watch_both(args):
    arm_a = connect_piper(args.can_a)
    arm_b = connect_piper(args.can_b)

    print_status(f"A {args.can_a}", arm_a)
    print_status(f"B {args.can_b}", arm_b)

    print("\n现在只移动主动臂。")
    print("看 A 或 B 哪一行关节值变化，用来确认 can 口对应关系。")
    print("Ctrl+C 退出。\n")

    try:
        while True:
            print(f"A {args.can_a}: {get_joint(arm_a).readable()}")
            print(f"B {args.can_b}: {get_joint(arm_b).readable()}")
            print("-" * 90)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[INFO] 退出。")


def cmd_watch_gripper(args):
    master = connect_piper(args.master_can)
    print_status("主动臂", master)

    print("\n[INFO] 现在操作主动臂示教器/夹爪开合。")
    print("[INFO] 观察 CTRL 或 FB 哪个数值会变化。")
    print("[INFO] 当前你这里应该是 FB 会变化。")
    print("[INFO] Ctrl+C 退出。\n")

    try:
        while True:
            ctrl = read_gripper_angle(master, "GetArmGripperCtrl")
            fb = read_gripper_angle(master, "GetArmGripperMsgs")

            ctrl_mm = None if ctrl is None else ctrl / 1000.0
            fb_mm = None if fb is None else fb / 1000.0

            print(f"CTRL={ctrl} ({ctrl_mm} mm) | FB={fb} ({fb_mm} mm)")
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[INFO] 退出主动臂夹爪观察。")


def cmd_standby(args):
    master = connect_piper(args.master_can)
    slave = connect_piper(args.slave_can)

    print_status("主臂 STANDBY 前", master)
    print_status("从臂 STANDBY 前", slave)

    print("\n[INFO] 两台机械臂发送 STANDBY")
    set_standby(master)
    set_standby(slave)
    time.sleep(1.0)

    print_status("主臂 STANDBY 后", master)
    print_status("从臂 STANDBY 后", slave)


def cmd_slave_can_j(args):
    slave = connect_piper(args.slave_can)

    print_status("从臂切换前", slave)

    print("\n[INFO] 从臂切 CAN_CTRL + MOVE_J")
    set_can_j(slave, args.speed)
    enable_arm(slave)
    time.sleep(1.0)

    print_status("从臂切换后", slave)


def cmd_test_slave(args):
    slave = connect_piper(args.slave_can)

    print_status("从臂测试前", slave)

    print("\n[INFO] 从臂切 CAN_CTRL + MOVE_J")
    set_can_j(slave, args.speed)
    enable_arm(slave)
    time.sleep(1.0)

    j0 = get_joint(slave)
    print("[INFO] 从臂当前关节：", j0.readable())

    input(f"确认从臂周围安全，按 Enter 测试 J6 +{args.test_deg} deg...")

    j1 = j0.copy()
    j1.j[5] += int(args.test_deg * 1000)

    print("[MOVE] 从臂 J6 小幅运动")
    send_joint(slave, j1)
    time.sleep(1.0)

    print("[MOVE] 从臂 J6 回原位")
    send_joint(slave, j0)
    time.sleep(1.0)

    print_status("从臂测试后", slave)


def cmd_test_gripper(args):
    slave = connect_piper(args.slave_can)

    print_status("从臂夹爪测试前", slave)

    print("\n[INFO] 从臂切 CAN_CTRL + MOVE_J，并使能机械臂")
    set_can_j(slave, args.speed)
    enable_arm(slave)
    time.sleep(1.0)

    open_raw = int(args.open_mm * 1000)
    close_raw = int(args.close_mm * 1000)

    print(f"\n[INFO] open_raw={open_raw}, close_raw={close_raw}, effort={args.effort}")
    input("确认从臂夹爪周围安全，按 Enter 开始测试夹爪开合...")

    print(f"\n[GRIPPER] 张开到 {args.open_mm} mm")
    for _ in range(5):
        send_gripper(slave, open_raw, args.effort)
        time.sleep(0.15)

    time.sleep(1.0)

    print(f"\n[GRIPPER] 闭合到 {args.close_mm} mm")
    for _ in range(5):
        send_gripper(slave, close_raw, args.effort)
        time.sleep(0.15)

    time.sleep(1.0)

    print(f"\n[GRIPPER] 再张开到 {args.open_mm} mm")
    for _ in range(5):
        send_gripper(slave, open_raw, args.effort)
        time.sleep(0.15)

    time.sleep(1.0)

    print_status("从臂夹爪测试后", slave)


def cmd_align_slave_to_master(args):
    master = connect_piper(args.master_can)
    slave = connect_piper(args.slave_can)

    print_status("主臂当前状态", master)
    print_status("从臂对齐前", slave)

    print("\n========== 从臂对齐主臂当前姿态 ==========")
    print("主臂保持手动 / 只读，程序不控制主臂")
    print("从臂将移动到主臂当前关节角")
    print(f"速度 speed={args.speed}")
    print(f"每步最大 step={args.step_deg} deg")
    print(f"控制频率 rate={args.rate} Hz")
    print("=======================================")
    print("\n[重要安全确认]")
    print("1. 请先把主臂手动摆到你想要的初始姿态")
    print("2. 从臂运动路径中不能有人手、线缆、相机或障碍物")
    print("3. 手放在实体急停附近")
    input("确认安全后按 Enter，开始让从臂对齐主臂当前姿态...")

    target = get_joint(master)
    print("\n[INFO] 对齐目标姿态:", target.readable())

    print("\n[INFO] 从臂切 CAN_CTRL + MOVE_J")
    set_can_j(slave, args.speed)
    enable_arm(slave)
    time.sleep(1.0)

    step_raw = int(args.step_deg * 1000)
    dt = 1.0 / max(args.rate, 1.0)
    tol_deg = args.tol_deg

    slave_cmd = get_joint(slave)

    last_print = time.time()
    start_t = time.time()

    while True:
        if args.live_master:
            target = get_joint(master)

        slave_cmd = step_joint_towards(slave_cmd, target, step_raw)
        send_joint(slave, slave_cmd)

        time.sleep(dt)

        slave_now = get_joint(slave)
        slave_err = joint_error_max_deg(slave_now, target)

        now = time.time()
        if now - last_print > 1.0:
            print("\n[ALIGN]")
            print("主臂目标:", target.readable())
            print("从臂当前:", slave_now.readable())
            print(f"从臂最大误差={slave_err:.3f} deg")
            last_print = now

        if slave_err <= tol_deg:
            break

        if now - start_t > args.timeout:
            print("\n[WARN] 从臂对齐超时，停止继续逼近。")
            break

    print("\n[INFO] 发送最终目标姿态")
    for _ in range(10):
        send_joint(slave, target)
        time.sleep(0.05)

    time.sleep(0.8)

    print_status("主臂最终状态", master)
    print_status("从臂对齐后", slave)

    if args.standby_slave:
        print("\n[INFO] 从臂切回 STANDBY。")
        set_standby(slave)

    print("\n[OK] 从臂已对齐主臂当前姿态。")
    print("下一步可以运行 absolute 主从跟随。")


def cmd_go_home_both(args):
    master = connect_piper(args.master_can)
    slave = connect_piper(args.slave_can)

    target = parse_joint_deg(args.home_deg)

    print_status("主臂回 home 前", master)
    print_status("从臂回 home 前", slave)

    print("\n========== 双臂回同一初始位 ==========")
    print("目标 home:", target.readable())
    print(f"速度 speed={args.speed}")
    print(f"每步最大 step={args.step_deg} deg")
    print(f"控制频率 rate={args.rate} Hz")
    print("====================================")
    print("\n[重要安全确认]")
    print("1. 两台机械臂路径中不能有人手、线缆、相机或其他障碍物")
    print("2. home-deg 必须是你确认过的安全姿态")
    print("3. 手放在实体急停附近")
    input("确认安全后按 Enter，开始让两台机械臂同时回到同一初始位...")

    print("\n[INFO] 两台机械臂切 CAN_CTRL + MOVE_J")
    set_can_j(master, args.speed)
    set_can_j(slave, args.speed)
    enable_arm(master)
    enable_arm(slave)
    time.sleep(1.0)

    step_raw = int(args.step_deg * 1000)
    dt = 1.0 / max(args.rate, 1.0)
    tol_deg = args.tol_deg

    master_cmd = get_joint(master)
    slave_cmd = get_joint(slave)

    last_print = time.time()
    start_t = time.time()

    while True:
        master_cmd = step_joint_towards(master_cmd, target, step_raw)
        slave_cmd = step_joint_towards(slave_cmd, target, step_raw)

        send_joint(master, master_cmd)
        send_joint(slave, slave_cmd)

        time.sleep(dt)

        master_now = get_joint(master)
        slave_now = get_joint(slave)

        master_err = joint_error_max_deg(master_now, target)
        slave_err = joint_error_max_deg(slave_now, target)

        now = time.time()
        if now - last_print > 1.0:
            print("\n[GO_HOME]")
            print("主臂当前:", master_now.readable())
            print("从臂当前:", slave_now.readable())
            print(f"主臂最大误差={master_err:.3f} deg, 从臂最大误差={slave_err:.3f} deg")
            last_print = now

        if master_err <= tol_deg and slave_err <= tol_deg:
            break

        if now - start_t > args.timeout:
            print("\n[WARN] 回 home 超时，停止继续逼近。")
            break

    print("\n[INFO] 发送最终 home 姿态")
    for _ in range(10):
        send_joint(master, target)
        send_joint(slave, target)
        time.sleep(0.05)

    time.sleep(0.8)

    print_status("主臂回 home 后", master)
    print_status("从臂回 home 后", slave)

    if not args.keep_master_can:
        print("\n[INFO] 主臂切回 STANDBY，方便后续人工示教 / 遥控。")
        set_standby(master)

    if args.standby_slave:
        print("[INFO] 从臂也切回 STANDBY。")
        set_standby(slave)

    print("\n[OK] 双臂初始位动作完成。")
    print("下一步可以运行 absolute 主从跟随。")


def compute_target_relative(
    master_home,
    slave_home,
    master_now,
    last_cmd,
    mirror,
    scale,
    max_delta_raw,
    max_step_raw,
    alpha,
    master_deadband_raw=120,
    cmd_deadband_raw=80,
):
    out = []

    for i in range(6):
        master_delta = master_now.j[i] - master_home.j[i]

        if abs(master_delta) < master_deadband_raw:
            master_delta = 0

        slave_delta = int(scale * mirror[i] * master_delta)
        slave_delta = clamp(slave_delta, -max_delta_raw, max_delta_raw)

        raw_target = slave_home.j[i] + slave_delta
        smooth_target = int(alpha * raw_target + (1.0 - alpha) * last_cmd.j[i])

        step = smooth_target - last_cmd.j[i]

        if abs(step) < cmd_deadband_raw:
            step = 0

        step = clamp(step, -max_step_raw, max_step_raw)

        out.append(last_cmd.j[i] + step)

    return Joint(out)


def compute_target_absolute(
    master_now,
    last_cmd,
    mirror,
    joint_offset_raw,
    max_step_raw,
    alpha,
    cmd_deadband_raw=80,
):
    out = []

    for i in range(6):
        raw_target = mirror[i] * master_now.j[i] + joint_offset_raw[i]
        smooth_target = int(alpha * raw_target + (1.0 - alpha) * last_cmd.j[i])

        step = smooth_target - last_cmd.j[i]

        if abs(step) < cmd_deadband_raw:
            step = 0

        step = clamp(step, -max_step_raw, max_step_raw)

        out.append(last_cmd.j[i] + step)

    return Joint(out)


def get_key(timeout=0.0):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return sys.stdin.read(1)
    return None


def cmd_follow(args):
    mirror = parse_mirror(args.mirror)
    joint_offset_raw = parse_joint_offset_deg(args.joint_offset_deg)

    dt = 1.0 / max(args.rate, 1.0)
    max_delta_raw = int(args.max_delta_deg * 1000)
    max_step_raw = int(args.max_step_deg * 1000)

    print("========== PiPER 主从跟随 ==========")
    print("主臂：人控制，Python 只读取关节角和夹爪 FB")
    print("从臂：Python 切 CAN_CTRL + MOVE_J，并跟随主臂")
    print(f"跟随模式：{args.follow_mode}")
    print("")
    print("SPACE：暂停 / 继续")
    print("H：重新记录 HOME")
    print("P：打印状态")
    print("ESC：退出")
    print("===================================")

    master = connect_piper(args.master_can)
    slave = connect_piper(args.slave_can)

    print_status("主臂初始状态", master)
    print_status("从臂初始状态", slave)

    print("\n[INFO] 从臂切 CAN_CTRL + MOVE_J")
    set_can_j(slave, args.speed)
    enable_arm(slave)
    time.sleep(1.0)

    print_status("从臂切换后", slave)

    if args.sync_gripper:
        print("\n[GRIPPER] 已开启夹爪同步")
        print(f"[GRIPPER] source={args.gripper_source}")
        print("[GRIPPER] gripper_target = offset + scale * master_raw")
        print(f"[GRIPPER] min={args.gripper_min}, max={args.gripper_max}")
        print(f"[GRIPPER] scale={args.gripper_scale}, offset={args.gripper_offset}")
        print(f"[GRIPPER] effort={args.gripper_effort}")

    print("\n[重要确认]")
    print("1. 主臂可以被人手动控制")
    print("2. 主臂是 can0，从臂是 can1；如果反了，退出后交换参数")
    print("3. 从臂周围安全")
    print("4. 手放在实体急停附近")
    print("5. absolute 模式下，建议先用 align-slave-to-master 让从臂对齐主臂")
    input("确认后按 Enter，记录当前姿态并开始跟随...")

    master_home = get_joint(master)
    slave_home = get_joint(slave)
    last_cmd = slave_home.copy()

    last_gripper_send_t = 0.0
    last_gripper_debug_t = 0.0
    last_gripper_target = None

    print("\n[HOME] 主臂:", master_home.readable())
    print("[HOME] 从臂:", slave_home.readable())

    paused = False

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    last_print = time.time()

    try:
        while True:
            key = get_key(0.0)

            if key is not None and ord(key) == 27:
                print("\n[INFO] 退出主从跟随。")
                break

            elif key == " ":
                paused = not paused
                print(f"\n[INFO] {'暂停' if paused else '继续'}跟随。")

            elif key is not None and key.lower() == "h":
                master_home = get_joint(master)
                slave_home = get_joint(slave)
                last_cmd = slave_home.copy()
                last_gripper_target = None

                print("\n[HOME] 已重新记录。")
                print("主臂:", master_home.readable())
                print("从臂:", slave_home.readable())

            elif key is not None and key.lower() == "p":
                print_status("主臂", master)
                print_status("从臂", slave)
                print("paused =", paused)

            if not paused:
                master_now = get_joint(master)

                if args.follow_mode == "absolute":
                    target = compute_target_absolute(
                        master_now=master_now,
                        last_cmd=last_cmd,
                        mirror=mirror,
                        joint_offset_raw=joint_offset_raw,
                        max_step_raw=max_step_raw,
                        alpha=args.alpha,
                        cmd_deadband_raw=int(args.cmd_deadband_deg * 1000),
                    )
                else:
                    target = compute_target_relative(
                        master_home=master_home,
                        slave_home=slave_home,
                        master_now=master_now,
                        last_cmd=last_cmd,
                        mirror=mirror,
                        scale=args.scale,
                        max_delta_raw=max_delta_raw,
                        max_step_raw=max_step_raw,
                        alpha=args.alpha,
                        master_deadband_raw=int(args.master_deadband_deg * 1000),
                        cmd_deadband_raw=int(args.cmd_deadband_deg * 1000),
                    )

                if target.j != last_cmd.j:
                    send_joint(slave, target)
                    last_cmd = target

                if args.sync_gripper:
                    master_g = get_master_gripper_angle(master, args.gripper_source)

                    if master_g is not None:
                        gripper_target = int(args.gripper_offset + args.gripper_scale * master_g)

                        if args.gripper_invert:
                            gripper_target = args.gripper_min + args.gripper_max - gripper_target

                        gripper_target = clamp(gripper_target, args.gripper_min, args.gripper_max)

                        now_g = time.time()

                        if now_g - last_gripper_send_t >= 0.10:
                            send_gripper(slave, gripper_target, args.gripper_effort)
                            last_gripper_send_t = now_g
                            last_gripper_target = gripper_target

                        if args.gripper_debug and now_g - last_gripper_debug_t >= 0.5:
                            print(
                                f"[GRIPPER] master_raw={master_g}, "
                                f"master_mm={master_g/1000.0:.2f}, "
                                f"target_raw={gripper_target}, "
                                f"target_mm={gripper_target/1000.0:.2f}"
                            )
                            last_gripper_debug_t = now_g

                    else:
                        now_g = time.time()
                        if args.gripper_debug and now_g - last_gripper_debug_t >= 1.0:
                            print("[GRIPPER] 未读到主动臂夹爪信号")
                            last_gripper_debug_t = now_g

            now = time.time()

            if now - last_print > 2.0:
                print("\n[RUNNING]")
                print("主臂当前:", get_joint(master).readable())
                print("从臂当前:", get_joint(slave).readable())
                print("从臂目标:", last_cmd.readable())
                if args.sync_gripper:
                    print("从臂夹爪目标:", last_gripper_target)
                print(f"mode={args.follow_mode}, scale={args.scale}, mirror={mirror}, paused={paused}")
                last_print = now

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C 退出。")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("[INFO] 程序结束。")


def main():
    parser = argparse.ArgumentParser(description="PiPER 双臂主从遥控最终版")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("find", help="查看 CAN 口")
    p.set_defaults(func=cmd_find)

    p = sub.add_parser("start-one", help="启动单个 CAN 口")
    p.add_argument("--can-if", required=True)
    p.set_defaults(func=cmd_start_one)

    p = sub.add_parser("start", help="启动主臂和从臂 CAN")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.set_defaults(func=cmd_start)

    p = sub.add_parser("dump", help="candump 检查 CAN 数据")
    p.add_argument("--can-if", required=True)
    p.add_argument("--seconds", type=int, default=5)
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("read-one", help="读取单个机械臂状态")
    p.add_argument("--can-if", required=True)
    p.set_defaults(func=cmd_read_one)

    p = sub.add_parser("read", help="读取双臂状态")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("watch-master", help="观察主臂关节角是否变化")
    p.add_argument("--master-can", required=True)
    p.set_defaults(func=cmd_watch_master)

    p = sub.add_parser("watch-both", help="同时观察两路 CAN，确认哪路是主臂")
    p.add_argument("--can-a", default="can0")
    p.add_argument("--can-b", default="can1")
    p.set_defaults(func=cmd_watch_both)

    p = sub.add_parser("watch-gripper", help="观察主动臂示教器 / 夹爪开合信号")
    p.add_argument("--master-can", required=True)
    p.set_defaults(func=cmd_watch_gripper)

    p = sub.add_parser("standby", help="两臂发送 STANDBY")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.set_defaults(func=cmd_standby)

    p = sub.add_parser("slave-can-j", help="从臂切 CAN_CTRL + MOVE_J")
    p.add_argument("--slave-can", required=True)
    p.add_argument("--speed", type=int, default=15)
    p.set_defaults(func=cmd_slave_can_j)

    p = sub.add_parser("test-gripper", help="测试从臂夹爪开合")
    p.add_argument("--slave-can", required=True)
    p.add_argument("--speed", type=int, default=5)
    p.add_argument("--open-mm", type=float, default=60.0)
    p.add_argument("--close-mm", type=float, default=10.0)
    p.add_argument("--effort", type=int, default=1000)
    p.set_defaults(func=cmd_test_gripper)

    p = sub.add_parser("test-slave", help="测试从臂 J6 小幅运动")
    p.add_argument("--slave-can", required=True)
    p.add_argument("--speed", type=int, default=5)
    p.add_argument("--test-deg", type=float, default=0.3)
    p.set_defaults(func=cmd_test_slave)

    p = sub.add_parser("align-slave-to-master", help="只控制从臂移动到主臂当前关节角，主臂只读不控制")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.add_argument("--speed", type=int, default=8)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--step-deg", type=float, default=0.8)
    p.add_argument("--tol-deg", type=float, default=0.3)
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--live-master", action="store_true", help="从臂持续追随主臂当前姿态，而不是只对齐启动时姿态")
    p.add_argument("--standby-slave", action="store_true", help="对齐完成后把从臂切回 STANDBY")
    p.set_defaults(func=cmd_align_slave_to_master)

    p = sub.add_parser("go-home-both", help="通过 CAN 控制两台机械臂回到同一组初始关节角")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.add_argument("--home-deg", required=True, help="目标初始关节角，单位 deg，例如 2.94,4.04,-2.62,0.99,-1.91,-5.20")
    p.add_argument("--speed", type=int, default=8)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--step-deg", type=float, default=0.8)
    p.add_argument("--tol-deg", type=float, default=0.3)
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--keep-master-can", action="store_true", help="回 home 后不把主臂切回 STANDBY")
    p.add_argument("--standby-slave", action="store_true", help="回 home 后把从臂也切回 STANDBY")
    p.set_defaults(func=cmd_go_home_both)

    p = sub.add_parser("follow", help="启动主从跟随")
    p.add_argument("--master-can", required=True)
    p.add_argument("--slave-can", required=True)
    p.add_argument("--speed", type=int, default=15)
    p.add_argument("--rate", type=float, default=25.0)
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--alpha", type=float, default=0.35)
    p.add_argument("--mirror", default="1,1,1,1,1,1")
    p.add_argument("--follow-mode", choices=["relative", "absolute"], default="relative")
    p.add_argument("--joint-offset-deg", default="0,0,0,0,0,0", help="absolute 模式下从臂相对主臂的关节偏置，单位 deg")
    p.add_argument("--max-delta-deg", type=float, default=100.0)
    p.add_argument("--max-step-deg", type=float, default=3.0)
    p.add_argument("--master-deadband-deg", type=float, default=0.12)
    p.add_argument("--cmd-deadband-deg", type=float, default=0.08)

    p.add_argument("--sync-gripper", action="store_true", help="同步主动臂夹爪到从动臂夹爪")
    p.add_argument("--gripper-source", choices=["ctrl", "fb", "auto"], default="fb")
    p.add_argument("--gripper-min", type=int, default=10000)
    p.add_argument("--gripper-max", type=int, default=60000)
    p.add_argument("--gripper-scale", type=float, default=1.235)
    p.add_argument("--gripper-offset", type=int, default=-12716)
    p.add_argument("--gripper-effort", type=int, default=1000)
    p.add_argument("--gripper-invert", action="store_true")
    p.add_argument("--gripper-debug", action="store_true")
    p.set_defaults(func=cmd_follow)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
PY
