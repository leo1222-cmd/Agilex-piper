import time
import argparse
from dataclasses import dataclass
from piper_sdk import C_PiperInterface


@dataclass
class EndPose:
    x: int
    y: int
    z: int
    rx: int
    ry: int
    rz: int

    def copy(self):
        return EndPose(self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def readable(self):
        return (
            f"X={self.x/1000:.3f} mm | "
            f"Y={self.y/1000:.3f} mm | "
            f"Z={self.z/1000:.3f} mm | "
            f"RX={self.rx/1000:.3f} deg | "
            f"RY={self.ry/1000:.3f} deg | "
            f"RZ={self.rz/1000:.3f} deg"
        )


def connect_piper(can_name: str, wait: float = 1.0):
    piper = C_PiperInterface(can_name)
    piper.ConnectPort()
    time.sleep(wait)
    return piper


def enable_arm(piper, repeat=10):
    for _ in range(repeat):
        try:
            piper.EnableArm(7)
        except Exception:
            pass
        time.sleep(0.05)


def set_can_p(piper, speed=5):
    """
    CAN_CTRL + MOVE_P
    0x01: CAN_CTRL
    0x00: MOVE_P
    speed: 运动速度百分比，先用低速
    """
    for _ in range(5):
        piper.MotionCtrl_2(0x01, 0x00, speed, 0x00)
        time.sleep(0.05)


def get_end_pose(piper) -> EndPose:
    msg = piper.GetArmEndPoseMsgs()
    ep = msg.end_pose

    return EndPose(
        x=int(ep.X_axis),
        y=int(ep.Y_axis),
        z=int(ep.Z_axis),
        rx=int(ep.RX_axis),
        ry=int(ep.RY_axis),
        rz=int(ep.RZ_axis),
    )


def send_end_pose(piper, pose: EndPose):
    piper.EndPoseCtrl(
        pose.x,
        pose.y,
        pose.z,
        pose.rx,
        pose.ry,
        pose.rz,
    )


def wait_and_print(piper, name="", wait_s=1.0):
    time.sleep(wait_s)
    pose = get_end_pose(piper)
    if name:
        print(f"[{name}] {pose.readable()}")
    else:
        print(pose.readable())
    return pose


def move_and_return(piper, hold_pose: EndPose, test_pose: EndPose, name: str, wait_s=1.5):
    print("\n======================================")
    print(f"[TEST] {name}")
    print("[TARGET]", test_pose.readable())
    input("确认周围安全，按 Enter 执行这一步小运动...")

    send_end_pose(piper, test_pose)
    wait_and_print(piper, "after move", wait_s)

    input("确认安全，按 Enter 回到原始位置...")
    send_end_pose(piper, hold_pose)
    wait_and_print(piper, "after return", wait_s)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--slave-can", default="can1")
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--step-mm", type=float, default=1.0)
    parser.add_argument("--wait-s", type=float, default=1.5)
    args = parser.parse_args()

    step_raw = int(args.step_mm * 1000)

    print("========== Piper Cartesian Jog Test ==========")
    print("slave_can:", args.slave_can)
    print("speed:", args.speed)
    print("step_mm:", args.step_mm)
    print("step_raw:", step_raw)
    print("==============================================")
    print("注意：")
    print("1. 本脚本会控制从臂末端做 X/Y/Z 小步运动。")
    print("2. 每一步都需要按 Enter 确认。")
    print("3. 手放急停附近。")
    print("4. 不要同时运行主从 follow 程序。")
    print("==============================================")

    input("确认从臂周围安全，并且没有运行主从跟随程序，按 Enter 继续...")

    piper = connect_piper(args.slave_can)

    print("\n[INFO] 切 CAN_CTRL + MOVE_P，并使能从臂")
    set_can_p(piper, speed=args.speed)
    enable_arm(piper)

    time.sleep(0.5)

    hold_pose = get_end_pose(piper)
    print("\n[HOLD_POSE]", hold_pose.readable())

    input("\n确认记录当前位姿为 hold_pose，按 Enter 开始小步测试...")

    # X +1 mm
    p = hold_pose.copy()
    p.x += step_raw
    move_and_return(piper, hold_pose, p, f"X +{args.step_mm} mm", args.wait_s)

    # X -1 mm
    p = hold_pose.copy()
    p.x -= step_raw
    move_and_return(piper, hold_pose, p, f"X -{args.step_mm} mm", args.wait_s)

    # Y +1 mm
    p = hold_pose.copy()
    p.y += step_raw
    move_and_return(piper, hold_pose, p, f"Y +{args.step_mm} mm", args.wait_s)

    # Y -1 mm
    p = hold_pose.copy()
    p.y -= step_raw
    move_and_return(piper, hold_pose, p, f"Y -{args.step_mm} mm", args.wait_s)

    # Z +1 mm
    p = hold_pose.copy()
    p.z += step_raw
    move_and_return(piper, hold_pose, p, f"Z +{args.step_mm} mm", args.wait_s)

    # Z -1 mm
    p = hold_pose.copy()
    p.z -= step_raw
    move_and_return(piper, hold_pose, p, f"Z -{args.step_mm} mm", args.wait_s)

    print("\n[INFO] 最后回到 hold_pose")
    send_end_pose(piper, hold_pose)
    wait_and_print(piper, "final", args.wait_s)

    print("\n[DONE] 笛卡尔小步运动测试完成")


if __name__ == "__main__":
    main()
