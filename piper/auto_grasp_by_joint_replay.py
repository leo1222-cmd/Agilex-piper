import argparse
import json
import re
import sys
import time
from pathlib import Path

from piper_sdk import C_PiperInterface

from relative_pull_ik import (
    plan_relative_pull,
)


class AutoFlowAbort(Exception):
    def __init__(
        self,
        code,
        message,
    ):
        super().__init__(
            message
        )

        self.code = int(
            code
        )


def parse_joint_raw_from_msg(
    joint_msg
):
    text = str(
        joint_msg
    )

    vals = re.findall(
        r"Joint\s*\d+\s*:\s*(-?\d+)",
        text,
    )

    if len(vals) >= 6:
        return [
            int(v)
            for v
            in vals[:6]
        ]

    raise RuntimeError(
        "无法解析 6 个关节角，"
        "请检查 GetArmJointMsgs 输出。"
    )


def read_joint_raw(
    piper
):
    return (
        parse_joint_raw_from_msg(
            piper.GetArmJointMsgs()
        )
    )


def read_endpose_raw(
    piper
):
    msg = (
        piper.GetArmEndPoseMsgs()
    )

    ep = msg.end_pose

    return [
        int(
            ep.X_axis
        ),
        int(
            ep.Y_axis
        ),
        int(
            ep.Z_axis
        ),
        int(
            ep.RX_axis
        ),
        int(
            ep.RY_axis
        ),
        int(
            ep.RZ_axis
        ),
    ]


def joint_readable(
    joint_raw
):
    d = [
        v / 1000.0
        for v
        in joint_raw
    ]

    return (
        f"J1={d[0]:.3f}° | "
        f"J2={d[1]:.3f}° | "
        f"J3={d[2]:.3f}° | "
        f"J4={d[3]:.3f}° | "
        f"J5={d[4]:.3f}° | "
        f"J6={d[5]:.3f}°"
    )


def endpose_readable(
    pose_raw
):
    return (
        f"X="
        f"{pose_raw[0]/1000.0:.3f} mm | "

        f"Y="
        f"{pose_raw[1]/1000.0:.3f} mm | "

        f"Z="
        f"{pose_raw[2]/1000.0:.3f} mm | "

        f"RX="
        f"{pose_raw[3]/1000.0:.3f} deg | "

        f"RY="
        f"{pose_raw[4]/1000.0:.3f} deg | "

        f"RZ="
        f"{pose_raw[5]/1000.0:.3f} deg"
    )


def normalize_endpose_raw(
    value
):
    if isinstance(
        value,
        dict,
    ):
        keys = [
            "X_axis",
            "Y_axis",
            "Z_axis",
            "RX_axis",
            "RY_axis",
            "RZ_axis",
        ]

        return [
            int(
                value[k]
            )
            for k
            in keys
        ]

    if (
        isinstance(
            value,
            (
                list,
                tuple,
            ),
        )
        and
        len(value) >= 6
    ):
        return [
            int(v)
            for v
            in value[:6]
        ]

    raise RuntimeError(
        "无法解析 "
        "slave_endpose_raw: "
        f"{value!r}"
    )


def wrapped_angle_error_deg(
    target_raw,
    current_raw,
):
    target_deg = (
        float(
            target_raw
        )
        /
        1000.0
    )

    current_deg = (
        float(
            current_raw
        )
        /
        1000.0
    )

    return (
        (
            target_deg
            -
            current_deg
            +
            180.0
        )
        %
        360.0
    ) - 180.0


def check_grasp_endpose(
    piper,
    target_pose_raw,
    xyz_tol_mm=3.0,
    rpy_tol_deg=2.0,
):
    current = (
        read_endpose_raw(
            piper
        )
    )

    xyz_err = [
        (
            int(
                target_pose_raw[i]
            )
            -
            int(
                current[i]
            )
        )
        /
        1000.0
        for i
        in range(3)
    ]

    rpy_err = [
        wrapped_angle_error_deg(
            target_pose_raw[i],
            current[i],
        )
        for i
        in range(
            3,
            6,
        )
    ]

    max_xyz = max(
        abs(v)
        for v
        in xyz_err
    )

    max_rpy = max(
        abs(v)
        for v
        in rpy_err
    )

    print(
        "\n========== "
        "GRASP POSE CHECK "
        "=========="
    )

    print(
        "[TARGET]",
        endpose_readable(
            target_pose_raw
        )
    )

    print(
        "[ACTUAL]",
        endpose_readable(
            current
        )
    )

    print(
        "XYZ error mm:",
        [
            round(
                v,
                3,
            )
            for v
            in xyz_err
        ],
    )

    print(
        "RPY error deg:",
        [
            round(
                v,
                3,
            )
            for v
            in rpy_err
        ],
    )

    print(
        f"max_xyz_error_mm = "
        f"{max_xyz:.3f}"
    )

    print(
        f"max_rpy_error_deg = "
        f"{max_rpy:.3f}"
    )

    print(
        f"limits: "
        f"XYZ <= "
        f"{xyz_tol_mm:.3f} mm, "
        f"RPY <= "
        f"{rpy_tol_deg:.3f} deg"
    )

    print(
        "======================================"
    )

    ok = (
        max_xyz
        <=
        float(
            xyz_tol_mm
        )
        and
        max_rpy
        <=
        float(
            rpy_tol_deg
        )
    )

    if ok:
        print(
            "[GRASP POSE PASS] "
            "抓取位到位误差合格。"
        )

    else:
        print(
            "[GRASP POSE FAIL] "
            "抓取位到位误差过大。"
        )

    return ok


def max_joint_error_deg(
    current,
    target,
):
    return (
        max(
            abs(
                c - t
            )
            for c, t
            in zip(
                current,
                target,
            )
        )
        /
        1000.0
    )


def enable_arm(
    piper,
    repeat=10,
):
    for _ in range(
        repeat
    ):
        try:
            piper.EnableArm(
                7
            )

        except Exception:
            pass

        time.sleep(
            0.05
        )


def set_can_j(
    piper,
    speed=8,
):
    for _ in range(5):
        piper.MotionCtrl_2(
            0x01,
            0x01,
            int(
                speed
            ),
            0x00,
        )

        time.sleep(
            0.05
        )


def send_joint(
    piper,
    joint_raw,
):
    piper.JointCtrl(
        *[
            int(v)
            for v
            in joint_raw
        ]
    )


def step_towards(
    current,
    target,
    step_raw,
):
    out = []

    for c, t in zip(
        current,
        target,
    ):
        diff = (
            t - c
        )

        if diff > step_raw:
            out.append(
                c + step_raw
            )

        elif diff < -step_raw:
            out.append(
                c - step_raw
            )

        else:
            out.append(
                t
            )

    return out


def move_joint_slowly(
    piper,
    target_raw,
    speed=8,
    step_deg=1.0,
    tol_deg=0.3,
    rate=20,
    max_iters=1500,
    name="MOVE",
):
    step_raw = max(
        1,
        int(
            abs(
                step_deg
            )
            * 1000
        ),
    )

    dt = (
        1.0
        /
        max(
            float(
                rate
            ),
            1.0,
        )
    )

    set_can_j(
        piper,
        speed=speed,
    )

    enable_arm(
        piper
    )

    time.sleep(
        0.3
    )

    print(
        f"\n========== "
        f"{name} "
        f"=========="
    )

    print(
        "[CURRENT]",
        joint_readable(
            read_joint_raw(
                piper
            )
        ),
    )

    print(
        "[TARGET ]",
        joint_readable(
            target_raw
        ),
    )

    for i in range(
        max_iters
    ):
        current = (
            read_joint_raw(
                piper
            )
        )

        err = (
            max_joint_error_deg(
                current,
                target_raw,
            )
        )

        if (
            i % 10
            ==
            0
        ):
            print(
                f"[{name} "
                f"{i:04d}] "
                f"err="
                f"{err:.3f} deg | "
                f"{joint_readable(current)}"
            )

        if (
            err
            <=
            tol_deg
        ):
            print(
                f"[REACHED] "
                f"{name} "
                f"err="
                f"{err:.3f} deg"
            )

            return True

        send_joint(
            piper,
            step_towards(
                current,
                target_raw,
                step_raw,
            ),
        )

        time.sleep(
            dt
        )

    current = (
        read_joint_raw(
            piper
        )
    )

    print(
        f"[FAIL] "
        f"{name} "
        f"final_err="
        f"{max_joint_error_deg(current, target_raw):.3f} deg"
    )

    return False


def gripper_mm_to_raw(
    mm
):
    return int(
        float(
            mm
        )
        * 1000
    )


def open_gripper(
    piper,
    open_mm=60.0,
    effort=1000,
):
    raw = (
        gripper_mm_to_raw(
            open_mm
        )
    )

    print(
        f"\n[GRIPPER] "
        f"open to "
        f"{open_mm:.1f} mm, "
        f"raw={raw}"
    )

    for _ in range(5):
        piper.GripperCtrl(
            raw,
            int(
                effort
            ),
            0x01,
            0x00,
        )

        time.sleep(
            0.15
        )


def close_gripper(
    piper,
    close_mm=10.0,
    effort=1000,
):
    raw = (
        gripper_mm_to_raw(
            close_mm
        )
    )

    print(
        f"\n[GRIPPER] "
        f"close to "
        f"{close_mm:.1f} mm, "
        f"raw={raw}"
    )

    for _ in range(5):
        piper.GripperCtrl(
            raw,
            int(
                effort
            ),
            0x01,
            0x00,
        )

        time.sleep(
            0.15
        )


def load_pose_data(
    pose_file
):
    p = Path(
        pose_file
    )

    if not p.exists():
        raise RuntimeError(
            f"pose_file 不存在: "
            f"{pose_file}"
        )

    return json.loads(
        p.read_text()
    )


def load_target_pose(
    data,
    pose_name,
):
    if pose_name not in data:
        raise RuntimeError(
            f"pose_name="
            f"{pose_name} "
            "不存在"
        )

    record = data[
        pose_name
    ]

    if (
        "slave_joint_raw"
        not in record
    ):
        raise RuntimeError(
            f"{pose_name} "
            "中没有 "
            "slave_joint_raw"
        )

    if (
        "slave_endpose_raw"
        not in record
    ):
        raise RuntimeError(
            f"{pose_name} "
            "中没有 "
            "slave_endpose_raw，"
            "请重新示教该夹取位"
        )

    joint_raw = [
        int(v)
        for v
        in record[
            "slave_joint_raw"
        ]
    ]

    endpose_raw = (
        normalize_endpose_raw(
            record[
                "slave_endpose_raw"
            ]
        )
    )

    return (
        joint_raw,
        endpose_raw,
    )


def main():
    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(
        "--slave-can",
        default="can1",
    )

    parser.add_argument(
        "--pose-file",
        default=(
            "taught_full_grasp_poses.json"
        ),
    )

    parser.add_argument(
        "--pose-name",
        required=True,
    )

    parser.add_argument(
        "--speed",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--step-deg",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--tol-deg",
        type=float,
        default=0.3,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=20,
    )

    parser.add_argument(
        "--open-mm",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--close-mm",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--gripper-effort",
        type=int,
        default=1000,
    )

    parser.add_argument(
        "--close-wait-s",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--grasp-xyz-tol-mm",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--grasp-rpy-tol-deg",
        type=float,
        default=2.0,
    )

    # ========================================================
    # TWO-STAGE DYNAMIC PULL
    #
    # Stage 1:
    #   试剂仍在盒内，要求尽量直拔。
    #
    # Stage 2:
    #   试剂已经脱离盒体，允许更明显的小弧线。
    # ========================================================

    parser.add_argument(
        "--stage1-distance-mm",
        type=float,
        default=120.0,
    )

    parser.add_argument(
        "--stage1-speed",
        type=int,
        default=4,
    )

    parser.add_argument(
        "--stage1-step-deg",
        type=float,
        default=0.35,
    )

    parser.add_argument(
        "--stage1-final-z-tol-mm",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--stage1-critical-xy-mm",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--stage1-max-xy-mm",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--stage1-max-rpy-deg",
        type=float,
        default=8.0,
    )

    parser.add_argument(
        "--stage2-distance-mm",
        type=float,
        default=180.0,
    )

    parser.add_argument(
        "--stage2-speed",
        type=int,
        default=5,
    )

    parser.add_argument(
        "--stage2-step-deg",
        type=float,
        default=0.5,
    )

    parser.add_argument(
        "--stage2-final-z-tol-mm",
        type=float,
        default=8.0,
    )

    parser.add_argument(
        "--stage2-critical-z-mm",
        type=float,
        default=40.0,
    )

    parser.add_argument(
        "--stage2-critical-xy-mm",
        type=float,
        default=40.0,
    )

    parser.add_argument(
        "--stage2-max-xy-mm",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--stage2-max-rpy-deg",
        type=float,
        default=50.0,
    )

    parser.add_argument(
        "--return-speed",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--return-step-deg",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--auto",
        action="store_true",
    )

    args = (
        parser.parse_args()
    )

    pose_data = (
        load_pose_data(
            args.pose_file
        )
    )

    (
        target_joint_raw,
        target_endpose_raw,
    ) = load_target_pose(
        pose_data,
        args.pose_name,
    )

    print(
        "========== "
        "AUTO GRASP / POST / RETURN "
        "=========="
    )

    print(
        "slave_can:",
        args.slave_can,
    )

    print(
        "pose_name:",
        args.pose_name,
    )

    print(
        "target_joint:",
        joint_readable(
            target_joint_raw
        ),
    )

    print(
        "target_endpose:",
        endpose_readable(
            target_endpose_raw
        ),
    )

    print(
        "stage1_distance_mm:",
        args.stage1_distance_mm,
    )

    print(
        "stage2_distance_mm:",
        args.stage2_distance_mm,
    )

    print(
        "total_pull_mm:",
        args.stage1_distance_mm
        + args.stage2_distance_mm,
    )

    print(
        "================================================"
    )

    if not args.auto:
        input(
            "确认从臂、试剂、夹爪"
            "及运动路径安全，"
            "按 Enter 开始..."
        )

    piper = None

    hold_joints = None

    moved = False

    action_ok = False

    return_ok = True

    result_code = 10

    try:
        piper = C_PiperInterface(
            args.slave_can
        )

        piper.ConnectPort()

        time.sleep(
            1.0
        )

        hold_joints = (
            read_joint_raw(
                piper
            )
        )

        print(
            "\n[HOLD_JOINTS] "
            "自动动作开始位置："
        )

        print(
            joint_readable(
                hold_joints
            )
        )

        open_gripper(
            piper,
            args.open_mm,
            args.gripper_effort,
        )

        time.sleep(
            0.5
        )

        moved = True

        if not move_joint_slowly(
            piper,
            target_joint_raw,

            speed=args.speed,

            step_deg=args.step_deg,

            tol_deg=args.tol_deg,

            rate=args.rate,

            name="GO_GRASP",
        ):
            raise AutoFlowAbort(
                2,
                "未到达预设抓取位，"
                "不闭合夹爪",
            )

        print(
            "\n[INFO] "
            "已到预设抓取关节位；"
            "不进行二次 ArUco 检测。"
        )

        if not check_grasp_endpose(
            piper,
            target_endpose_raw,

            xyz_tol_mm=(
                args.grasp_xyz_tol_mm
            ),

            rpy_tol_deg=(
                args.grasp_rpy_tol_deg
            ),
        ):
            raise AutoFlowAbort(
                3,
                "抓取位 EndPose "
                "误差过大，"
                "本次不闭合夹爪",
            )

        # ====================================================
        # TWO-STAGE DYNAMIC PULL
        #
        # 第一阶段：
        # 当前真实抓取位 -> +120 mm
        # 强约束 XY / RPY
        #
        # 第二阶段：
        # 第一阶段真实终点 -> +180 mm
        # 试剂已基本脱离盒体，允许更明显小弧线
        # ====================================================

        pull_start_joint_raw = read_joint_raw(
            piper
        )

        pull_start_pose_raw = read_endpose_raw(
            piper
        )

        print(
            "\n========== TWO-STAGE PULL PLAN =========="
        )

        print(
            "[START JOINT]",
            joint_readable(
                pull_start_joint_raw
            ),
        )

        print(
            "[START POSE ]",
            endpose_readable(
                pull_start_pose_raw
            ),
        )


        # ====================================================
        # STAGE 1 PLAN
        # 先规划，再闭夹。
        # 如果第一阶段都没有安全解，就不夹试剂。
        # ====================================================

        print(
            "\n[STAGE1 PLAN] "
            f"+{args.stage1_distance_mm:.1f} mm"
        )

        stage1_plan = plan_relative_pull(
            current_joint_raw=(
                pull_start_joint_raw
            ),

            current_endpose_raw=(
                pull_start_pose_raw
            ),

            distance_mm=(
                args.stage1_distance_mm
            ),

            final_z_tol_mm=(
                args.stage1_final_z_tol_mm
            ),

            critical_z_mm=(
                args.stage1_distance_mm
            ),

            critical_xy_drift_mm=(
                args.stage1_critical_xy_mm
            ),

            max_xy_drift_mm=(
                args.stage1_max_xy_mm
            ),

            max_rpy_drift_deg=(
                args.stage1_max_rpy_deg
            ),
        )

        stage1_target_raw = [
            int(
                round(
                    v * 1000.0
                )
            )
            for v
            in stage1_plan.target_joint_deg
        ]

        print(
            "[STAGE1 TARGET]",
            joint_readable(
                stage1_target_raw
            ),
        )

        print(
            f"[STAGE1 PLAN] "
            f"dz="
            f"{stage1_plan.final_dz_mm:.3f} mm"
        )

        print(
            f"[STAGE1 PLAN] "
            f"max_xy="
            f"{stage1_plan.max_xy_drift_mm:.3f} mm"
        )

        print(
            f"[STAGE1 PLAN] "
            f"max_rpy="
            f"{stage1_plan.max_rpy_drift_deg:.3f} deg"
        )

        print(
            f"[STAGE1 PLAN] "
            f"target_J5="
            f"{stage1_plan.target_joint_deg[4]:.3f} deg"
        )


        # ====================================================
        # 在夹取前，用 Stage1 的预测终点预检查 Stage2。
        #
        # 这样如果整个 300 mm 根本不可达，
        # 不会先把试剂夹起来再发现第二阶段无解。
        # ====================================================

        predicted_stage1_pose_raw = [
            int(
                round(
                    v * 1000.0
                )
            )
            for v
            in stage1_plan.target_pose_mm_deg
        ]

        print(
            "\n[STAGE2 PRECHECK] "
            f"+{args.stage2_distance_mm:.1f} mm"
        )

        stage2_precheck = plan_relative_pull(
            current_joint_raw=(
                stage1_target_raw
            ),

            current_endpose_raw=(
                predicted_stage1_pose_raw
            ),

            distance_mm=(
                args.stage2_distance_mm
            ),

            final_z_tol_mm=(
                args.stage2_final_z_tol_mm
            ),

            critical_z_mm=(
                args.stage2_critical_z_mm
            ),

            critical_xy_drift_mm=(
                args.stage2_critical_xy_mm
            ),

            max_xy_drift_mm=(
                args.stage2_max_xy_mm
            ),

            max_rpy_drift_deg=(
                args.stage2_max_rpy_deg
            ),
        )

        print(
            "[STAGE2 PRECHECK] PASS"
        )

        print(
            f"[STAGE2 PRECHECK] "
            f"dz="
            f"{stage2_precheck.final_dz_mm:.3f} mm"
        )

        print(
            f"[STAGE2 PRECHECK] "
            f"max_xy="
            f"{stage2_precheck.max_xy_drift_mm:.3f} mm"
        )

        print(
            f"[STAGE2 PRECHECK] "
            f"max_rpy="
            f"{stage2_precheck.max_rpy_drift_deg:.3f} deg"
        )

        print(
            f"[STAGE2 PRECHECK] "
            f"target_J5="
            f"{stage2_precheck.target_joint_deg[4]:.3f} deg"
        )


        # ====================================================
        # 两阶段规划均有解以后才真正闭夹
        # ====================================================

        close_gripper(
            piper,
            args.close_mm,
            args.gripper_effort,
        )

        print(
            f"[INFO] "
            f"夹爪闭合后等待 "
            f"{args.close_wait_s:.1f} s"
        )

        time.sleep(
            args.close_wait_s
        )


        # ====================================================
        # EXECUTE STAGE 1
        # ====================================================

        print(
            "\n========== "
            "PULL STAGE 1 "
            "=========="
        )

        if not move_joint_slowly(
            piper,
            stage1_target_raw,

            speed=(
                args.stage1_speed
            ),

            step_deg=(
                args.stage1_step_deg
            ),

            tol_deg=(
                args.tol_deg
            ),

            rate=(
                args.rate
            ),

            name="PULL_STAGE1",
        ):
            raise AutoFlowAbort(
                4,
                "第一阶段 +120 mm "
                "拔出目标未到位",
            )

        stage1_actual_joint_raw = read_joint_raw(
            piper
        )

        stage1_actual_pose_raw = read_endpose_raw(
            piper
        )

        stage1_actual_dz = (
            (
                stage1_actual_pose_raw[2]
                -
                pull_start_pose_raw[2]
            )
            /
            1000.0
        )

        stage1_actual_dx = (
            (
                stage1_actual_pose_raw[0]
                -
                pull_start_pose_raw[0]
            )
            /
            1000.0
        )

        stage1_actual_dy = (
            (
                stage1_actual_pose_raw[1]
                -
                pull_start_pose_raw[1]
            )
            /
            1000.0
        )

        stage1_actual_xy = (
            stage1_actual_dx ** 2
            +
            stage1_actual_dy ** 2
        ) ** 0.5

        print(
            "\n[STAGE1 ACTUAL POSE]",
            endpose_readable(
                stage1_actual_pose_raw
            ),
        )

        print(
            f"[STAGE1 RESULT] "
            f"actual_dz="
            f"{stage1_actual_dz:.3f} mm"
        )

        print(
            f"[STAGE1 RESULT] "
            f"actual_xy="
            f"{stage1_actual_xy:.3f} mm"
        )

        if (
            abs(
                stage1_actual_dz
                -
                args.stage1_distance_mm
            )
            >
            args.stage1_final_z_tol_mm
        ):
            raise AutoFlowAbort(
                5,
                "第一阶段实际上升距离异常: "
                f"{stage1_actual_dz:.3f} mm",
            )


        # ====================================================
        # STAGE 2
        #
        # 这里不直接使用之前的预测目标。
        # 第一阶段完成以后重新读取机器人真实位置，
        # 再基于真实位置重新规划第二阶段。
        # ====================================================

        print(
            "\n[STAGE2 PLAN] "
            "基于第一阶段真实终点重新规划..."
        )

        stage2_plan = plan_relative_pull(
            current_joint_raw=(
                stage1_actual_joint_raw
            ),

            current_endpose_raw=(
                stage1_actual_pose_raw
            ),

            distance_mm=(
                args.stage2_distance_mm
            ),

            final_z_tol_mm=(
                args.stage2_final_z_tol_mm
            ),

            critical_z_mm=(
                args.stage2_critical_z_mm
            ),

            critical_xy_drift_mm=(
                args.stage2_critical_xy_mm
            ),

            max_xy_drift_mm=(
                args.stage2_max_xy_mm
            ),

            max_rpy_drift_deg=(
                args.stage2_max_rpy_deg
            ),
        )

        stage2_target_raw = [
            int(
                round(
                    v * 1000.0
                )
            )
            for v
            in stage2_plan.target_joint_deg
        ]

        print(
            "[STAGE2 TARGET]",
            joint_readable(
                stage2_target_raw
            ),
        )

        print(
            f"[STAGE2 PLAN] "
            f"dz="
            f"{stage2_plan.final_dz_mm:.3f} mm"
        )

        print(
            f"[STAGE2 PLAN] "
            f"max_xy="
            f"{stage2_plan.max_xy_drift_mm:.3f} mm"
        )

        print(
            f"[STAGE2 PLAN] "
            f"max_rpy="
            f"{stage2_plan.max_rpy_drift_deg:.3f} deg"
        )

        print(
            f"[STAGE2 PLAN] "
            f"target_J5="
            f"{stage2_plan.target_joint_deg[4]:.3f} deg"
        )


        # ====================================================
        # EXECUTE STAGE 2
        # ====================================================

        print(
            "\n========== "
            "PULL STAGE 2 "
            "=========="
        )

        if not move_joint_slowly(
            piper,
            stage2_target_raw,

            speed=(
                args.stage2_speed
            ),

            step_deg=(
                args.stage2_step_deg
            ),

            tol_deg=(
                args.tol_deg
            ),

            rate=(
                args.rate
            ),

            name="PULL_STAGE2",
        ):
            raise AutoFlowAbort(
                6,
                "第二阶段 +180 mm "
                "拔出目标未到位",
            )


        # ====================================================
        # FINAL CHECK
        # ====================================================

        final_pose_raw = read_endpose_raw(
            piper
        )

        final_dz = (
            (
                final_pose_raw[2]
                -
                pull_start_pose_raw[2]
            )
            /
            1000.0
        )

        final_dx = (
            (
                final_pose_raw[0]
                -
                pull_start_pose_raw[0]
            )
            /
            1000.0
        )

        final_dy = (
            (
                final_pose_raw[1]
                -
                pull_start_pose_raw[1]
            )
            /
            1000.0
        )

        final_xy = (
            final_dx ** 2
            +
            final_dy ** 2
        ) ** 0.5

        expected_total = (
            args.stage1_distance_mm
            +
            args.stage2_distance_mm
        )

        total_tol = (
            args.stage1_final_z_tol_mm
            +
            args.stage2_final_z_tol_mm
        )

        print(
            "\n========== "
            "TWO-STAGE PULL RESULT "
            "=========="
        )

        print(
            "[FINAL POSE]",
            endpose_readable(
                final_pose_raw
            ),
        )

        print(
            f"actual_total_dz = "
            f"{final_dz:.3f} mm"
        )

        print(
            f"actual_final_xy_drift = "
            f"{final_xy:.3f} mm"
        )

        print(
            f"expected_total_dz = "
            f"{expected_total:.3f} mm"
        )

        print(
            "==========================================="
        )

        if (
            abs(
                final_dz
                -
                expected_total
            )
            >
            total_tol
        ):
            raise AutoFlowAbort(
                7,
                "总拔出高度异常: "
                f"actual="
                f"{final_dz:.3f} mm, "
                f"expected="
                f"{expected_total:.3f} mm",
            )

        action_ok = True

        result_code = 0

        print(
            "\n[AUTO PASS] "
            "抓取 + "
            "动态计算小弧线拔出完成。"
        )

    except AutoFlowAbort as e:
        result_code = (
            e.code
        )

        print(
            f"\n[AUTO FAIL] "
            f"{e}"
        )

    except KeyboardInterrupt:
        result_code = 130

        print(
            "\n[USER STOP] "
            "用户中断。"
        )

    except Exception as e:
        result_code = 10

        print(
            "\n[ERROR] "
            "自动夹取流程异常:",
            repr(e),
        )

    finally:
        if (
            piper
            is not None
            and
            hold_joints
            is not None
            and
            moved
        ):
            return_speed = (
                args.return_speed
                if
                args.return_speed
                is not None
                else
                args.speed
            )

            return_step = (
                args.return_step_deg
                if
                args.return_step_deg
                is not None
                else
                args.step_deg
            )

            print(
                "\n[RETURN] "
                "回到自动动作开始前"
                "的从臂位置"
            )

            print(
                "[RETURN TARGET]",
                joint_readable(
                    hold_joints
                ),
            )

            try:
                return_ok = (
                    move_joint_slowly(
                        piper,
                        hold_joints,

                        speed=(
                            return_speed
                        ),

                        step_deg=(
                            return_step
                        ),

                        tol_deg=(
                            args.tol_deg
                        ),

                        rate=(
                            args.rate
                        ),

                        name=(
                            "RETURN_HOLD"
                        ),
                    )
                )

            except Exception as e:
                print(
                    "[RETURN ERROR]",
                    repr(e),
                )

                return_ok = False

            print(
                "[RETURN RESULT]",
                (
                    "PASS"
                    if return_ok
                    else "FAIL"
                ),
            )

            print(
                "[ACTION RESULT]",
                (
                    "PASS"
                    if action_ok
                    else "FAIL"
                ),
            )

            if (
                not return_ok
                and
                result_code == 0
            ):
                result_code = 20

    return result_code


if __name__ == "__main__":
    sys.exit(
        main()
    )
