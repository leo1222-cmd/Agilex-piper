#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time
from pathlib import Path

import numpy as np

import auto_grasp_by_joint_replay as grasp
from relative_pull_ik import plan_relative_pull


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def parse_endpose_raw(record):
    ep = record["slave_endpose_raw"]

    if isinstance(ep, dict):
        return [
            int(ep["X_axis"]),
            int(ep["Y_axis"]),
            int(ep["Z_axis"]),
            int(ep["RX_axis"]),
            int(ep["RY_axis"]),
            int(ep["RZ_axis"]),
        ]

    return [int(v) for v in ep]


def max_joint_diff_deg(a_raw, b_raw):
    return max(
        abs(int(a) - int(b)) / 1000.0
        for a, b in zip(a_raw, b_raw)
    )


def open_gripper(piper, open_mm, effort):
    print(
        f"[GRIPPER] open to "
        f"{open_mm:.1f} mm"
    )

    grasp.open_gripper(
        piper,
        open_mm=float(open_mm),
        effort=int(effort),
    )


def build_preinsert_plan(
    grasp_joint_raw,
    grasp_endpose_raw,
    cfg,
):
    """
    由已经示教并验证过的抓取位，
    动态计算抓取位上方 +distance_mm 的预插入位。

    这里使用的就是抓取/拔出阶段已经验证过的
    relative_pull_ik.py。
    """

    return plan_relative_pull(
        current_joint_raw=grasp_joint_raw,
        current_endpose_raw=grasp_endpose_raw,

        distance_mm=float(
            cfg["distance_mm"]
        ),

        final_z_tol_mm=float(
            cfg.get(
                "final_z_tol_mm",
                5.0,
            )
        ),

        critical_z_mm=float(
            cfg.get(
                "critical_z_mm",
                cfg["distance_mm"],
            )
        ),

        critical_xy_drift_mm=float(
            cfg.get(
                "critical_xy_drift_mm",
                10.0,
            )
        ),

        max_xy_drift_mm=float(
            cfg.get(
                "max_xy_drift_mm",
                12.0,
            )
        ),

        max_rpy_drift_deg=float(
            cfg.get(
                "max_rpy_drift_deg",
                8.5,
            )
        ),
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--slave-can",
        default="can1",
    )

    parser.add_argument(
        "--pose-file",
        default="taught_full_grasp_poses.json",
    )

    parser.add_argument(
        "--pose-name",
        required=True,
    )

    parser.add_argument(
        "--distance-mm",
        type=float,
        default=120.0,
    )

    parser.add_argument(
        "--approach-speed",
        type=int,
        default=5,
    )

    parser.add_argument(
        "--approach-step-deg",
        type=float,
        default=0.7,
    )

    parser.add_argument(
        "--insert-speed",
        type=int,
        default=3,
    )

    parser.add_argument(
        "--insert-step-deg",
        type=float,
        default=0.25,
    )

    parser.add_argument(
        "--retract-speed",
        type=int,
        default=4,
    )

    parser.add_argument(
        "--retract-step-deg",
        type=float,
        default=0.35,
    )

    parser.add_argument(
        "--tol-deg",
        type=float,
        default=0.3,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--open-mm",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--gripper-effort",
        type=int,
        default=1000,
    )

    parser.add_argument(
        "--release-wait-s",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--max-start-joint-error-deg",
        type=float,
        default=25.0,
    )

    parser.add_argument(
        "--final-z-tol-mm",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--critical-xy-mm",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--max-xy-mm",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--max-rpy-deg",
        type=float,
        default=8.5,
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
    )

    args = parser.parse_args()

    poses = load_json(
        args.pose_file
    )

    if args.pose_name not in poses:
        raise RuntimeError(
            f"pose_name 不存在: "
            f"{args.pose_name}"
        )

    record = poses[
        args.pose_name
    ]

    grasp_joint_raw = [
        int(v)
        for v in record[
            "slave_joint_raw"
        ]
    ]

    grasp_endpose_raw = (
        parse_endpose_raw(
            record
        )
    )

    print(
        "========== AUTO RETURN / REINSERT =========="
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
        "[INSERT TARGET JOINT]",
        grasp.joint_readable(
            grasp_joint_raw
        ),
    )

    print(
        "[INSERT TARGET POSE ]",
        grasp.endpose_readable(
            grasp_endpose_raw
        ),
    )

    print(
        "return_distance_mm:",
        args.distance_mm,
    )

    print(
        "============================================"
    )

    plan_cfg = {
        "distance_mm":
            args.distance_mm,

        "final_z_tol_mm":
            args.final_z_tol_mm,

        "critical_z_mm":
            args.distance_mm,

        "critical_xy_drift_mm":
            args.critical_xy_mm,

        "max_xy_drift_mm":
            args.max_xy_mm,

        "max_rpy_drift_deg":
            args.max_rpy_deg,
    }

    # =========================================================
    # 根据示教抓取位计算 +120 mm 的预插入位置
    # =========================================================

    plan = build_preinsert_plan(
        grasp_joint_raw,
        grasp_endpose_raw,
        plan_cfg,
    )

    pre_joint_raw = [
        int(
            round(
                v * 1000.0
            )
        )
        for v in plan.target_joint_deg
    ]

    print()
    print(
        "========== RETURN PRE-POSE PLAN =========="
    )

    print(
        "[GRASP ]",
        grasp.joint_readable(
            grasp_joint_raw
        ),
    )

    print(
        "[PRE   ]",
        grasp.joint_readable(
            pre_joint_raw
        ),
    )

    print(
        f"predicted_dz = "
        f"{plan.final_dz_mm:.3f} mm"
    )

    print(
        f"predicted_critical_xy = "
        f"{plan.critical_xy_drift_mm:.3f} mm"
    )

    print(
        f"predicted_max_xy = "
        f"{plan.max_xy_drift_mm:.3f} mm"
    )

    print(
        f"predicted_max_rpy = "
        f"{plan.max_rpy_drift_deg:.3f} deg"
    )

    print(
        f"target_J5 = "
        f"{plan.target_joint_deg[4]:.3f} deg"
    )

    print(
        "=========================================="
    )

    if args.dry_run:
        print()
        print(
            "[DRY RUN PASS] "
            "只完成预插入位计算，未连接真机。"
        )
        return

    # =========================================================
    # 真机
    # =========================================================

    piper = grasp.C_PiperInterface(
        args.slave_can
    )

    piper.ConnectPort()

    time.sleep(
        1.0
    )

    current_raw = (
        grasp.read_joint_raw(
            piper
        )
    )

    # ---------------------------------------------------------
    # 保存按 r 时的从臂位置。
    # 自动放回完成后必须回到这里，
    # 然后总控才能恢复 absolute 主从。
    # ---------------------------------------------------------

    slave_hold_joints = current_raw.copy()

    print()
    print(
        "[HOLD_JOINTS] "
        "按 r 时的主从暂停位置："
    )
    print(
        grasp.joint_readable(
            slave_hold_joints
        )
    )

    print()
    print(
        "[CURRENT]",
        grasp.joint_readable(
            current_raw
        ),
    )

    start_err = (
        max_joint_diff_deg(
            current_raw,
            pre_joint_raw,
        )
    )

    print(
        f"[PRE CHECK] "
        f"current → pre max joint error "
        f"= {start_err:.3f} deg"
    )

    # 防止操作者在距离试剂盒过远的位置直接按 r，
    # 导致机器人自动做一次很大的跨工作区运动。
    if (
        start_err
        >
        args.max_start_joint_error_deg
    ):
        raise RuntimeError(
            "当前从臂距离预插入位过远，拒绝自动放回。"
            f" current_to_pre="
            f"{start_err:.3f} deg,"
            f" limit="
            f"{args.max_start_joint_error_deg:.3f} deg。"
            " 请先通过主从把试剂移动到对应槽位上方附近。"
        )

    # =========================================================
    # STEP 1
    # 当前人工位置 -> 预插入位置
    # =========================================================

    print()
    print(
        "========== RETURN_APPROACH =========="
    )

    ok = grasp.move_joint_slowly(
        piper=piper,

        target_raw=pre_joint_raw,

        speed=args.approach_speed,

        step_deg=args.approach_step_deg,

        tol_deg=args.tol_deg,

        rate=args.rate,

        max_iters=1500,

        name="RETURN_APPROACH",
    )

    if not ok:
        raise RuntimeError(
            "RETURN_APPROACH 未到位，"
            "停止自动放回。"
        )

    # =========================================================
    # STEP 2
    # +120 mm 预插入位 -> 原抓取位
    #
    # 这一步就是原拔出 Stage1 的反向。
    # 保持夹爪闭合。
    # =========================================================

    print()
    print(
        "========== REVERSE_PULL_INSERT =========="
    )

    ok = grasp.move_joint_slowly(
        piper=piper,

        target_raw=grasp_joint_raw,

        speed=args.insert_speed,

        step_deg=args.insert_step_deg,

        tol_deg=args.tol_deg,

        rate=args.rate,

        max_iters=1500,

        name="REVERSE_PULL_INSERT",
    )

    if not ok:
        raise RuntimeError(
            "REVERSE_PULL_INSERT 未到位。"
            "不打开夹爪，请人工检查。"
        )

    # =========================================================
    # STEP 3
    # 插入到位 -> 打开夹爪释放试剂
    # =========================================================

    print()
    print(
        "========== RELEASE =========="
    )

    open_gripper(
        piper,
        args.open_mm,
        args.gripper_effort,
    )

    print(
        f"[RELEASE] 等待 "
        f"{args.release_wait_s:.1f} s"
    )

    time.sleep(
        args.release_wait_s
    )

    # =========================================================
    # STEP 4
    # 原抓取位 -> +120 mm 预插入位
    #
    # 试剂已经留在槽内，
    # 机械臂沿原轨迹退出。
    # =========================================================

    print()
    print(
        "========== RETRACT =========="
    )

    ok = grasp.move_joint_slowly(
        piper=piper,

        target_raw=pre_joint_raw,

        speed=args.retract_speed,

        step_deg=args.retract_step_deg,

        tol_deg=args.tol_deg,

        rate=args.rate,

        max_iters=1500,

        name="RETURN_RETRACT",
    )

    if not ok:
        raise RuntimeError(
            "RETURN_RETRACT 未完全到位，"
            "请人工检查。"
        )

    # =========================================================
    # STEP 5
    # PRE -> 按 r 时保存的 HOLD
    #
    # 只有真正回到 HOLD，
    # 总控才允许恢复 absolute follow。
    # =========================================================

    print()
    print(
        "========== RETURN_HOLD =========="
    )

    print(
        "[HOLD TARGET]",
        grasp.joint_readable(
            slave_hold_joints
        ),
    )

    ok = grasp.move_joint_slowly(
        piper=piper,

        target_raw=slave_hold_joints,

        speed=args.approach_speed,

        step_deg=args.approach_step_deg,

        tol_deg=args.tol_deg,

        rate=args.rate,

        max_iters=2000,

        name="RETURN_HOLD",
    )

    if not ok:
        raise RuntimeError(
            "RETURN_HOLD 未到位。"
            "禁止自动恢复主从，"
            "请人工检查。"
        )

    final_raw = (
        grasp.read_joint_raw(
            piper
        )
    )

    final_err = (
        max_joint_diff_deg(
            final_raw,
            slave_hold_joints,
        )
    )

    print()
    print(
        "========== AUTO RETURN RESULT =========="
    )

    print(
        "[HOLD TARGET]",
        grasp.joint_readable(
            slave_hold_joints
        ),
    )

    print(
        "[FINAL     ]",
        grasp.joint_readable(
            final_raw
        ),
    )

    print(
        f"final_hold_error_deg = "
        f"{final_err:.3f}"
    )

    print(
        "return_result = PASS"
    )

    print(
        "========================================"
    )


if __name__ == "__main__":
    main()
