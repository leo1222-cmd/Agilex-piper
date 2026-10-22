import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


BASE_DIR = (
    Path(__file__)
    .resolve()
    .parent
)


def load_json(path):
    p = Path(
        path
    )

    if not p.is_absolute():
        p = (
            BASE_DIR
            /
            p
        )

    return json.loads(
        p.read_text()
    )


def create_aruco_detector(
    dict_name
):
    aruco = (
        cv2.aruco
    )

    dict_map = {
        "4x4_50":
            aruco.DICT_4X4_50,

        "4x4_100":
            aruco.DICT_4X4_100,

        "4x4_250":
            aruco.DICT_4X4_250,

        "4x4_1000":
            aruco.DICT_4X4_1000,
    }

    if (
        dict_name
        not in dict_map
    ):
        raise RuntimeError(
            "不支持的 ArUco 字典: "
            f"{dict_name}"
        )

    dictionary = (
        aruco.getPredefinedDictionary(
            dict_map[
                dict_name
            ]
        )
    )

    parameters = (
        aruco.DetectorParameters()
        if hasattr(
            aruco,
            "DetectorParameters",
        )
        else
        aruco.DetectorParameters_create()
    )

    detector = (
        aruco.ArucoDetector(
            dictionary,
            parameters,
        )
        if hasattr(
            aruco,
            "ArucoDetector",
        )
        else
        None
    )

    return (
        dictionary,
        parameters,
        detector,
    )


def detect_markers(
    gray,
    dictionary,
    parameters,
    detector,
):
    if (
        detector
        is not None
    ):
        return (
            detector.detectMarkers(
                gray
            )
        )

    return (
        cv2.aruco.detectMarkers(
            gray,
            dictionary,
            parameters=parameters,
        )
    )


class FollowProcess:
    def __init__(
        self,
        config,
        show_log=False,
    ):
        self.config = config
        self.proc = None
        self.show_log = show_log
        self.log_file = None

    def run_align(
        self
    ):
        align_cfg = (
            self.config.get(
                "align",
                {},
            )
        )

        if not align_cfg.get(
            "enabled",
            True,
        ):
            return

        f = (
            self.config[
                "follow"
            ]
        )

        cmd = [
            sys.executable,
            "ms.py",
            "align-slave-to-master",

            "--master-can",
            str(
                f.get(
                    "master_can",
                    "can0",
                )
            ),

            "--slave-can",
            str(
                f.get(
                    "slave_can",
                    "can1",
                )
            ),

            "--speed",
            str(
                align_cfg.get(
                    "speed",
                    8,
                )
            ),

            "--rate",
            str(
                align_cfg.get(
                    "rate",
                    20,
                )
            ),

            "--step-deg",
            str(
                align_cfg.get(
                    "step_deg",
                    0.8,
                )
            ),

            "--tol-deg",
            str(
                align_cfg.get(
                    "tol_deg",
                    0.3,
                )
            ),
        ]

        print(
            "\n[ALIGN] "
            "启动前先让从臂"
            "对齐主动臂"
        )

        input(
            "[ALIGN] "
            "确认路径安全，"
            "按 Enter 开始..."
        )

        subprocess.run(
            cmd,
            cwd=str(
                BASE_DIR
            ),
        )

        time.sleep(
            0.5
        )

    def build_cmd(
        self
    ):
        f = (
            self.config[
                "follow"
            ]
        )

        cmd = [
            sys.executable,
            "ms.py",
            "follow",

            "--master-can",
            str(
                f.get(
                    "master_can",
                    "can0",
                )
            ),

            "--slave-can",
            str(
                f.get(
                    "slave_can",
                    "can1",
                )
            ),

            "--follow-mode",
            str(
                f.get(
                    "follow_mode",
                    "absolute",
                )
            ),

            "--speed",
            str(
                f.get(
                    "speed",
                    12,
                )
            ),

            "--rate",
            str(
                f.get(
                    "rate",
                    25,
                )
            ),

            "--alpha",
            str(
                f.get(
                    "alpha",
                    0.30,
                )
            ),

            "--max-step-deg",
            str(
                f.get(
                    "max_step_deg",
                    2.0,
                )
            ),

            "--cmd-deadband-deg",
            str(
                f.get(
                    "cmd_deadband_deg",
                    0.08,
                )
            ),

            "--gripper-source",
            str(
                f.get(
                    "gripper_source",
                    "fb",
                )
            ),

            "--gripper-min",
            str(
                f.get(
                    "gripper_min",
                    10000,
                )
            ),

            "--gripper-max",
            str(
                f.get(
                    "gripper_max",
                    60000,
                )
            ),

            "--gripper-scale",
            str(
                f.get(
                    "gripper_scale",
                    1.235,
                )
            ),

            "--gripper-offset",
            str(
                f.get(
                    "gripper_offset",
                    -12716,
                )
            ),

            "--gripper-effort",
            str(
                f.get(
                    "gripper_effort",
                    1000,
                )
            ),

            "--mirror",
            str(
                f.get(
                    "mirror",
                    "1,1,1,1,1,1",
                )
            ),

            "--joint-offset-deg",
            str(
                f.get(
                    "joint_offset_deg",
                    "0,0,0,0,0,0",
                )
            ),
        ]

        if f.get(
            "sync_gripper",
            True,
        ):
            cmd.append(
                "--sync-gripper"
            )

        return cmd

    def start(
        self,
        do_align=True,
    ):
        if (
            self.proc
            is not None
            and
            self.proc.poll()
            is None
        ):
            return

        if do_align:
            self.run_align()

        else:
            print(
                "\n[FOLLOW] "
                "自动动作结束，"
                "直接恢复主从，"
                "不二次对齐"
            )

        cmd = (
            self.build_cmd()
        )

        if self.show_log:
            stdout = None
            stderr = None

        else:
            self.log_file = open(
                BASE_DIR
                /
                "operator_follow.log",
                "a",
                buffering=1,
            )

            stdout = (
                self.log_file
            )

            stderr = (
                self.log_file
            )

        print(
            "[FOLLOW] "
            "启动主从控制"
        )

        self.proc = (
            subprocess.Popen(
                cmd,

                cwd=str(
                    BASE_DIR
                ),

                stdout=stdout,
                stderr=stderr,

                preexec_fn=(
                    os.setsid
                ),
            )
        )

        time.sleep(
            1.0
        )

    def stop(
        self
    ):
        if (
            self.proc
            is None
        ):
            return

        if (
            self.proc.poll()
            is not None
        ):
            self.proc = None
            return

        print(
            "\n[FOLLOW] "
            "暂停主从控制"
        )

        try:
            os.killpg(
                os.getpgid(
                    self.proc.pid
                ),
                signal.SIGINT,
            )

            self.proc.wait(
                timeout=5
            )

        except Exception:
            try:
                os.killpg(
                    os.getpgid(
                        self.proc.pid
                    ),
                    signal.SIGTERM,
                )

                self.proc.wait(
                    timeout=3
                )

            except Exception:
                try:
                    os.killpg(
                        os.getpgid(
                            self.proc.pid
                        ),
                        signal.SIGKILL,
                    )

                except Exception:
                    pass

        self.proc = None

        if (
            self.log_file
            is not None
        ):
            try:
                self.log_file.close()

            except Exception:
                pass

            self.log_file = None

        time.sleep(
            0.5
        )


class CameraPanel:
    def __init__(
        self,
        config,
    ):
        self.config = config

        self.pipeline = None

        self.align = None

        self.dictionary = None

        self.parameters = None

        self.detector = None

    def start(
        self
    ):
        cam_cfg = (
            self.config.get(
                "camera",
                {},
            )
        )

        (
            self.dictionary,
            self.parameters,
            self.detector,
        ) = create_aruco_detector(
            cam_cfg.get(
                "dict",
                "4x4_50",
            )
        )

        self.pipeline = (
            rs.pipeline()
        )

        rs_cfg = (
            rs.config()
        )

        rs_cfg.enable_stream(
            rs.stream.color,
            640,
            480,
            rs.format.bgr8,
            30,
        )

        rs_cfg.enable_stream(
            rs.stream.depth,
            640,
            480,
            rs.format.z16,
            30,
        )

        self.pipeline.start(
            rs_cfg
        )

        self.align = rs.align(
            rs.stream.color
        )

        print(
            "[CAMERA] "
            "相机已开启；"
            "自动夹取期间也不会关闭"
        )

    def stop(
        self
    ):
        try:
            if (
                self.pipeline
                is not None
            ):
                self.pipeline.stop()

        except Exception:
            pass

        self.pipeline = None

        cv2.destroyAllWindows()

    def read_frame_and_detect(
        self
    ):
        frames = (
            self.pipeline
            .wait_for_frames()
        )

        frames = (
            self.align.process(
                frames
            )
        )

        color_frame = (
            frames.get_color_frame()
        )

        if not color_frame:
            return (
                None,
                [],
            )

        color = np.asanyarray(
            color_frame.get_data()
        )

        gray = cv2.cvtColor(
            color,
            cv2.COLOR_BGR2GRAY,
        )

        (
            corners,
            ids,
            _,
        ) = detect_markers(
            gray,
            self.dictionary,
            self.parameters,
            self.detector,
        )

        detected_ids = []

        if (
            ids
            is not None
            and
            len(ids) > 0
        ):
            detected_ids = [
                int(x)
                for x
                in ids.flatten().tolist()
            ]

            cv2.aruco.drawDetectedMarkers(
                color,
                corners,
                ids,
            )

            for (
                i,
                marker_id,
            ) in enumerate(
                detected_ids
            ):
                c = (
                    corners[i][0]
                )

                cx = int(
                    np.mean(
                        c[:, 0]
                    )
                )

                cy = int(
                    np.mean(
                        c[:, 1]
                    )
                )

                cv2.putText(
                    color,
                    f"ID:{marker_id}",

                    (
                        cx - 25,
                        cy - 10,
                    ),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.8,

                    (
                        0,
                        255,
                        0,
                    ),

                    2,
                )

        return (
            color,
            detected_ids,
        )


def get_target_cfg(
    config,
    marker_id,
):
    target = (
        config.get(
            "targets",
            {},
        )
        .get(
            str(
                marker_id
            )
        )
    )

    if (
        target is None
        or
        not target.get(
            "enabled",
            True,
        )
    ):
        return None

    return target


def build_grasp_cmd(
    config,
    target_cfg,
):
    g = config.get(
        "grasp",
        {},
    )

    grasp_check = target_cfg.get(
        "grasp_check",
        {},
    )

    pull = target_cfg.get(
        "pull",
        {},
    )

    stage1 = pull.get(
        "stage1",
        {},
    )

    stage2 = pull.get(
        "stage2",
        {},
    )

    cmd = [
        sys.executable,
        "auto_grasp_by_joint_replay.py",

        "--slave-can",
        str(
            config["follow"].get(
                "slave_can",
                "can1",
            )
        ),

        "--pose-file",
        str(
            target_cfg.get(
                "pose_file",
                "taught_full_grasp_poses.json",
            )
        ),

        "--pose-name",
        str(
            target_cfg["pose_name"]
        ),

        "--speed",
        str(
            g.get(
                "speed",
                8,
            )
        ),

        "--step-deg",
        str(
            g.get(
                "step_deg",
                1.0,
            )
        ),

        "--tol-deg",
        str(
            g.get(
                "tol_deg",
                0.3,
            )
        ),

        "--rate",
        str(
            g.get(
                "rate",
                20,
            )
        ),

        "--open-mm",
        str(
            target_cfg.get(
                "open_mm",
                60.0,
            )
        ),

        "--close-mm",
        str(
            target_cfg.get(
                "close_mm",
                10.0,
            )
        ),

        "--gripper-effort",
        str(
            target_cfg.get(
                "gripper_effort",
                1000,
            )
        ),

        "--close-wait-s",
        str(
            target_cfg.get(
                "close_wait_s",
                2.0,
            )
        ),

        "--grasp-xyz-tol-mm",
        str(
            grasp_check.get(
                "xyz_tol_mm",
                3.0,
            )
        ),

        "--grasp-rpy-tol-deg",
        str(
            grasp_check.get(
                "rpy_tol_deg",
                2.0,
            )
        ),

        "--stage1-distance-mm",
        str(
            stage1.get(
                "distance_mm",
                120.0,
            )
        ),

        "--stage1-speed",
        str(
            stage1.get(
                "speed",
                4,
            )
        ),

        "--stage1-step-deg",
        str(
            stage1.get(
                "step_deg",
                0.35,
            )
        ),

        "--stage1-final-z-tol-mm",
        str(
            stage1.get(
                "final_z_tol_mm",
                5.0,
            )
        ),

        "--stage1-critical-xy-mm",
        str(
            stage1.get(
                "critical_xy_drift_mm",
                10.0,
            )
        ),

        "--stage1-max-xy-mm",
        str(
            stage1.get(
                "max_xy_drift_mm",
                12.0,
            )
        ),

        "--stage1-max-rpy-deg",
        str(
            stage1.get(
                "max_rpy_drift_deg",
                8.0,
            )
        ),

        "--stage2-distance-mm",
        str(
            stage2.get(
                "distance_mm",
                180.0,
            )
        ),

        "--stage2-speed",
        str(
            stage2.get(
                "speed",
                5,
            )
        ),

        "--stage2-step-deg",
        str(
            stage2.get(
                "step_deg",
                0.5,
            )
        ),

        "--stage2-final-z-tol-mm",
        str(
            stage2.get(
                "final_z_tol_mm",
                8.0,
            )
        ),

        "--stage2-critical-z-mm",
        str(
            stage2.get(
                "critical_z_mm",
                40.0,
            )
        ),

        "--stage2-critical-xy-mm",
        str(
            stage2.get(
                "critical_xy_drift_mm",
                40.0,
            )
        ),

        "--stage2-max-xy-mm",
        str(
            stage2.get(
                "max_xy_drift_mm",
                100.0,
            )
        ),

        "--stage2-max-rpy-deg",
        str(
            stage2.get(
                "max_rpy_drift_deg",
                50.0,
            )
        ),

        "--auto",
    ]

    return cmd


def build_return_cmd(
    config,
    target_cfg,
):
    return_cfg = target_cfg.get(
        "return",
        {},
    )

    pull_cfg = target_cfg.get(
        "pull",
        {},
    )

    stage1_cfg = pull_cfg.get(
        "stage1",
        {},
    )

    pose_file = target_cfg.get(
        "pose_file",
        "taught_full_grasp_poses.json",
    )

    pose_name = target_cfg[
        "pose_name"
    ]

    cmd = [
        sys.executable,
        "auto_return_by_reverse_pull.py",

        "--slave-can",
        str(
            config["follow"].get(
                "slave_can",
                "can1",
            )
        ),

        "--pose-file",
        str(
            pose_file
        ),

        "--pose-name",
        str(
            pose_name
        ),

        "--distance-mm",
        str(
            return_cfg.get(
                "distance_mm",
                stage1_cfg.get(
                    "distance_mm",
                    120.0,
                ),
            )
        ),

        "--approach-speed",
        str(
            return_cfg.get(
                "approach_speed",
                5,
            )
        ),

        "--approach-step-deg",
        str(
            return_cfg.get(
                "approach_step_deg",
                0.7,
            )
        ),

        "--insert-speed",
        str(
            return_cfg.get(
                "insert_speed",
                3,
            )
        ),

        "--insert-step-deg",
        str(
            return_cfg.get(
                "insert_step_deg",
                0.25,
            )
        ),

        "--retract-speed",
        str(
            return_cfg.get(
                "retract_speed",
                4,
            )
        ),

        "--retract-step-deg",
        str(
            return_cfg.get(
                "retract_step_deg",
                0.35,
            )
        ),

        "--tol-deg",
        str(
            return_cfg.get(
                "tol_deg",
                0.3,
            )
        ),

        "--rate",
        str(
            return_cfg.get(
                "rate",
                20,
            )
        ),

        "--open-mm",
        str(
            target_cfg.get(
                "open_mm",
                60.0,
            )
        ),

        "--gripper-effort",
        str(
            target_cfg.get(
                "gripper_effort",
                1000,
            )
        ),

        "--release-wait-s",
        str(
            return_cfg.get(
                "release_wait_s",
                2.0,
            )
        ),

        "--max-start-joint-error-deg",
        str(
            return_cfg.get(
                "max_start_joint_error_deg",
                25.0,
            )
        ),

        "--final-z-tol-mm",
        str(
            return_cfg.get(
                "final_z_tol_mm",
                stage1_cfg.get(
                    "final_z_tol_mm",
                    5.0,
                ),
            )
        ),

        "--critical-xy-mm",
        str(
            return_cfg.get(
                "critical_xy_drift_mm",
                stage1_cfg.get(
                    "critical_xy_drift_mm",
                    10.0,
                ),
            )
        ),

        "--max-xy-mm",
        str(
            return_cfg.get(
                "max_xy_drift_mm",
                stage1_cfg.get(
                    "max_xy_drift_mm",
                    12.0,
                ),
            )
        ),

        "--max-rpy-deg",
        str(
            return_cfg.get(
                "max_rpy_drift_deg",
                stage1_cfg.get(
                    "max_rpy_drift_deg",
                    8.5,
                ),
            )
        ),
    ]

    return cmd



def draw_panel_text(
    img,
    selected_id,
    detected_ids,
    config,
    auto_running,
):
    h, _ = (
        img.shape[:2]
    )

    visible = [
        mid
        for mid
        in detected_ids
        if
        get_target_cfg(
            config,
            mid,
        )
        is not None
    ]

    lines = [
        "Piper Operator Panel",

        (
            "q: exit   "
            "g: auto grasp   "
            "0-9: select ID   "
            "i: input ID"
        ),

        f"Detected IDs: "
        f"{detected_ids}",

        f"Configured visible IDs: "
        f"{visible}",

        f"Selected ID: "
        f"{selected_id}",

        (
            f"Mode: "
            f"{'AUTO GRASP' if auto_running else 'MASTER-SLAVE FOLLOW'}"
        ),
    ]

    # 自动放回快捷键提示
    lines.insert(
        2,
        "r: auto return"
    )


    y = 25

    for line in lines:
        cv2.putText(
            img,
            line,

            (
                15,
                y,
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.55,

            (
                255,
                255,
                255,
            ),

            2,
        )

        y += 24

    if (
        selected_id
        is not None
    ):
        target = (
            get_target_cfg(
                config,
                selected_id,
            )
        )

        if (
            target
            is not None
        ):
            info = (
                f"ID "
                f"{selected_id} | "
                f"{target.get('label', '')} | "
                f"{target.get('pose_name', '')}"
            )

            color = (
                0,
                255,
                0,
            )

        else:
            info = (
                f"ID "
                f"{selected_id} "
                f"has no enabled config"
            )

            color = (
                0,
                0,
                255,
            )

        cv2.putText(
            img,
            info,

            (
                15,
                h - 25,
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.6,

            color,

            2,
        )


def main():
    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(
        "--config",
        default=(
            "task_targets.json"
        ),
    )

    parser.add_argument(
        "--show-follow-log",
        action="store_true",
    )

    args = (
        parser.parse_args()
    )

    config = (
        load_json(
            args.config
        )
    )

    # 每次运行人工选择 ID。
    selected_id = None

    follow = FollowProcess(
        config,

        show_log=(
            args.show_follow_log
        ),
    )

    camera = CameraPanel(
        config
    )

    grasp_proc = None
    auto_kind = None

    print(
        "========== "
        "Piper Operator Panel "
        "=========="
    )

    print(
        "人工主从 "
        "→ 人工选择ID "
        "→ 相机看到该ArUco "
        "→ g "
        "→ 自动抓取 "
        "→ 动态计算小弧线拔出 "
        "→ 回HOLD "
        "→ 恢复主从"
    )

    print(
        "ArUco 只在按 g 启动前"
        "做目标可见性确认；"
        "自动动作开始后"
        "不再依赖 ArUco。"
    )

    print(
        "D435i 相机从程序启动"
        "到退出始终保持开启。"
    )

    print(
        "=========================================="
    )

    try:
        follow.start()

        camera.start()

        while True:
            if (
                grasp_proc
                is not None
                and
                grasp_proc.poll()
                is not None
            ):
                code = (
                    grasp_proc.returncode
                )

                action_name = (
                    "自动放回"
                    if auto_kind == "return"
                    else "自动夹取"
                )

                print(
                    f"\n[AUTO] "
                    f"{action_name}子进程结束，"
                    f"returncode="
                    f"{code}"
                )

                finished_kind = auto_kind

                grasp_proc = None
                auto_kind = None

                if code == 0:
                    print(
                        "[FOLLOW] "
                        "自动动作 PASS，"
                        "已回到主从暂停位置；"
                        "恢复 absolute 主从，"
                        "不二次对齐"
                    )

                    follow.start(
                        do_align=False
                    )

                else:
                    print(
                        "\n[FOLLOW BLOCKED] "
                        f"{finished_kind} "
                        f"执行失败，returncode={code}"
                    )

                    print(
                        "[FOLLOW BLOCKED] "
                        "当前不自动恢复主从。"
                        "请先检查从臂位置和现场状态。"
                    )

            (
                img,
                detected_ids,
            ) = (
                camera
                .read_frame_and_detect()
            )

            if (
                img
                is None
            ):
                continue

            auto_running = (
                grasp_proc
                is not None
                and
                grasp_proc.poll()
                is None
            )

            draw_panel_text(
                img,
                selected_id,
                detected_ids,
                config,
                auto_running,
            )

            cv2.imshow(
                "Piper Operator Panel",
                img,
            )

            key = (
                cv2.waitKey(
                    1
                )
                &
                0xFF
            )

            if (
                key
                ==
                255
            ):
                continue

            if (
                key
                ==
                ord(
                    "q"
                )
                or
                key
                ==
                27
            ):
                print(
                    "\n[USER] "
                    "退出"
                )

                break

            if (
                auto_running
            ):
                if (
                    key
                    ==
                    ord(
                        "g"
                    )
                ):
                    print(
                        "[WARN] "
                        "自动夹取正在执行，"
                        "忽略新的 g 指令。"
                    )

                continue

            if (
                ord(
                    "0"
                )
                <=
                key
                <=
                ord(
                    "9"
                )
            ):
                selected_id = (
                    key
                    -
                    ord(
                        "0"
                    )
                )

                print(
                    f"\n[SELECT] "
                    f"当前选择 "
                    f"ArUco ID="
                    f"{selected_id}"
                )

                continue

            if (
                key
                ==
                ord(
                    "i"
                )
            ):
                try:
                    selected_id = int(
                        input(
                            "\n请输入 "
                            "ArUco ID："
                        ).strip()
                    )

                    print(
                        f"[SELECT] "
                        f"当前选择 "
                        f"ArUco ID="
                        f"{selected_id}"
                    )

                except Exception:
                    print(
                        "[WARN] "
                        "输入无效"
                    )

                continue

            if (
                key
                ==
                ord(
                    "r"
                )
            ):
                # --------------------------------------------
                # 必须人工明确选择 ID
                # --------------------------------------------

                if selected_id is None:
                    print(
                        "\n[RETURN WARN] "
                        "尚未选择 ArUco ID。"
                    )

                    print(
                        "[RETURN WARN] "
                        "请先按数字键选择对应 ID，"
                        "再按 r。"
                    )

                    continue

                target_id = int(
                    selected_id
                )

                # --------------------------------------------
                # r 启动前必须看到对应 ArUco
                # --------------------------------------------

                if (
                    target_id
                    not in
                    detected_ids
                ):
                    print(
                        f"\n[RETURN WARN] "
                        f"当前没有检测到 "
                        f"ID={target_id}，"
                        "不执行自动放回。"
                    )

                    continue

                target_cfg = (
                    get_target_cfg(
                        config,
                        target_id,
                    )
                )

                if target_cfg is None:
                    print(
                        f"\n[RETURN WARN] "
                        f"ID={target_id} "
                        "没有启用的任务配置。"
                    )

                    continue

                return_cfg = (
                    target_cfg.get(
                        "return",
                        {},
                    )
                )

                if not return_cfg.get(
                    "enabled",
                    False,
                ):
                    print(
                        f"\n[RETURN WARN] "
                        f"ID={target_id} "
                        "没有启用 return 配置。"
                    )

                    continue

                mode = return_cfg.get(
                    "mode",
                    "reverse_pull",
                )

                if (
                    mode
                    !=
                    "reverse_pull"
                ):
                    print(
                        f"\n[RETURN WARN] "
                        f"暂不支持 return.mode="
                        f"{mode}"
                    )

                    continue

                print(
                    f"\n[RETURN] "
                    f"ID={target_id} "
                    "已选择且当前可见，"
                    "开始自动放回"
                )

                print(
                    "[RETURN] "
                    "只暂停主从；"
                    "D435i 相机保持开启"
                )

                follow.stop()

                cmd = (
                    build_return_cmd(
                        config,
                        target_cfg,
                    )
                )

                print(
                    "[RETURN CMD]",
                    " ".join(
                        cmd
                    ),
                )

                auto_kind = "return"

                grasp_proc = (
                    subprocess.Popen(
                        cmd,
                        cwd=str(
                            BASE_DIR
                        ),
                    )
                )

                continue


            if (
                key
                ==
                ord(
                    "g"
                )
            ):
                if (
                    selected_id
                    is None
                ):
                    print(
                        "\n[WARN] "
                        "尚未人工选择目标 ID；"
                        "请先按数字键"
                        "或按 i 输入 ID。"
                    )

                    continue

                target_id = int(
                    selected_id
                )

                target_cfg = (
                    get_target_cfg(
                        config,
                        target_id,
                    )
                )

                if (
                    target_cfg
                    is None
                ):
                    print(
                        f"\n[WARN] "
                        f"ID={target_id} "
                        "未配置或已禁用。"
                    )

                    continue

                if (
                    target_id
                    not in detected_ids
                ):
                    print(
                        f"\n[WARN] "
                        "当前画面没有检测到"
                        "人工选择的 "
                        f"ID={target_id}，"
                        "不执行。"
                    )

                    continue

                print(
                    f"\n[AUTO] "
                    f"ID={target_id} "
                    "已选择且当前可见，"
                    "开始自动夹取"
                )

                print(
                    "[AUTO] "
                    "只暂停主从；"
                    "D435i 相机保持开启"
                )

                follow.stop()

                cmd = (
                    build_grasp_cmd(
                        config,
                        target_cfg,
                    )
                )

                print(
                    "[AUTO CMD]",
                    " ".join(
                        cmd
                    ),
                )

                auto_kind = "grasp"

                grasp_proc = (
                    subprocess.Popen(
                        cmd,
                        cwd=str(
                            BASE_DIR
                        ),
                    )
                )

    finally:
        if (
            grasp_proc
            is not None
            and
            grasp_proc.poll()
            is None
        ):
            try:
                grasp_proc.send_signal(
                    signal.SIGINT
                )

                grasp_proc.wait(
                    timeout=3
                )

            except Exception:
                try:
                    grasp_proc.kill()

                except Exception:
                    pass

        try:
            follow.stop()

        except Exception:
            pass

        try:
            camera.stop()

        except Exception:
            pass

        print(
            "\n[DONE] "
            "Piper Operator Panel "
            "已退出"
        )


if __name__ == "__main__":
    main()
