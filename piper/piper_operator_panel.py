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


BASE_DIR = Path(__file__).resolve().parent


def load_json(path):
    p = Path(path)
    if not p.is_absolute():
        p = BASE_DIR / p
    return json.loads(p.read_text())


def create_aruco_detector(dict_name):
    aruco = cv2.aruco

    dict_map = {
        "4x4_50": aruco.DICT_4X4_50,
        "4x4_100": aruco.DICT_4X4_100,
        "4x4_250": aruco.DICT_4X4_250,
        "4x4_1000": aruco.DICT_4X4_1000,
    }

    if dict_name not in dict_map:
        raise RuntimeError(f"不支持的 ArUco 字典: {dict_name}")

    dictionary = aruco.getPredefinedDictionary(dict_map[dict_name])

    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()

    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, parameters)
    else:
        detector = None

    return dictionary, parameters, detector


def detect_markers(gray, dictionary, parameters, detector):
    if detector is not None:
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


class FollowProcess:
    def __init__(self, config, show_log=False):
        self.config = config
        self.proc = None
        self.show_log = show_log
        self.log_file = None

    def run_align(self):
        align_cfg = self.config.get("align", {})
        if not align_cfg.get("enabled", True):
            print("[ALIGN] 配置中已关闭自动对齐，跳过 align")
            return

        f = self.config["follow"]

        cmd = [
            sys.executable,
            "ms.py",
            "align-slave-to-master",
            "--master-can", str(f.get("master_can", "can0")),
            "--slave-can", str(f.get("slave_can", "can1")),
            "--speed", str(align_cfg.get("speed", 8)),
            "--rate", str(align_cfg.get("rate", 20)),
            "--step-deg", str(align_cfg.get("step_deg", 0.8)),
            "--tol-deg", str(align_cfg.get("tol_deg", 0.3)),
        ]

        print("\n[ALIGN] 绝对位置主从控制前，先执行从臂对齐主动臂")
        print("[ALIGN CMD]", " ".join(cmd))

        input("[ALIGN] 请确认主臂、从臂、夹爪、相机线周围安全，按 Enter 开始从臂对齐主动臂...")
        result = subprocess.run(cmd, cwd=str(BASE_DIR))

        if result.returncode != 0:
            print("[ALIGN WARN] 对齐命令返回非零状态码:", result.returncode)
        else:
            print("[ALIGN] 对齐完成，准备启动主从 follow")

        time.sleep(0.5)

    def build_cmd(self):
        f = self.config["follow"]

        cmd = [
            sys.executable,
            "ms.py",
            "follow",
            "--master-can", str(f.get("master_can", "can0")),
            "--slave-can", str(f.get("slave_can", "can1")),
            "--follow-mode", str(f.get("follow_mode", "absolute")),
            "--speed", str(f.get("speed", 12)),
            "--rate", str(f.get("rate", 25)),
            "--alpha", str(f.get("alpha", 0.30)),
            "--max-step-deg", str(f.get("max_step_deg", 2.0)),
            "--cmd-deadband-deg", str(f.get("cmd_deadband_deg", 0.08)),
            "--gripper-source", str(f.get("gripper_source", "fb")),
            "--gripper-min", str(f.get("gripper_min", 10000)),
            "--gripper-max", str(f.get("gripper_max", 60000)),
            "--gripper-scale", str(f.get("gripper_scale", 1.235)),
            "--gripper-offset", str(f.get("gripper_offset", -12716)),
            "--gripper-effort", str(f.get("gripper_effort", 1000)),
            "--mirror", str(f.get("mirror", "1,1,1,1,1,1")),
            "--joint-offset-deg", str(f.get("joint_offset_deg", "0,0,0,0,0,0")),
        ]

        if f.get("sync_gripper", True):
            cmd.append("--sync-gripper")

        return cmd

    def start(self, do_align=True):
        if self.proc is not None and self.proc.poll() is None:
            return

        if do_align:
            self.run_align()
        else:
            print("\n[ALIGN] 恢复主从时跳过二次对齐，直接启动 follow")
            print("[ALIGN] 注意：自动夹取过程中请不要移动主动臂。")

        cmd = self.build_cmd()

        if self.show_log:
            stdout = None
            stderr = None
        else:
            log_path = BASE_DIR / "operator_follow.log"
            self.log_file = open(log_path, "a", buffering=1)
            self.log_file.write("\n\n========== START FOLLOW ==========\n")
            self.log_file.write(" ".join(cmd) + "\n")
            stdout = self.log_file
            stderr = self.log_file

        print("\n[FOLLOW] 启动主从控制")
        print("[FOLLOW] 如果机械臂没有立即跟随，请在当前终端按 Enter 开始主从跟随。")
        self.proc = subprocess.Popen(
            cmd,
            cwd=str(BASE_DIR),
            stdout=stdout,
            stderr=stderr,
            preexec_fn=os.setsid,
        )
        time.sleep(1.0)

    def stop(self):
        if self.proc is None:
            return

        if self.proc.poll() is not None:
            self.proc = None
            return

        print("\n[FOLLOW] 停止主从控制")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except Exception:
                    pass

        self.proc = None

        if self.log_file is not None:
            try:
                self.log_file.flush()
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

        time.sleep(0.5)


class CameraPanel:
    def __init__(self, config):
        self.config = config
        self.pipeline = None
        self.align = None
        self.dictionary = None
        self.parameters = None
        self.detector = None

    def start(self):
        cam_cfg = self.config.get("camera", {})
        dict_name = cam_cfg.get("dict", "4x4_50")

        self.dictionary, self.parameters, self.detector = create_aruco_detector(dict_name)

        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(rs_config)
        self.align = rs.align(rs.stream.color)

        print("[CAMERA] 相机实时显示已开启")

    def stop(self):
        print("[CAMERA] 关闭相机显示")
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass
        self.pipeline = None
        cv2.destroyAllWindows()
        time.sleep(0.3)

    def read_frame_and_detect(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            return None, []

        color = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detect_markers(
            gray,
            self.dictionary,
            self.parameters,
            self.detector,
        )

        detected_ids = []

        if ids is not None and len(ids) > 0:
            detected_ids = [int(x) for x in ids.flatten().tolist()]
            cv2.aruco.drawDetectedMarkers(color, corners, ids)

            for i, marker_id in enumerate(detected_ids):
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.putText(
                    color,
                    f"ID:{marker_id}",
                    (cx - 25, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

        return color, detected_ids


def get_target_cfg(config, marker_id):
    targets = config.get("targets", {})
    key = str(marker_id)

    if key not in targets:
        return None

    t = targets[key]
    if not t.get("enabled", True):
        return None

    return t


def build_grasp_cmd(config, target_id, target_cfg):
    grasp_cfg = config.get("grasp", {})

    pose_file = target_cfg.get("pose_file", "taught_full_grasp_poses.json")
    pose_name = target_cfg["pose_name"]

    cmd = [
        sys.executable,
        "auto_grasp_by_joint_replay.py",

        "--slave-can", str(config["follow"].get("slave_can", "can1")),
        "--pose-file", str(pose_file),
        "--pose-name", str(pose_name),

        "--speed", str(grasp_cfg.get("speed", 8)),
        "--step-deg", str(grasp_cfg.get("step_deg", 1.0)),
        "--tol-deg", str(grasp_cfg.get("tol_deg", 0.3)),
        "--rate", str(grasp_cfg.get("rate", 20)),

        "--xy-tol-mm", str(target_cfg.get("xy_tol_mm", 5)),
        "--z-tol-mm", str(target_cfg.get("z_tol_mm", 8)),
        "--verify-samples", str(target_cfg.get("verify_samples", 60)),

        "--open-mm", str(target_cfg.get("open_mm", 60)),
        "--close-mm", str(target_cfg.get("close_mm", 10)),
        "--gripper-effort", str(target_cfg.get("gripper_effort", 1000)),

        "--auto",
    ]

    post_waypoints = target_cfg.get("post_waypoints", [])
    if isinstance(post_waypoints, list) and post_waypoints:
        cmd += ["--post-waypoints", ",".join(post_waypoints)]
    elif isinstance(post_waypoints, str) and post_waypoints.strip():
        cmd += ["--post-waypoints", post_waypoints.strip()]

    return cmd


def run_grasp(config, target_id, target_cfg):
    label = target_cfg.get("label", f"id_{target_id}")

    print("\n========== AUTO GRASP ==========")
    print("target_id:", target_id)
    print("label:", label)
    print("pose_file:", target_cfg.get("pose_file"))
    print("pose_name:", target_cfg.get("pose_name"))
    print("================================")

    cmd = build_grasp_cmd(config, target_id, target_cfg)

    print("[AUTO] 开始执行自动夹取流程")
    result = subprocess.run(cmd, cwd=str(BASE_DIR))

    if result.returncode == 0:
        print("[AUTO] 自动夹取脚本执行完成")
    else:
        print("[AUTO WARN] 自动夹取脚本返回非零状态码:", result.returncode)

    time.sleep(1.0)


def draw_panel_text(img, selected_id, detected_ids, config):
    h, w = img.shape[:2]

    targets = config.get("targets", {})

    visible_configured = []
    for mid in detected_ids:
        if str(mid) in targets and targets[str(mid)].get("enabled", True):
            visible_configured.append(mid)

    lines = [
        "Piper Operator Panel",
        "q: exit   g: grasp selected   0-9: select ID   i: input ID",
        f"Detected IDs: {detected_ids}",
        f"Configured visible IDs: {visible_configured}",
        f"Selected ID: {selected_id}",
    ]

    y = 25
    for line in lines:
        cv2.putText(
            img,
            line,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        y += 24

    if selected_id is not None:
        t = get_target_cfg(config, selected_id)
        if t is not None:
            label = t.get("label", f"id_{selected_id}")
            pose_name = t.get("pose_name", "")
            info = f"Selected target: ID {selected_id} | {label} | {pose_name}"
            color = (0, 255, 0)
        else:
            info = f"Selected ID {selected_id} has no enabled config"
            color = (0, 0, 255)

        cv2.putText(
            img,
            info,
            (15, h - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )


def choose_auto_selected(config, detected_ids, selected_id):
    targets = config.get("targets", {})

    configured_visible = []
    for mid in detected_ids:
        if str(mid) in targets and targets[str(mid)].get("enabled", True):
            configured_visible.append(int(mid))

    # 如果用户选中的 ID 当前可见，并且有配置，就优先执行它
    if selected_id is not None:
        if int(selected_id) in configured_visible:
            return int(selected_id)

    # 如果画面中只有一个已配置的 ArUco，就自动选择它
    if len(configured_visible) == 1:
        return configured_visible[0]

    # 如果默认 ID 可见，就选择默认 ID
    default_id = config.get("default_target_id", None)
    if default_id is not None:
        try:
            default_id = int(default_id)
            if default_id in configured_visible:
                return default_id
        except Exception:
            pass

    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="task_targets.json")
    parser.add_argument("--show-follow-log", action="store_true")
    args = parser.parse_args()

    config = load_json(args.config)

    print("========== Piper Operator Panel ==========")
    print("config:", args.config)
    print("说明：")
    print("1. 脚本启动后会自动开启主从 follow 和相机实时显示。")
    print("2. 画面中可以看到识别到的 ArUco ID。")
    print("3. 按数字键 0-9 选择目标 ID。")
    print("4. 按 g 后：停止主从和相机 → 自动夹取 → 回暂停位 → 重新开启主从和相机。")
    print("5. 按 q 退出，主从和相机会一起关闭。")
    print("==========================================")

    selected_id = config.get("default_target_id", None)
    if selected_id is not None:
        selected_id = int(selected_id)

    follow = FollowProcess(config, show_log=args.show_follow_log)
    camera = CameraPanel(config)

    try:
        follow.start()
        camera.start()

        while True:
            img, detected_ids = camera.read_frame_and_detect()
            if img is None:
                continue

            draw_panel_text(img, selected_id, detected_ids, config)

            cv2.imshow("Piper Operator Panel", img)
            key = cv2.waitKey(1) & 0xFF

            if key == 255:
                continue

            if key == ord("q") or key == 27:
                print("\n[USER] 退出")
                break

            if ord("0") <= key <= ord("9"):
                selected_id = key - ord("0")
                print(f"\n[SELECT] 当前选择 ArUco ID = {selected_id}")
                continue

            if key == ord("i"):
                try:
                    text = input("\n请输入要选择的 ArUco ID：").strip()
                    selected_id = int(text)
                    print(f"[SELECT] 当前选择 ArUco ID = {selected_id}")
                except Exception:
                    print("[WARN] 输入无效")
                continue

            if key == ord("g"):
                target_id = choose_auto_selected(config, detected_ids, selected_id)

                if target_id is None:
                    print("\n[WARN] 未选择可执行的目标 ID。")
                    print("如果画面中有多个 ArUco，请先按数字键选择对应 ID。")
                    continue

                if target_id not in detected_ids:
                    print(f"\n[WARN] 当前画面没有检测到选中的 ID={target_id}，不执行夹取。")
                    continue

                target_cfg = get_target_cfg(config, target_id)
                if target_cfg is None:
                    print(f"\n[WARN] ID={target_id} 没有配置目标动作，不执行夹取。")
                    continue

                print(f"\n[USER] 准备执行 ID={target_id} 的自动夹取")
                print("[STEP] 停止主从 follow 和相机显示")
                follow.stop()
                camera.stop()

                run_grasp(config, target_id, target_cfg)

                print("[STEP] 自动夹取流程结束，重新开启主从 follow 和相机显示")
                follow.start(do_align=False)
                camera.start()

    finally:
        try:
            camera.stop()
        except Exception:
            pass

        try:
            follow.stop()
        except Exception:
            pass

        print("\n[DONE] Piper Operator Panel 已退出")


if __name__ == "__main__":
    main()
