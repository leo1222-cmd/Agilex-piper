import argparse
import json
import re
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from piper_sdk import C_PiperInterface


def create_aruco_detector(dict_name):
    aruco = cv2.aruco
    dict_map = {
        "4x4_50": aruco.DICT_4X4_50,
        "4x4_100": aruco.DICT_4X4_100,
        "4x4_250": aruco.DICT_4X4_250,
        "4x4_1000": aruco.DICT_4X4_1000,
    }

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


def get_camera_matrix(intr):
    camera_matrix = np.array(
        [
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)
    return camera_matrix, dist_coeffs


def estimate_pose(corners, marker_size_m, camera_matrix, dist_coeffs):
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            marker_size_m,
            camera_matrix,
            dist_coeffs,
        )
        return rvecs, tvecs

    s = marker_size_m / 2.0
    obj_points = np.array(
        [
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ],
        dtype=np.float64,
    )

    rvecs = []
    tvecs = []

    for c in corners:
        img_points = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None, None
        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))

    return np.array(rvecs, dtype=np.float64), np.array(tvecs, dtype=np.float64)


def estimate_marker_once(color, camera_matrix, dist_coeffs,
                         dictionary, parameters, detector,
                         marker_size_m, marker_id):
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detect_markers(
        gray,
        dictionary,
        parameters,
        detector,
    )

    if ids is None or len(ids) == 0:
        return None, corners, ids

    rvecs, tvecs = estimate_pose(
        corners,
        marker_size_m,
        camera_matrix,
        dist_coeffs,
    )

    if rvecs is None:
        return None, corners, ids

    for i, mid in enumerate(ids.flatten()):
        if int(mid) != int(marker_id):
            continue

        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        c = corners[i][0]
        center_u = int(np.mean(c[:, 0]))
        center_v = int(np.mean(c[:, 1]))

        return {
            "marker_id": int(mid),
            "marker_camera_rvec": rvec,
            "marker_camera_xyz_m": tvec,
            "center_pixel": [center_u, center_v],
        }, corners, ids

    return None, corners, ids


def parse_joint_raw_from_msg(joint_msg):
    """
    piper_sdk 的 joint_state 字段不同版本可能属性名不同。
    这里直接从字符串里解析 Joint 1:xxxx，最稳。
    """
    text = str(joint_msg)
    vals = re.findall(r"Joint\s*\d+\s*:\s*(-?\d+)", text)

    if len(vals) >= 6:
        return [int(v) for v in vals[:6]]

    raise RuntimeError("无法从 GetArmJointMsgs 输出中解析 6 个关节角，请把输出发给我。")


def read_slave_joint(piper):
    msg = piper.GetArmJointMsgs()
    raw = parse_joint_raw_from_msg(msg)
    deg = [v / 1000.0 for v in raw]
    return raw, deg, str(msg)


def read_slave_endpose(piper):
    msg = piper.GetArmEndPoseMsgs()
    ep = msg.end_pose

    raw = {
        "X_axis": int(ep.X_axis),
        "Y_axis": int(ep.Y_axis),
        "Z_axis": int(ep.Z_axis),
        "RX_axis": int(ep.RX_axis),
        "RY_axis": int(ep.RY_axis),
        "RZ_axis": int(ep.RZ_axis),
    }

    mm_deg = {
        "X_mm": raw["X_axis"] / 1000.0,
        "Y_mm": raw["Y_axis"] / 1000.0,
        "Z_mm": raw["Z_axis"] / 1000.0,
        "RX_deg": raw["RX_axis"] / 1000.0,
        "RY_deg": raw["RY_axis"] / 1000.0,
        "RZ_deg": raw["RZ_axis"] / 1000.0,
    }

    return raw, mm_deg, str(msg)


def load_json(path):
    p = Path(path)
    if not p.exists():
        return {}
    return json.loads(p.read_text())


def save_json(path, data):
    p = Path(path)
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--slave-can", default="can1")
    parser.add_argument("--pose-name", default="teach_reagent_grasp_pose_fixed")
    parser.add_argument("--out", default="taught_full_grasp_poses.json")

    parser.add_argument("--marker-size-m", type=float, default=0.032)
    parser.add_argument("--dict", default="4x4_50",
                        choices=["4x4_50", "4x4_100", "4x4_250", "4x4_1000"])
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--samples", type=int, default=80)

    parser.add_argument("--open-mm", type=float, default=60.0)
    parser.add_argument("--close-mm", type=float, default=10.0)
    parser.add_argument("--gripper-effort", type=int, default=1000)

    args = parser.parse_args()

    print("========== Teach Full Grasp Pose ==========")
    print("slave_can:", args.slave_can)
    print("pose_name:", args.pose_name)
    print("out:", args.out)
    print("marker_size_m:", args.marker_size_m)
    print("marker_id:", args.marker_id)
    print("samples:", args.samples)
    print("===========================================")
    print("说明：")
    print("1. 这个脚本只读取相机、从臂关节角、从臂末端位姿，不主动控制机械臂。")
    print("2. 请先用主从控制把从臂移动到真正适合夹取试剂的位置。")
    print("3. 到位后停止主从 follow，再运行本脚本。")
    print("4. 看到 ArUco 稳定后，按 a 保存完整示教位。")
    print("5. 按 q 退出。")
    print("===========================================")

    piper = C_PiperInterface(args.slave_can)
    piper.ConnectPort()
    time.sleep(1.0)

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    camera_matrix, dist_coeffs = get_camera_matrix(intr)

    dictionary, parameters, detector = create_aruco_detector(args.dict)

    print("\n[INFO] 相机已启动。看到 ArUco 后按 a 保存，按 q 退出。")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            show = color.copy()

            result, corners, ids = estimate_marker_once(
                color=color,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                dictionary=dictionary,
                parameters=parameters,
                detector=detector,
                marker_size_m=args.marker_size_m,
                marker_id=args.marker_id,
            )

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(show, corners, ids)

            if result is not None:
                rvec = np.array(result["marker_camera_rvec"], dtype=np.float64)
                tvec = np.array(result["marker_camera_xyz_m"], dtype=np.float64)

                try:
                    cv2.drawFrameAxes(
                        show,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        args.marker_size_m * 0.7,
                    )
                except Exception:
                    pass

                xyz_mm = tvec * 1000.0
                cv2.putText(
                    show,
                    "marker_xyz_mm=(%.1f, %.1f, %.1f)" % tuple(xyz_mm),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                )
                cv2.putText(
                    show,
                    "Press a to save full grasp pose",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    show,
                    "Target ArUco not detected",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow("teach_grasp_full_pose", show)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            if key == ord("a"):
                print("\n[INFO] 开始采样 ArUco 位姿...")

                xyz_list = []
                rvec_list = []
                center_list = []

                for i in range(args.samples):
                    frames = pipeline.wait_for_frames()
                    frames = align.process(frames)
                    color_frame = frames.get_color_frame()

                    if not color_frame:
                        continue

                    color2 = np.asanyarray(color_frame.get_data())

                    res, _, _ = estimate_marker_once(
                        color=color2,
                        camera_matrix=camera_matrix,
                        dist_coeffs=dist_coeffs,
                        dictionary=dictionary,
                        parameters=parameters,
                        detector=detector,
                        marker_size_m=args.marker_size_m,
                        marker_id=args.marker_id,
                    )

                    if res is not None:
                        xyz_list.append(res["marker_camera_xyz_m"])
                        rvec_list.append(res["marker_camera_rvec"])
                        center_list.append(res["center_pixel"])

                    time.sleep(0.01)

                if len(xyz_list) == 0:
                    print("[ERROR] 采样失败，没有检测到目标 ArUco。")
                    continue

                xyz_arr = np.array(xyz_list, dtype=np.float64)
                rvec_arr = np.array(rvec_list, dtype=np.float64)
                center_arr = np.array(center_list, dtype=np.float64)

                marker_xyz = np.median(xyz_arr, axis=0)
                marker_rvec = np.median(rvec_arr, axis=0)
                center_pixel = np.median(center_arr, axis=0).astype(int).tolist()

                std_xyz = np.std(xyz_arr, axis=0)
                std_rvec = np.std(rvec_arr, axis=0)

                print("[INFO] 读取从臂关节角和末端位姿...")
                slave_joint_raw, slave_joint_deg, slave_joint_text = read_slave_joint(piper)
                slave_endpose_raw, slave_endpose_mm_deg, slave_endpose_text = read_slave_endpose(piper)

                data = load_json(args.out)

                record = {
                    "pose_name": args.pose_name,
                    "timestamp": time.time(),

                    "dict": args.dict,
                    "marker_id": args.marker_id,
                    "marker_size_m": args.marker_size_m,

                    "marker_camera_xyz_m": marker_xyz.tolist(),
                    "marker_camera_xyz_mm": (marker_xyz * 1000.0).tolist(),
                    "marker_camera_rvec": marker_rvec.tolist(),
                    "center_pixel": center_pixel,

                    "std_marker_xyz_m": std_xyz.tolist(),
                    "std_marker_xyz_mm": (std_xyz * 1000.0).tolist(),
                    "std_marker_rvec": std_rvec.tolist(),
                    "valid_samples": len(xyz_list),
                    "requested_samples": args.samples,

                    "slave_joint_raw": slave_joint_raw,
                    "slave_joint_deg": slave_joint_deg,
                    "slave_joint_text": slave_joint_text,

                    "slave_endpose_raw": slave_endpose_raw,
                    "slave_endpose_mm_deg": slave_endpose_mm_deg,
                    "slave_endpose_text": slave_endpose_text,

                    "gripper": {
                        "open_mm": args.open_mm,
                        "close_mm": args.close_mm,
                        "effort": args.gripper_effort,
                    },

                    "camera_intrinsics": {
                        "width": intr.width,
                        "height": intr.height,
                        "fx": intr.fx,
                        "fy": intr.fy,
                        "ppx": intr.ppx,
                        "ppy": intr.ppy,
                        "coeffs": list(intr.coeffs),
                    },
                }

                data[args.pose_name] = record
                save_json(args.out, data)

                print("\n[SAVED]", args.out)
                print("pose_name:", args.pose_name)
                print("marker_camera_xyz_mm:", record["marker_camera_xyz_mm"])
                print("std_marker_xyz_mm:", record["std_marker_xyz_mm"])
                print("slave_joint_deg:", record["slave_joint_deg"])
                print("slave_endpose_mm_deg:", record["slave_endpose_mm_deg"])
                print("valid_samples:", record["valid_samples"])

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
