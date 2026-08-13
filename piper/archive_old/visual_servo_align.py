import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from piper_sdk import C_PiperInterface


@dataclass
class EndPose:
    x: int
    y: int
    z: int
    rx: int
    ry: int
    rz: int

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
    # CAN_CTRL + MOVE_P
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


def collect_marker_xyz(pipeline, align, camera_matrix, dist_coeffs,
                       dictionary, parameters, detector,
                       marker_size_m, marker_id,
                       samples=8, show_window=True):
    xyz_list = []
    last_show = None
    last_corners = None
    last_ids = None
    last_result = None

    for _ in range(samples):
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color = np.asanyarray(color_frame.get_data())
        result, corners, ids = estimate_marker_once(
            color=color,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=dictionary,
            parameters=parameters,
            detector=detector,
            marker_size_m=marker_size_m,
            marker_id=marker_id,
        )

        last_show = color.copy()
        last_corners = corners
        last_ids = ids
        last_result = result

        if result is not None:
            xyz_list.append(result["marker_camera_xyz_m"])

        time.sleep(0.02)

    if len(xyz_list) == 0:
        return None, None, last_show, last_corners, last_ids, last_result

    xyz = np.median(np.array(xyz_list, dtype=np.float64), axis=0)
    std = np.std(np.array(xyz_list, dtype=np.float64), axis=0)

    return xyz, std, last_show, last_corners, last_ids, last_result


def clip(v, max_abs):
    return float(np.clip(v, -max_abs, max_abs))


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--slave-can", default="can1")
    parser.add_argument("--speed", type=int, default=5)

    parser.add_argument("--pose-file", default="taught_marker_poses.json")
    parser.add_argument("--pose-name", default="teach_reagent_grasp_pose")

    parser.add_argument("--marker-size-m", type=float, default=0.032)
    parser.add_argument("--dict", default="4x4_50",
                        choices=["4x4_50", "4x4_100", "4x4_250", "4x4_1000"])
    parser.add_argument("--marker-id", type=int, default=0)

    parser.add_argument("--gain", type=float, default=0.3)
    parser.add_argument("--max-step-mm", type=float, default=1.0)
    parser.add_argument("--max-total-mm", type=float, default=30.0)

    parser.add_argument("--xy-tol-mm", type=float, default=3.0)
    parser.add_argument("--z-tol-mm", type=float, default=5.0)

    parser.add_argument("--max-iters", type=int, default=30)
    parser.add_argument("--sample-frames", type=int, default=8)
    parser.add_argument("--wait-s", type=float, default=1.0)

    # 符号映射：
    # 你已经验证：error_x > 0，往坐标轴负方向动，所以 x_sign 默认 -1
    # 你已经验证：error_y > 0，往坐标轴负方向动，所以 y_sign 默认 -1
    # z_sign 需要根据实际情况确认：
    # 如果 EndPose Z+ 是靠近 ArUco，则 z_sign=+1；
    # 如果 EndPose Z- 是靠近 ArUco，则 z_sign=-1。
    parser.add_argument("--x-sign", type=float, default=-1.0)
    parser.add_argument("--y-sign", type=float, default=-1.0)
    parser.add_argument("--z-sign", type=float, default=1.0)

    parser.add_argument("--auto", action="store_true",
                        help="不按 Enter，自动执行每一步。首次测试不要加。")

    args = parser.parse_args()

    pose_data = json.loads(Path(args.pose_file).read_text())
    if args.pose_name not in pose_data:
        raise RuntimeError(f"cannot find pose_name={args.pose_name} in {args.pose_file}")

    desired_xyz = np.array(
        pose_data[args.pose_name]["marker_camera_xyz_m"],
        dtype=np.float64,
    )
    desired_xyz_mm = desired_xyz * 1000.0

    print("========== Visual Servo Align ==========")
    print("slave_can:", args.slave_can)
    print("pose_file:", args.pose_file)
    print("pose_name:", args.pose_name)
    print("desired_xyz_mm:", desired_xyz_mm.tolist())
    print("gain:", args.gain)
    print("max_step_mm:", args.max_step_mm)
    print("max_total_mm:", args.max_total_mm)
    print("tol: xy =", args.xy_tol_mm, "mm, z =", args.z_tol_mm, "mm")
    print("sign: x =", args.x_sign, " y =", args.y_sign, " z =", args.z_sign)
    print("auto:", args.auto)
    print("========================================")
    print("注意：")
    print("1. 不要同时运行主从 follow。")
    print("2. 首次测试不要加 --auto，每一步确认后再动。")
    print("3. 如果发现 error 变大，立即 Ctrl+C 停止。")
    print("4. 如果 Z 方向越调越远，下一次把 --z-sign 改成 -1。")
    print("========================================")

    input("确认从臂、夹爪、相机线、试剂周围安全，按 Enter 继续...")

    piper = connect_piper(args.slave_can)

    print("\n[INFO] 切 CAN_CTRL + MOVE_P，并使能从臂")
    set_can_p(piper, speed=args.speed)
    enable_arm(piper)

    start_pose = get_end_pose(piper)
    print("[START_END_POSE]", start_pose.readable())

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

    total_move = np.zeros(3, dtype=np.float64)

    try:
        for it in range(1, args.max_iters + 1):
            current_xyz, std_xyz, show, corners, ids, result = collect_marker_xyz(
                pipeline=pipeline,
                align=align,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                dictionary=dictionary,
                parameters=parameters,
                detector=detector,
                marker_size_m=args.marker_size_m,
                marker_id=args.marker_id,
                samples=args.sample_frames,
                show_window=True,
            )

            if show is not None:
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

            if current_xyz is None:
                print(f"[{it:02d}] ArUco not detected, stop.")
                if show is not None:
                    cv2.imshow("visual_servo_align", show)
                    cv2.waitKey(1)
                break

            current_xyz_mm = current_xyz * 1000.0
            error_mm = current_xyz_mm - desired_xyz_mm

            aligned = (
                abs(error_mm[0]) <= args.xy_tol_mm
                and abs(error_mm[1]) <= args.xy_tol_mm
                and abs(error_mm[2]) <= args.z_tol_mm
            )

            print("\n----------------------------------------")
            print(f"[ITER {it:02d}]")
            print(
                "current_xyz_mm = "
                f"({current_xyz_mm[0]:.2f}, {current_xyz_mm[1]:.2f}, {current_xyz_mm[2]:.2f})"
            )
            print(
                "desired_xyz_mm = "
                f"({desired_xyz_mm[0]:.2f}, {desired_xyz_mm[1]:.2f}, {desired_xyz_mm[2]:.2f})"
            )
            print(
                "error_xyz_mm   = "
                f"({error_mm[0]:.2f}, {error_mm[1]:.2f}, {error_mm[2]:.2f})"
            )
            if std_xyz is not None:
                print(
                    "std_xyz_mm     = "
                    f"({std_xyz[0]*1000:.2f}, {std_xyz[1]*1000:.2f}, {std_xyz[2]*1000:.2f})"
                )

            if show is not None:
                cv2.putText(
                    show,
                    "error_mm=(%.1f, %.1f, %.1f)" % tuple(error_mm),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                )
                cv2.putText(
                    show,
                    "ALIGNED" if aligned else "MOVING",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 0) if aligned else (0, 255, 255),
                    2,
                )
                cv2.imshow("visual_servo_align", show)
                cv2.waitKey(1)

            if aligned:
                print("[ALIGNED] 已进入误差阈值，停止对准。")
                break

            move_mm = np.array(
                [
                    args.x_sign * args.gain * error_mm[0],
                    args.y_sign * args.gain * error_mm[1],
                    args.z_sign * args.gain * error_mm[2],
                ],
                dtype=np.float64,
            )

            move_mm[0] = clip(move_mm[0], args.max_step_mm)
            move_mm[1] = clip(move_mm[1], args.max_step_mm)
            move_mm[2] = clip(move_mm[2], args.max_step_mm)

            total_move += move_mm
            total_norm = float(np.linalg.norm(total_move))

            print(
                "move_cmd_mm     = "
                f"({move_mm[0]:.2f}, {move_mm[1]:.2f}, {move_mm[2]:.2f})"
            )
            print(f"total_move_norm = {total_norm:.2f} mm")

            if total_norm > args.max_total_mm:
                print("[STOP] 累计移动超过 max_total_mm，停止，防止跑飞。")
                break

            now_pose = get_end_pose(piper)
            target_pose = EndPose(
                x=now_pose.x + int(move_mm[0] * 1000),
                y=now_pose.y + int(move_mm[1] * 1000),
                z=now_pose.z + int(move_mm[2] * 1000),
                rx=now_pose.rx,
                ry=now_pose.ry,
                rz=now_pose.rz,
            )

            print("[NOW_POSE]   ", now_pose.readable())
            print("[TARGET_POSE]", target_pose.readable())

            if not args.auto:
                input("确认执行这一步小运动，按 Enter；如果不安全按 Ctrl+C...")

            send_end_pose(piper, target_pose)
            time.sleep(args.wait_s)

        print("\n[DONE] visual_servo_align finished.")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
