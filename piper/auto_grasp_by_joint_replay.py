import argparse
import json
import re
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from piper_sdk import C_PiperInterface


def parse_joint_raw_from_msg(joint_msg):
    text = str(joint_msg)
    vals = re.findall(r"Joint\s*\d+\s*:\s*(-?\d+)", text)
    if len(vals) >= 6:
        return [int(v) for v in vals[:6]]
    raise RuntimeError("无法解析 6 个关节角，请把 GetArmJointMsgs 输出发给我。")


def read_joint_raw(piper):
    msg = piper.GetArmJointMsgs()
    return parse_joint_raw_from_msg(msg)


def joint_deg(joint_raw):
    return [v / 1000.0 for v in joint_raw]


def joint_readable(joint_raw):
    d = joint_deg(joint_raw)
    return (
        f"J1={d[0]:.3f}° | J2={d[1]:.3f}° | J3={d[2]:.3f}° | "
        f"J4={d[3]:.3f}° | J5={d[4]:.3f}° | J6={d[5]:.3f}°"
    )


def max_joint_error_deg(current, target):
    return max(abs(c - t) for c, t in zip(current, target)) / 1000.0


def set_can_j(piper, speed=8):
    # CAN_CTRL + MOVE_J
    for _ in range(5):
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        time.sleep(0.05)


def enable_arm(piper, repeat=10):
    for _ in range(repeat):
        try:
            piper.EnableArm(7)
        except Exception:
            pass
        time.sleep(0.05)


def send_joint(piper, joint_raw):
    piper.JointCtrl(*[int(v) for v in joint_raw])


def step_towards(current, target, step_raw):
    out = []
    for c, t in zip(current, target):
        diff = t - c
        if diff > step_raw:
            out.append(c + step_raw)
        elif diff < -step_raw:
            out.append(c - step_raw)
        else:
            out.append(t)
    return out


def move_joint_slowly(
    piper,
    target_raw,
    speed=8,
    step_deg=1.0,
    tol_deg=0.3,
    rate=20,
    max_iters=1000,
    name="MOVE",
    soft_tol_deg=0.6,
    stall_patience=40,
    stall_delta_deg=0.02,
):
    """
    关节角低速复现。
    tol_deg: 严格到位阈值。
    soft_tol_deg: 软到位阈值。若误差已经很小但机械臂不再继续收敛，则认为可接受。
    stall_patience: 连续多少轮没有明显变好后判定卡滞。
    stall_delta_deg: 误差改善小于该值时认为没有明显进步。
    """
    step_raw = int(step_deg * 1000)
    dt = 1.0 / float(rate)

    set_can_j(piper, speed=speed)
    enable_arm(piper)

    time.sleep(0.3)

    current = read_joint_raw(piper)

    print(f"\n========== {name} ==========")
    print("[CURRENT]", joint_readable(current))
    print("[TARGET ]", joint_readable(target_raw))
    print("max_error_deg:", max_joint_error_deg(current, target_raw))
    print("step_deg:", step_deg)
    print("tol_deg:", tol_deg)
    print("soft_tol_deg:", soft_tol_deg)
    print("stall_patience:", stall_patience)
    print("==================================")

    best_err = 999999.0
    stagnant_count = 0

    for i in range(max_iters):
        current = read_joint_raw(piper)
        err = max_joint_error_deg(current, target_raw)

        if i % 10 == 0:
            print(f"[{name} {i:04d}] err={err:.3f} deg | {joint_readable(current)}")

        # 严格到位
        if err <= tol_deg:
            print(f"[REACHED] {name} err={err:.3f} deg")
            return True

        # 判断是否还在明显变好
        if err < best_err - stall_delta_deg:
            best_err = err
            stagnant_count = 0
        else:
            stagnant_count += 1

        # 如果已经接近目标，但误差不再下降，认为软到位
        if stagnant_count >= stall_patience and err <= soft_tol_deg:
            print(f"[SOFT_REACHED] {name} err={err:.3f} deg <= soft_tol_deg={soft_tol_deg:.3f} deg")
            print(f"[SOFT_REACHED] 连续 {stall_patience} 轮误差没有明显下降，停止继续发指令。")
            return True

        # 如果误差还比较大，但长时间不下降，认为卡滞失败
        if stagnant_count >= stall_patience and err > soft_tol_deg:
            print(f"[STALLED] {name} err={err:.3f} deg > soft_tol_deg={soft_tol_deg:.3f} deg")
            print(f"[STALLED] 连续 {stall_patience} 轮误差没有明显下降，停止继续发指令。")
            current = read_joint_raw(piper)
            print("[FINAL]", joint_readable(current))
            print("final_error_deg:", max_joint_error_deg(current, target_raw))
            return False

        cmd = step_towards(current, target_raw, step_raw)
        send_joint(piper, cmd)
        time.sleep(dt)

    print(f"[WARN] {name} reached max_iters but not within tolerance.")
    current = read_joint_raw(piper)
    final_err = max_joint_error_deg(current, target_raw)
    print("[FINAL]", joint_readable(current))
    print("final_error_deg:", final_err)

    if final_err <= soft_tol_deg:
        print(f"[SOFT_REACHED] {name} final_err={final_err:.3f} deg <= soft_tol_deg={soft_tol_deg:.3f} deg")
        return True

    return False


def send_gripper(piper, angle_raw, effort=1000):
    piper.GripperCtrl(int(angle_raw), int(effort), 0x01, 0x00)


def read_endpose_raw(piper):
    msg = piper.GetArmEndPoseMsgs()
    ep = msg.end_pose
    return [
        int(ep.X_axis),
        int(ep.Y_axis),
        int(ep.Z_axis),
        int(ep.RX_axis),
        int(ep.RY_axis),
        int(ep.RZ_axis),
    ]


def endpose_readable(pose_raw):
    return (
        f"X={pose_raw[0]/1000.0:.3f} mm | "
        f"Y={pose_raw[1]/1000.0:.3f} mm | "
        f"Z={pose_raw[2]/1000.0:.3f} mm | "
        f"RX={pose_raw[3]/1000.0:.3f} deg | "
        f"RY={pose_raw[4]/1000.0:.3f} deg | "
        f"RZ={pose_raw[5]/1000.0:.3f} deg"
    )


def set_can_p(piper, speed=5):
    # CAN_CTRL + MOVE_P
    for _ in range(5):
        piper.MotionCtrl_2(0x01, 0x00, speed, 0x00)
        time.sleep(0.05)


def send_endpose(piper, pose_raw):
    piper.EndPoseCtrl(*[int(v) for v in pose_raw])


def move_lift_z_slowly(
    piper,
    lift_mm=200.0,
    speed=5,
    step_mm=5.0,
    tol_mm=3.0,
    rate=20,
    max_iters=1000,
):
    """
    夹取后沿机器人基坐标系 Z+ 方向抬升。
    如果机器人底座水平安装，Z+ 通常可近似理解为垂直桌面向上。
    """
    if lift_mm <= 0:
        print("[LIFT] lift_mm <= 0，跳过抬升。")
        return True

    set_can_p(piper, speed=speed)
    enable_arm(piper)

    time.sleep(0.3)

    start_pose = read_endpose_raw(piper)
    target_pose = start_pose.copy()
    target_pose[2] = start_pose[2] + int(lift_mm * 1000)

    step_raw = int(abs(step_mm) * 1000)
    dt = 1.0 / float(rate)

    print("\n========== LIFT_Z ==========")
    print("[START ]", endpose_readable(start_pose))
    print("[TARGET]", endpose_readable(target_pose))
    print("lift_mm:", lift_mm)
    print("step_mm:", step_mm)
    print("tol_mm:", tol_mm)
    print("============================")

    current_z = start_pose[2]
    target_z = target_pose[2]

    direction = 1 if target_z >= current_z else -1

    for i in range(max_iters):
        current = read_endpose_raw(piper)
        err_mm = abs(target_z - current[2]) / 1000.0

        if i % 10 == 0:
            print(f"[LIFT {i:04d}] z_err={err_mm:.2f} mm | {endpose_readable(current)}")

        if err_mm <= tol_mm:
            print(f"[REACHED] LIFT_Z z_err={err_mm:.2f} mm")
            return True

        next_z = current[2] + direction * step_raw

        if direction > 0:
            next_z = min(next_z, target_z)
        else:
            next_z = max(next_z, target_z)

        cmd = start_pose.copy()
        cmd[2] = next_z

        send_endpose(piper, cmd)
        time.sleep(dt)

    print("[WARN] LIFT_Z reached max_iters but not within tolerance.")
    current = read_endpose_raw(piper)
    print("[FINAL]", endpose_readable(current))
    print("final_z_error_mm:", abs(target_z - current[2]) / 1000.0)
    return False


def gripper_mm_to_raw(mm):
    # 你之前 ms.py 里 open=60mm -> 60000，close=10mm -> 10000
    return int(mm * 1000)


def open_gripper(piper, open_mm=60.0, effort=1000):
    raw = gripper_mm_to_raw(open_mm)
    print(f"\n[GRIPPER] open to {open_mm:.1f} mm, raw={raw}")
    for _ in range(5):
        send_gripper(piper, raw, effort)
        time.sleep(0.1)


def close_gripper(piper, close_mm=10.0, effort=1000):
    raw = gripper_mm_to_raw(close_mm)
    print(f"\n[GRIPPER] close to {close_mm:.1f} mm, raw={raw}, effort={effort}")
    for _ in range(8):
        send_gripper(piper, raw, effort)
        time.sleep(0.1)


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


def estimate_marker_once(
    color,
    camera_matrix,
    dist_coeffs,
    dictionary,
    parameters,
    detector,
    marker_size_m,
    marker_id,
):
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect_markers(gray, dictionary, parameters, detector)

    if ids is None or len(ids) == 0:
        return None, corners, ids

    rvecs, tvecs = estimate_pose(corners, marker_size_m, camera_matrix, dist_coeffs)

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


def verify_aruco_pose(record, samples=60, xy_tol_mm=5.0, z_tol_mm=8.0, show=True):
    desired_xyz_mm = np.array(record["marker_camera_xyz_mm"], dtype=np.float64)

    marker_size_m = float(record["marker_size_m"])
    dict_name = record["dict"]
    marker_id = int(record["marker_id"])

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()

    camera_matrix, dist_coeffs = get_camera_matrix(intr)
    dictionary, parameters, detector = create_aruco_detector(dict_name)

    xyz_list = []

    print("\n[INFO] 开始 ArUco 到位校验...")

    try:
        while len(xyz_list) < samples:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            show_img = color.copy()

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

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(show_img, corners, ids)

            if result is not None:
                xyz_mm = np.array(result["marker_camera_xyz_m"], dtype=np.float64) * 1000.0
                xyz_list.append(xyz_mm)

                rvec = np.array(result["marker_camera_rvec"], dtype=np.float64)
                tvec = np.array(result["marker_camera_xyz_m"], dtype=np.float64)

                try:
                    cv2.drawFrameAxes(
                        show_img,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        marker_size_m * 0.7,
                    )
                except Exception:
                    pass

                err_now = xyz_mm - desired_xyz_mm
                cv2.putText(
                    show_img,
                    "err_mm=(%.1f, %.1f, %.1f)" % tuple(err_now),
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                )
            else:
                cv2.putText(
                    show_img,
                    "Target ArUco not detected",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

            if show:
                cv2.imshow("verify_aruco_pose", show_img)
                cv2.waitKey(1)

            time.sleep(0.01)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if len(xyz_list) == 0:
        print("[VERIFY FAIL] 未检测到 ArUco。")
        return False

    arr = np.array(xyz_list, dtype=np.float64)
    current_xyz_mm = np.median(arr, axis=0)
    std_xyz_mm = np.std(arr, axis=0)
    err_mm = current_xyz_mm - desired_xyz_mm

    ok = (
        abs(err_mm[0]) <= xy_tol_mm
        and abs(err_mm[1]) <= xy_tol_mm
        and abs(err_mm[2]) <= z_tol_mm
    )

    print("\n========== ArUco Verify Result ==========")
    print("desired_xyz_mm:", desired_xyz_mm.tolist())
    print("current_xyz_mm:", current_xyz_mm.tolist())
    print("error_xyz_mm  :", err_mm.tolist())
    print("std_xyz_mm    :", std_xyz_mm.tolist())
    print("tol_xy_mm:", xy_tol_mm, "tol_z_mm:", z_tol_mm)
    print("result:", "PASS" if ok else "FAIL")
    print("=========================================")

    return ok


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--slave-can", default="can1")
    parser.add_argument("--pose-file", default="taught_full_grasp_poses.json")
    parser.add_argument("--pose-name", default="teach_reagent_grasp_pose_fixed")

    parser.add_argument("--speed", type=int, default=8)
    parser.add_argument("--step-deg", type=float, default=1.0)
    parser.add_argument("--tol-deg", type=float, default=0.3)
    parser.add_argument("--rate", type=float, default=20)

    parser.add_argument("--xy-tol-mm", type=float, default=5.0)
    parser.add_argument("--z-tol-mm", type=float, default=8.0)
    parser.add_argument("--verify-samples", type=int, default=60)

    parser.add_argument("--open-mm", type=float, default=60.0)
    parser.add_argument("--close-mm", type=float, default=10.0)
    parser.add_argument("--gripper-effort", type=int, default=1000)

    parser.add_argument("--close-wait-s", type=float, default=2.0)
    parser.add_argument("--post-waypoints", default="", help="夹取后依次执行的关节位 pose_name，多个用逗号分隔")

    parser.add_argument("--lift-mm", type=float, default=0.0)
    parser.add_argument("--lift-speed", type=int, default=5)
    parser.add_argument("--lift-step-mm", type=float, default=5.0)
    parser.add_argument("--lift-tol-mm", type=float, default=3.0)

    parser.add_argument("--return-speed", type=int, default=None)
    parser.add_argument("--return-step-deg", type=float, default=None)

    parser.add_argument("--skip-open", action="store_true")
    parser.add_argument("--skip-return", action="store_true")
    parser.add_argument("--auto", action="store_true")

    args = parser.parse_args()

    data = json.loads(Path(args.pose_file).read_text())
    if args.pose_name not in data:
        raise RuntimeError(f"pose_name={args.pose_name} not found in {args.pose_file}")

    record = data[args.pose_name]
    target_joint_raw = [int(v) for v in record["slave_joint_raw"]]

    print("========== Auto Grasp By Joint Replay ==========")
    print("slave_can:", args.slave_can)
    print("pose_file:", args.pose_file)
    print("pose_name:", args.pose_name)
    print("target_joint:", joint_readable(target_joint_raw))
    print("target_marker_xyz_mm:", record["marker_camera_xyz_mm"])
    print("open_mm:", args.open_mm, "close_mm:", args.close_mm, "effort:", args.gripper_effort)
    print("skip_open:", args.skip_open, "skip_return:", args.skip_return, "auto:", args.auto)
    print("================================================")
    print("注意：")
    print("1. 运行前必须先停止主从 follow。")
    print("2. 本脚本会记录当前从臂关节角作为 slave_hold_joints。")
    print("3. 然后从臂运行到示教夹取关节角。")
    print("4. ArUco 校验通过后闭合夹爪。")
    print("5. 无论校验失败还是夹取成功，只要机械臂离开开始位置，都会尽量回到 slave_hold_joints。")
    print("================================================")

    if not args.auto:
        input("确认从臂、夹爪、相机线、试剂周围安全，按 Enter 开始...")

    piper = None
    slave_hold_joints = None
    moved_away_from_hold = False
    user_abort = False
    grasp_success = False

    try:
        piper = C_PiperInterface(args.slave_can)
        piper.ConnectPort()
        time.sleep(1.0)

        slave_hold_joints = read_joint_raw(piper)

        print("\n[HOLD_JOINTS] 本次自动动作开始位置：")
        print(joint_readable(slave_hold_joints))

        if not args.skip_open:
            if not args.auto:
                input("确认夹爪张开安全，按 Enter 打开夹爪...")
            open_gripper(piper, open_mm=args.open_mm, effort=args.gripper_effort)
            time.sleep(0.5)

        if not args.auto:
            input("确认路径安全，按 Enter 运行到示教夹取关节角...")

        ok_move = move_joint_slowly(
            piper=piper,
            target_raw=target_joint_raw,
            speed=args.speed,
            step_deg=args.step_deg,
            tol_deg=args.tol_deg,
            rate=args.rate,
            name="GO_GRASP",
        )

        moved_away_from_hold = True

        if not ok_move:
            print("[WARN] 关节角未到位，不进行夹取。准备回到开始位置。")
            return

        if not args.auto:
            input("从臂已到示教关节角，按 Enter 开始 ArUco 到位校验...")

        ok_verify = verify_aruco_pose(
            record=record,
            samples=args.verify_samples,
            xy_tol_mm=args.xy_tol_mm,
            z_tol_mm=args.z_tol_mm,
            show=True,
        )

        if not ok_verify:
            print("[WARN] ArUco 到位校验失败，不闭合夹爪。准备回到开始位置。")
            return

        if not args.auto:
            input("ArUco 校验通过。确认夹爪可以闭合夹取，按 Enter 闭合夹爪...")

        close_gripper(piper, close_mm=args.close_mm, effort=args.gripper_effort)

        print(f"\n[INFO] 夹爪闭合完成，等待 {args.close_wait_s:.1f} s。")
        time.sleep(args.close_wait_s)

        post_names = [x.strip() for x in args.post_waypoints.split(",") if x.strip()]
        if post_names:
            all_data = json.loads(Path(args.pose_file).read_text())
            for idx, post_name in enumerate(post_names, start=1):
                if post_name not in all_data:
                    print(f"[WARN] post waypoint {post_name} 不存在，跳过。")
                    continue

                post_record = all_data[post_name]
                if "slave_joint_raw" not in post_record:
                    print(f"[WARN] post waypoint {post_name} 没有 slave_joint_raw，跳过。")
                    continue

                post_joint_raw = [int(v) for v in post_record["slave_joint_raw"]]

                print(f"\n[POST_WAYPOINT {idx}/{len(post_names)}] 运行到 {post_name}")
                print("[POST TARGET]", joint_readable(post_joint_raw))

                if not args.auto:
                    input(f"确认运行到 {post_name} 的路径安全，按 Enter 继续...")

                ok_post = move_joint_slowly(
                    piper=piper,
                    target_raw=post_joint_raw,
                    speed=args.speed,
                    step_deg=args.step_deg,
                    tol_deg=args.tol_deg,
                    rate=args.rate,
                    max_iters=1500,
                    name=f"POST_{idx}",
                )

                if not ok_post:
                    print(f"[WARN] post waypoint {post_name} 未完全到位，但仍会尝试继续/回位。")

        elif args.lift_mm > 0:
            print("[WARN] 当前不建议继续使用笛卡尔 lift_mm，建议改用 --post-waypoints 关节位拔出。")
            if not args.auto:
                input(f"仍要尝试 Z+ 抬升 {args.lift_mm:.1f} mm，按 Enter 继续，或 Ctrl+C 停止...")
            ok_lift = move_lift_z_slowly(
                piper=piper,
                lift_mm=args.lift_mm,
                speed=args.lift_speed,
                step_mm=args.lift_step_mm,
                tol_mm=args.lift_tol_mm,
                rate=args.rate,
            )
            if not ok_lift:
                print("[WARN] 抬升未完全到位，但仍会尝试回到开始位置。")

        grasp_success = True

    except KeyboardInterrupt:
        user_abort = True
        print("\n[USER STOP] 用户中断。为了安全，不自动回位，请人工检查机械臂状态。")

    except Exception as e:
        print("\n[ERROR] 自动夹取流程出现异常：", repr(e))
        print("[ERROR] 如果机械臂已经离开开始位置，程序会尝试回到 slave_hold_joints。")

    finally:
        if user_abort:
            print("[STOP] 用户主动中断，跳过自动回位。")
            return

        if args.skip_return:
            print("[DONE] skip_return=True，跳过回到开始位置。")
            return

        if piper is None or slave_hold_joints is None:
            print("[WARN] 没有成功记录开始位置，无法自动回位。")
            return

        if not moved_away_from_hold:
            print("[DONE] 机械臂尚未离开开始位置，无需回位。")
            return

        if not args.auto:
            input("确认回到开始位置路径安全，按 Enter 执行 RETURN_HOLD...")

        return_speed = args.return_speed if args.return_speed is not None else args.speed
        return_step_deg = args.return_step_deg if args.return_step_deg is not None else args.step_deg

        print("\n[RETURN] 开始回到本次动作开始位置 slave_hold_joints")
        print("[RETURN TARGET]", joint_readable(slave_hold_joints))

        ok_return = move_joint_slowly(
            piper=piper,
            target_raw=slave_hold_joints,
            speed=return_speed,
            step_deg=return_step_deg,
            tol_deg=args.tol_deg,
            rate=args.rate,
            max_iters=1500,
            name="RETURN_HOLD",
        )

        final_joints = read_joint_raw(piper)
        final_err = max_joint_error_deg(final_joints, slave_hold_joints)

        print("\n========== FINAL RETURN CHECK ==========")
        print("[HOLD ]", joint_readable(slave_hold_joints))
        print("[FINAL]", joint_readable(final_joints))
        print("final_return_error_deg:", final_err)
        print("return_result:", "PASS" if ok_return else "FAIL")
        print("grasp_success:", grasp_success)
        print("========================================")

        if ok_return:
            if grasp_success:
                print("\n[DONE] 自动夹取完成，并已回到开始位置。现在可以恢复主从 follow。")
            else:
                print("\n[DONE] 本次未完成夹取，但已回到开始位置。")
        else:
            print("\n[WARN] 回到开始位置失败或未完全到位，请人工检查后再恢复主从。")


if __name__ == "__main__":
    main()
