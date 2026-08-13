import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def create_aruco_detector(dict_name):
    aruco = cv2.aruco

    dict_map = {
        "4x4_50": aruco.DICT_4X4_50,
        "4x4_100": aruco.DICT_4X4_100,
        "4x4_250": aruco.DICT_4X4_250,
        "4x4_1000": aruco.DICT_4X4_1000,
    }

    if dict_name not in dict_map:
        raise ValueError(f"Unsupported dict: {dict_name}")

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

    return cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )


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
    """
    优先使用 OpenCV 自带 estimatePoseSingleMarkers。
    如果当前 OpenCV 版本没有该函数，则自动使用 solvePnP。
    """
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
            rvec = np.zeros((3, 1), dtype=np.float64)
            tvec = np.zeros((3, 1), dtype=np.float64)

        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))

    return np.array(rvecs, dtype=np.float64), np.array(tvecs, dtype=np.float64)


def median_depth(depth_frame, u, v, win=5):
    vals = []
    h = depth_frame.get_height()
    w = depth_frame.get_width()
    r = win // 2

    for yy in range(max(0, v - r), min(h, v + r + 1)):
        for xx in range(max(0, u - r), min(w, u + r + 1)):
            d = depth_frame.get_distance(xx, yy)
            if 0.05 < d < 3.0:
                vals.append(d)

    if len(vals) == 0:
        return 0.0

    return float(np.median(vals))


def estimate_target_marker(color, depth_frame, camera_matrix, dist_coeffs,
                           dictionary, parameters, detector,
                           marker_size_m, marker_id):
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detect_markers(
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

    for i, mid in enumerate(ids.flatten()):
        if int(mid) != int(marker_id):
            continue

        c = corners[i][0]
        center_u = int(np.mean(c[:, 0]))
        center_v = int(np.mean(c[:, 1]))
        depth_m = median_depth(depth_frame, center_u, center_v, win=5)

        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        result = {
            "marker_id": int(mid),
            "rvec": rvec.tolist(),
            "marker_camera_xyz_m": tvec.tolist(),
            "center_pixel": [int(center_u), int(center_v)],
            "depth_center_m": float(depth_m),
        }

        return result, corners, ids

    return None, corners, ids


def load_json(path):
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def save_result(path, key, data):
    old = load_json(path)
    old[key] = data
    path.write_text(json.dumps(old, indent=2, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--marker-size-m",
        type=float,
        required=True,
        help="ArUco 黑色码区域边长。你的码是 32 mm，所以填 0.032。",
    )

    parser.add_argument(
        "--dict",
        default="4x4_50",
        choices=["4x4_50", "4x4_100", "4x4_250", "4x4_1000"],
    )

    parser.add_argument(
        "--marker-id",
        type=int,
        default=0,
        help="要跟踪的 ArUco ID，默认 0。",
    )

    parser.add_argument(
        "--samples",
        type=int,
        default=60,
        help="按 a 后采样帧数。",
    )

    parser.add_argument(
        "--out",
        default="aruco_marker_center.json",
        help="保存 JSON 文件。",
    )

    args = parser.parse_args()
    out_path = Path(args.out)

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

    print("========== ArUco RealSense Locator ==========")
    print("OpenCV:", cv2.__version__)
    print("dict:", args.dict)
    print("marker_id:", args.marker_id)
    print("marker_size_m:", args.marker_size_m)
    print("out:", out_path)
    print("---------------------------------------------")
    print("操作：")
    print("a：采样并保存当前 ArUco 中心坐标")
    print("q / ESC：退出")
    print("---------------------------------------------")
    print("相机内参：")
    print("width:", intr.width, "height:", intr.height)
    print("fx:", intr.fx, "fy:", intr.fy)
    print("ppx:", intr.ppx, "ppy:", intr.ppy)
    print("dist_coeffs:", dist_coeffs.tolist())
    print("=============================================")

    last_print_time = 0.0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            show = color.copy()

            result, corners, ids = estimate_target_marker(
                color=color,
                depth_frame=depth_frame,
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
                marker_xyz = result["marker_camera_xyz_m"]
                center_u, center_v = result["center_pixel"]
                depth_m = result["depth_center_m"]
                rvec = np.array(result["rvec"], dtype=np.float64)
                tvec = np.array(marker_xyz, dtype=np.float64)

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

                cv2.circle(show, (center_u, center_v), 5, (0, 0, 255), -1)

                cv2.putText(
                    show,
                    f"id={args.marker_id}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    show,
                    f"tvec=({marker_xyz[0]:.3f},{marker_xyz[1]:.3f},{marker_xyz[2]:.3f})m",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    show,
                    f"depth_center={depth_m:.3f}m",
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                now = time.time()
                if now - last_print_time > 0.5:
                    print(
                        f"id={args.marker_id} "
                        f"marker_camera_xyz_m=({marker_xyz[0]:.4f}, {marker_xyz[1]:.4f}, {marker_xyz[2]:.4f}) "
                        f"depth_center_m={depth_m:.4f}"
                    )
                    last_print_time = now

            else:
                cv2.putText(
                    show,
                    f"Target ArUco id={args.marker_id} not detected",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

                if ids is not None and len(ids) > 0:
                    detected_ids = [int(x) for x in ids.flatten()]
                    cv2.putText(
                        show,
                        f"Detected ids: {detected_ids}",
                        (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 255),
                        2,
                    )

            cv2.imshow("aruco_detect_realsense", show)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            elif key == ord("a"):
                print(f"[COLLECT] collecting {args.samples} samples...")

                marker_samples = []
                depth_samples = []
                pixel_samples = []

                for _ in range(args.samples):
                    frames2 = pipeline.wait_for_frames()
                    frames2 = align.process(frames2)

                    color_frame2 = frames2.get_color_frame()
                    depth_frame2 = frames2.get_depth_frame()

                    if not color_frame2 or not depth_frame2:
                        continue

                    color2 = np.asanyarray(color_frame2.get_data())

                    sample_result, _, _ = estimate_target_marker(
                        color=color2,
                        depth_frame=depth_frame2,
                        camera_matrix=camera_matrix,
                        dist_coeffs=dist_coeffs,
                        dictionary=dictionary,
                        parameters=parameters,
                        detector=detector,
                        marker_size_m=args.marker_size_m,
                        marker_id=args.marker_id,
                    )

                    if sample_result is None:
                        continue

                    marker_samples.append(sample_result["marker_camera_xyz_m"])
                    depth_samples.append(sample_result["depth_center_m"])
                    pixel_samples.append(sample_result["center_pixel"])

                    time.sleep(0.005)

                if len(marker_samples) == 0:
                    print("[WARN] 没有采到有效 ArUco。请检查标签是否完整可见。")
                    continue

                marker_arr = np.array(marker_samples, dtype=np.float64)
                depth_arr = np.array(depth_samples, dtype=np.float64)
                pixel_arr = np.array(pixel_samples, dtype=np.float64)

                marker_med = np.median(marker_arr, axis=0)
                marker_std = np.std(marker_arr, axis=0)

                depth_med = float(np.median(depth_arr))
                depth_std = float(np.std(depth_arr))
                pixel_med = np.median(pixel_arr, axis=0)

                save_data = {
                    "name": "marker_center",
                    "timestamp": time.time(),
                    "dict": args.dict,
                    "marker_id": int(args.marker_id),
                    "marker_size_m": float(args.marker_size_m),
                    "marker_camera_xyz_m": marker_med.tolist(),
                    "std_marker_xyz_m": marker_std.tolist(),
                    "center_pixel": [int(pixel_med[0]), int(pixel_med[1])],
                    "depth_center_m": depth_med,
                    "std_depth_center_m": depth_std,
                    "valid_samples": int(len(marker_samples)),
                    "requested_samples": int(args.samples),
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

                save_result(out_path, "marker_center", save_data)

                print("[SAVE]", out_path)
                print(json.dumps(save_data, indent=2, ensure_ascii=False))

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
