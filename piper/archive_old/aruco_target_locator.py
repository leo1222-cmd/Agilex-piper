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


def marker_point_to_camera(rvec, tvec, point_marker):
    R, _ = cv2.Rodrigues(rvec)

    p_m = np.array(point_marker, dtype=np.float64).reshape(3, 1)
    t = np.array(tvec, dtype=np.float64).reshape(3, 1)

    p_c = R @ p_m + t
    return p_c.flatten()


def project_marker_point(camera_matrix, dist_coeffs, rvec, tvec, point_marker):
    point_3d = np.array([point_marker], dtype=np.float64)

    img_pts, _ = cv2.projectPoints(
        point_3d,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )

    u, v = img_pts[0][0]
    return int(round(u)), int(round(v))


def estimate_once(color, camera_matrix, dist_coeffs,
                  dictionary, parameters, detector,
                  marker_size_m, marker_id,
                  target_marker_xyz_m):
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

        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        marker_camera = np.array(tvec, dtype=np.float64)
        target_camera = marker_point_to_camera(
            rvec,
            tvec,
            target_marker_xyz_m,
        )

        target_pixel = project_marker_point(
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            target_marker_xyz_m,
        )

        result = {
            "marker_id": int(mid),
            "rvec": rvec.tolist(),
            "marker_camera_xyz_m": marker_camera.tolist(),
            "target_camera_xyz_m": target_camera.tolist(),
            "target_pixel": [int(target_pixel[0]), int(target_pixel[1])],
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

    parser.add_argument("--marker-size-m", type=float, required=True)
    parser.add_argument("--dict", default="4x4_50",
                        choices=["4x4_50", "4x4_100", "4x4_250", "4x4_1000"])
    parser.add_argument("--marker-id", type=int, default=0)

    parser.add_argument("--target-name", default="target_1")
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)

    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--out", default="aruco_targets_camera.json")

    args = parser.parse_args()

    target_marker_xyz_m = np.array([args.dx, args.dy, args.dz], dtype=np.float64)
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

    print("========== ArUco Target Locator ==========")
    print("OpenCV:", cv2.__version__)
    print("dict:", args.dict)
    print("marker_id:", args.marker_id)
    print("marker_size_m:", args.marker_size_m)
    print("target_name:", args.target_name)
    print("target_marker_xyz_m:", target_marker_xyz_m.tolist())
    print("out:", out_path)
    print("------------------------------------------")
    print("操作：")
    print("a：采样并保存目标点")
    print("q / ESC：退出")
    print("------------------------------------------")
    print("说明：输出 target_camera_xyz_m，是相机坐标系下的目标点。")
    print("==========================================")

    last_print_time = 0.0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            show = color.copy()

            result, corners, ids = estimate_once(
                color=color,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                dictionary=dictionary,
                parameters=parameters,
                detector=detector,
                marker_size_m=args.marker_size_m,
                marker_id=args.marker_id,
                target_marker_xyz_m=target_marker_xyz_m,
            )

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(show, corners, ids)

            if result is not None:
                marker_xyz = result["marker_camera_xyz_m"]
                target_xyz = result["target_camera_xyz_m"]
                target_u, target_v = result["target_pixel"]

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

                cv2.circle(show, (target_u, target_v), 8, (255, 0, 0), -1)

                cv2.putText(
                    show,
                    f"id={args.marker_id}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    show,
                    f"marker=({marker_xyz[0]:.3f},{marker_xyz[1]:.3f},{marker_xyz[2]:.3f})m",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    show,
                    f"{args.target_name}=({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f})m",
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                )

                now = time.time()
                if now - last_print_time > 0.5:
                    print(
                        f"marker_camera_xyz=({marker_xyz[0]:.4f}, {marker_xyz[1]:.4f}, {marker_xyz[2]:.4f}) "
                        f"target_camera_xyz=({target_xyz[0]:.4f}, {target_xyz[1]:.4f}, {target_xyz[2]:.4f}) "
                        f"target_pixel=({target_u}, {target_v})"
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

            cv2.imshow("aruco_target_locator", show)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            elif key == ord("a"):
                print(f"[COLLECT] collecting {args.samples} samples...")

                marker_samples = []
                target_samples = []
                pixel_samples = []

                for _ in range(args.samples):
                    frames2 = pipeline.wait_for_frames()
                    frames2 = align.process(frames2)

                    color_frame2 = frames2.get_color_frame()
                    if not color_frame2:
                        continue

                    color2 = np.asanyarray(color_frame2.get_data())

                    sample_result, _, _ = estimate_once(
                        color=color2,
                        camera_matrix=camera_matrix,
                        dist_coeffs=dist_coeffs,
                        dictionary=dictionary,
                        parameters=parameters,
                        detector=detector,
                        marker_size_m=args.marker_size_m,
                        marker_id=args.marker_id,
                        target_marker_xyz_m=target_marker_xyz_m,
                    )

                    if sample_result is None:
                        continue

                    marker_samples.append(sample_result["marker_camera_xyz_m"])
                    target_samples.append(sample_result["target_camera_xyz_m"])
                    pixel_samples.append(sample_result["target_pixel"])

                    time.sleep(0.005)

                if len(target_samples) == 0:
                    print("[WARN] 没有采到有效样本。")
                    continue

                marker_arr = np.array(marker_samples, dtype=np.float64)
                target_arr = np.array(target_samples, dtype=np.float64)
                pixel_arr = np.array(pixel_samples, dtype=np.float64)

                marker_med = np.median(marker_arr, axis=0)
                target_med = np.median(target_arr, axis=0)
                pixel_med = np.median(pixel_arr, axis=0)

                marker_std = np.std(marker_arr, axis=0)
                target_std = np.std(target_arr, axis=0)

                save_data = {
                    "target_name": args.target_name,
                    "timestamp": time.time(),
                    "dict": args.dict,
                    "marker_id": int(args.marker_id),
                    "marker_size_m": float(args.marker_size_m),
                    "target_marker_xyz_m": target_marker_xyz_m.tolist(),
                    "marker_camera_xyz_m": marker_med.tolist(),
                    "target_camera_xyz_m": target_med.tolist(),
                    "target_pixel": [int(pixel_med[0]), int(pixel_med[1])],
                    "std_marker_xyz_m": marker_std.tolist(),
                    "std_target_xyz_m": target_std.tolist(),
                    "valid_samples": int(len(target_samples)),
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

                save_result(out_path, args.target_name, save_data)

                print("[SAVE]", out_path)
                print(json.dumps(save_data, indent=2, ensure_ascii=False))

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
