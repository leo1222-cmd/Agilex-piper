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


def estimate_marker_once(color, camera_matrix, dist_coeffs,
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

        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        c = corners[i][0]
        center_u = int(np.mean(c[:, 0]))
        center_v = int(np.mean(c[:, 1]))

        return {
            "marker_id": int(mid),
            "marker_camera_rvec": rvec.tolist(),
            "marker_camera_xyz_m": tvec.tolist(),
            "center_pixel": [center_u, center_v],
        }, corners, ids

    return None, corners, ids


def load_json(path):
    if not path.exists():
        return {}

    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def save_pose(path, pose_name, data):
    old = load_json(path)
    old[pose_name] = data
    path.write_text(json.dumps(old, indent=2, ensure_ascii=False))


def collect_samples(pipeline, align, camera_matrix, dist_coeffs,
                    dictionary, parameters, detector,
                    marker_size_m, marker_id, samples):
    tvec_samples = []
    rvec_samples = []
    pixel_samples = []

    for _ in range(samples):
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color = np.asanyarray(color_frame.get_data())

        result, _, _ = estimate_marker_once(
            color=color,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            dictionary=dictionary,
            parameters=parameters,
            detector=detector,
            marker_size_m=marker_size_m,
            marker_id=marker_id,
        )

        if result is None:
            continue

        tvec_samples.append(result["marker_camera_xyz_m"])
        rvec_samples.append(result["marker_camera_rvec"])
        pixel_samples.append(result["center_pixel"])

        time.sleep(0.005)

    if len(tvec_samples) == 0:
        return None

    tvec_arr = np.array(tvec_samples, dtype=np.float64)
    rvec_arr = np.array(rvec_samples, dtype=np.float64)
    pixel_arr = np.array(pixel_samples, dtype=np.float64)

    data = {
        "marker_camera_xyz_m": np.median(tvec_arr, axis=0).tolist(),
        "marker_camera_rvec": np.median(rvec_arr, axis=0).tolist(),
        "center_pixel": [int(x) for x in np.median(pixel_arr, axis=0).tolist()],
        "std_marker_xyz_m": np.std(tvec_arr, axis=0).tolist(),
        "std_marker_rvec": np.std(rvec_arr, axis=0).tolist(),
        "valid_samples": int(len(tvec_samples)),
        "requested_samples": int(samples),
    }

    return data


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--mode", choices=["teach", "check"], required=True)
    parser.add_argument("--pose-name", required=True)
    parser.add_argument("--out", default="taught_marker_poses.json")

    parser.add_argument("--marker-size-m", type=float, required=True)
    parser.add_argument("--dict", default="4x4_50",
                        choices=["4x4_50", "4x4_100", "4x4_250", "4x4_1000"])
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--samples", type=int, default=80)

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

    desired_pose = None
    if args.mode == "check":
        all_poses = load_json(out_path)
        if args.pose_name not in all_poses:
            raise RuntimeError(f"Cannot find pose '{args.pose_name}' in {out_path}")

        desired_pose = all_poses[args.pose_name]
        print("Loaded desired pose:")
        print(json.dumps(desired_pose, indent=2, ensure_ascii=False))

    print("========== Marker Pose Teach/Check ==========")
    print("mode:", args.mode)
    print("pose_name:", args.pose_name)
    print("out:", out_path)
    print("dict:", args.dict)
    print("marker_id:", args.marker_id)
    print("marker_size_m:", args.marker_size_m)
    print("---------------------------------------------")
    if args.mode == "teach":
        print("操作：手动把夹爪移动到正确位置后，按 a 采样保存；按 q 退出")
    else:
        print("操作：观察 error_xyz_mm，手动移动机械臂让误差接近 0；按 q 退出")
    print("=============================================")

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
                xyz = result["marker_camera_xyz_m"]
                rvec = np.array(result["marker_camera_rvec"], dtype=np.float64)
                tvec = np.array(xyz, dtype=np.float64)
                u, v = result["center_pixel"]

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

                cv2.circle(show, (u, v), 5, (0, 0, 255), -1)

                cv2.putText(
                    show,
                    f"current xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )

                if args.mode == "check" and desired_pose is not None:
                    desired_xyz = np.array(desired_pose["marker_camera_xyz_m"], dtype=np.float64)
                    current_xyz = np.array(xyz, dtype=np.float64)

                    error = current_xyz - desired_xyz
                    error_mm = error * 1000.0

                    cv2.putText(
                        show,
                        f"error xyz=({error_mm[0]:.1f},{error_mm[1]:.1f},{error_mm[2]:.1f})mm",
                        (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 0, 0),
                        2,
                    )

                    now = time.time()
                    if now - last_print_time > 0.5:
                        print(
                            "current_xyz_m=(%.4f, %.4f, %.4f) "
                            "desired_xyz_m=(%.4f, %.4f, %.4f) "
                            "error_xyz_mm=(%.1f, %.1f, %.1f)"
                            % (
                                current_xyz[0], current_xyz[1], current_xyz[2],
                                desired_xyz[0], desired_xyz[1], desired_xyz[2],
                                error_mm[0], error_mm[1], error_mm[2],
                            )
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

            cv2.imshow("teach_marker_pose", show)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            elif key == ord("a") and args.mode == "teach":
                print(f"[COLLECT] collecting {args.samples} samples...")

                pose_data = collect_samples(
                    pipeline=pipeline,
                    align=align,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    dictionary=dictionary,
                    parameters=parameters,
                    detector=detector,
                    marker_size_m=args.marker_size_m,
                    marker_id=args.marker_id,
                    samples=args.samples,
                )

                if pose_data is None:
                    print("[WARN] no valid samples. ArUco 没有稳定识别。")
                    continue

                pose_data.update({
                    "pose_name": args.pose_name,
                    "timestamp": time.time(),
                    "dict": args.dict,
                    "marker_id": int(args.marker_id),
                    "marker_size_m": float(args.marker_size_m),
                    "camera_intrinsics": {
                        "width": intr.width,
                        "height": intr.height,
                        "fx": intr.fx,
                        "fy": intr.fy,
                        "ppx": intr.ppx,
                        "ppy": intr.ppy,
                        "coeffs": list(intr.coeffs),
                    },
                })

                save_pose(out_path, args.pose_name, pose_data)

                print("[SAVE]", out_path)
                print(json.dumps(pose_data, indent=2, ensure_ascii=False))

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
