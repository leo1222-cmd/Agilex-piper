import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from piper_sdk import C_PiperForwardKinematics


JOINT_MIN_DEG = np.array(
    [-150.0, 0.0, -170.0, -100.0, -70.0, -120.0],
    dtype=float,
)

JOINT_MAX_DEG = np.array(
    [150.0, 180.0, 0.0, 100.0, 68.0, 120.0],
    dtype=float,
)


def _wrap_deg(v):
    return (
        np.asarray(v, dtype=float)
        + 180.0
    ) % 360.0 - 180.0


def _pose_error(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    xyz = a[:3] - b[:3]

    rpy = _wrap_deg(
        a[3:6] - b[3:6]
    )

    return xyz, rpy


def _make_fk(dh_is_offset):
    errors = []

    for mode in ("kw", "pos"):
        try:
            if mode == "kw":
                return C_PiperForwardKinematics(
                    dh_is_offset=int(
                        dh_is_offset
                    )
                )

            return C_PiperForwardKinematics(
                int(dh_is_offset)
            )

        except Exception as exc:
            errors.append(
                repr(exc)
            )

    raise RuntimeError(
        "无法创建 C_PiperForwardKinematics；"
        "请确认当前 piper_sdk 支持 dh_is_offset。"
        f" errors={errors}"
    )


def _fk_pose(fk, q_deg):
    """
    PiPER C_PiperForwardKinematics.CalFK()
    返回 6 个连杆的位姿：

        [
            pose_joint1,
            pose_joint2,
            ...
            pose_joint6
        ]

    每一项为：
        [X, Y, Z, RX, RY, RZ]

    我们需要的是最后一行，也就是末端执行器位姿。
    """

    q_rad = np.deg2rad(
        np.asarray(
            q_deg,
            dtype=float,
        )
    ).tolist()

    result = fk.CalFK(
        q_rad
    )

    arr = np.asarray(
        result,
        dtype=float,
    )

    # --------------------------------------------------------
    # 官方 SDK 当前返回：
    # shape = (6, 6)
    #
    # 最后一行才是末端位姿。
    # --------------------------------------------------------
    if (
        arr.ndim == 2
        and
        arr.shape[0] >= 1
        and
        arr.shape[1] >= 6
    ):
        pose = arr[-1, :6]

    # --------------------------------------------------------
    # 兼容未来/其他版本 SDK：
    # 如果直接返回一维 [X,Y,Z,RX,RY,RZ]
    # --------------------------------------------------------
    elif (
        arr.ndim == 1
        and
        arr.size >= 6
    ):
        pose = arr[:6]

    else:
        raise RuntimeError(
            "CalFK 返回格式异常: "
            f"shape={arr.shape}, "
            f"value={result!r}"
        )

    return np.asarray(
        pose,
        dtype=float,
    )

def select_fk_model(
    current_joint_deg,
    current_pose_mm_deg,
):
    current_joint_deg = np.asarray(
        current_joint_deg,
        dtype=float,
    )

    current_pose_mm_deg = np.asarray(
        current_pose_mm_deg,
        dtype=float,
    )

    candidates = []

    for dh in (1, 0):
        try:
            fk = _make_fk(
                dh
            )

            pose = _fk_pose(
                fk,
                current_joint_deg,
            )

            xyz_err, rpy_err = (
                _pose_error(
                    pose,
                    current_pose_mm_deg,
                )
            )

            score = float(
                np.linalg.norm(
                    xyz_err
                )
                +
                2.0
                * np.linalg.norm(
                    rpy_err
                )
            )

            candidates.append(
                (
                    score,
                    dh,
                    fk,
                    pose,
                    xyz_err,
                    rpy_err,
                )
            )

        except Exception:
            continue

    if not candidates:
        raise RuntimeError(
            "FK 初始化失败，"
            "无法进行动态拔出规划。"
        )

    candidates.sort(
        key=lambda x: x[0]
    )

    (
        _,
        dh,
        fk,
        pose,
        xyz_err,
        rpy_err,
    ) = candidates[0]

    max_xyz = float(
        np.max(
            np.abs(
                xyz_err
            )
        )
    )

    max_rpy = float(
        np.max(
            np.abs(
                rpy_err
            )
        )
    )

    if (
        max_xyz > 25.0
        or
        max_rpy > 8.0
    ):
        raise RuntimeError(
            "FK 与机器人当前 EndPose "
            "不一致，拒绝规划。"
            f" dh_is_offset={dh}, "
            f"max_xyz_error="
            f"{max_xyz:.3f} mm, "
            f"max_rpy_error="
            f"{max_rpy:.3f} deg"
        )

    return (
        fk,
        dh,
        pose,
        max_xyz,
        max_rpy,
    )


@dataclass
class PullPlan:
    target_joint_deg: list
    target_pose_mm_deg: list
    dh_is_offset: int

    final_dz_mm: float
    final_xy_drift_mm: float

    max_xy_drift_mm: float
    critical_xy_drift_mm: float

    max_rpy_drift_deg: float

    min_dz_mm: float
    score: float


def _path_metrics(
    fk,
    q0,
    q1,
    start_pose,
    critical_z_mm,
    samples=121,
):
    q0 = np.asarray(
        q0,
        dtype=float,
    )

    q1 = np.asarray(
        q1,
        dtype=float,
    )

    start_pose = np.asarray(
        start_pose,
        dtype=float,
    )

    max_xy = 0.0
    critical_xy = 0.0
    max_rpy = 0.0

    min_dz = float(
        "inf"
    )

    final_pose = None

    for alpha in np.linspace(
        0.0,
        1.0,
        int(samples),
    ):
        q = (
            q0
            +
            alpha
            * (
                q1 - q0
            )
        )

        pose = _fk_pose(
            fk,
            q,
        )

        final_pose = pose

        dx = float(
            pose[0]
            - start_pose[0]
        )

        dy = float(
            pose[1]
            - start_pose[1]
        )

        dz = float(
            pose[2]
            - start_pose[2]
        )

        xy = math.hypot(
            dx,
            dy,
        )

        rpy = _wrap_deg(
            pose[3:6]
            - start_pose[3:6]
        )

        max_xy = max(
            max_xy,
            xy,
        )

        max_rpy = max(
            max_rpy,
            float(
                np.max(
                    np.abs(
                        rpy
                    )
                )
            ),
        )

        min_dz = min(
            min_dz,
            dz,
        )

        if (
            dz
            <=
            float(
                critical_z_mm
            )
            + 1e-9
        ):
            critical_xy = max(
                critical_xy,
                xy,
            )

    final_dz = float(
        final_pose[2]
        - start_pose[2]
    )

    final_xy = math.hypot(
        float(
            final_pose[0]
            - start_pose[0]
        ),
        float(
            final_pose[1]
            - start_pose[1]
        ),
    )

    return {
        "final_pose":
            final_pose,

        "final_dz":
            final_dz,

        "final_xy":
            final_xy,

        "max_xy":
            max_xy,

        "critical_xy":
            critical_xy,

        "max_rpy":
            max_rpy,

        "min_dz":
            min_dz,
    }


def plan_relative_pull(
    current_joint_raw,
    current_endpose_raw,

    distance_mm=300.0,

    final_z_tol_mm=8.0,

    max_xy_drift_mm=80.0,

    critical_z_mm=120.0,

    critical_xy_drift_mm=30.0,

    max_rpy_drift_deg=35.0,
):
    """
    根据当前真实抓取构型动态计算
    一个向上约 distance_mm 的
    可达终端关节位。

    最终执行为关节空间插值，
    因此 TCP 会形成自然小弧线，
    而不是强制笛卡尔直线。
    """

    q0 = (
        np.asarray(
            current_joint_raw,
            dtype=float,
        )
        / 1000.0
    )

    start_pose = (
        np.asarray(
            current_endpose_raw,
            dtype=float,
        )
        / 1000.0
    )

    (
        fk,
        dh,
        fk_start,
        fk_xyz_err,
        fk_rpy_err,
    ) = select_fk_model(
        q0,
        start_pose,
    )

    print(
        "[IK FK] "
        f"dh_is_offset={dh}, "
        f"xyz_error="
        f"{fk_xyz_err:.4f} mm, "
        f"rpy_error="
        f"{fk_rpy_err:.4f} deg"
    )

    target_z_fk = float(
        fk_start[2]
        +
        float(
            distance_mm
        )
    )

    def residual(q):
        pose = _fk_pose(
            fk,
            q,
        )

        dxy = (
            pose[:2]
            -
            fk_start[:2]
        )

        dz = float(
            pose[2]
            -
            target_z_fk
        )

        drpy = _wrap_deg(
            pose[3:6]
            -
            fk_start[3:6]
        )

        dq = (
            np.asarray(
                q,
                dtype=float,
            )
            -
            q0
        )

        # Z 是核心目标。
        # XY / 姿态均作为软约束。
        # 同时偏好关节变化较小的解。
        return np.concatenate([
            dxy / 18.0,

            np.array([
                dz / 2.5
            ]),

            drpy / 12.0,

            dq / 45.0,
        ])

    seeds = [
        q0.copy()
    ]

    for j2 in (
        -25.0,
        -12.0,
        12.0,
        25.0,
    ):
        s = q0.copy()
        s[1] += j2
        seeds.append(s)

    for j3 in (
        -20.0,
        -10.0,
        10.0,
        20.0,
    ):
        s = q0.copy()
        s[2] += j3
        seeds.append(s)

    for j5 in (
        -25.0,
        -12.0,
        12.0,
    ):
        s = q0.copy()
        s[4] += j5
        seeds.append(s)

    for (
        d2,
        d3,
        d5,
    ) in [
        (
            -20.0,
            15.0,
            -20.0,
        ),
        (
            -15.0,
            10.0,
            -15.0,
        ),
        (
            15.0,
            -10.0,
            -15.0,
        ),
        (
            20.0,
            -15.0,
            -20.0,
        ),
    ]:
        s = q0.copy()

        s[1] += d2
        s[2] += d3
        s[4] += d5

        seeds.append(s)

    candidates = []

    for seed in seeds:
        seed = np.clip(
            seed,
            JOINT_MIN_DEG
            + 1e-4,
            JOINT_MAX_DEG
            - 1e-4,
        )

        try:
            sol = least_squares(
                residual,
                seed,

                bounds=(
                    JOINT_MIN_DEG,
                    JOINT_MAX_DEG,
                ),

                max_nfev=1500,

                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
            )

        except Exception:
            continue

        q1 = np.asarray(
            sol.x,
            dtype=float,
        )

        metrics = _path_metrics(
            fk,
            q0,
            q1,
            fk_start,

            critical_z_mm=float(
                critical_z_mm
            ),

            samples=121,
        )

        z_err = abs(
            metrics["final_dz"]
            -
            float(
                distance_mm
            )
        )

        backward = max(
            0.0,
            -metrics["min_dz"],
        )

        q_move = float(
            np.linalg.norm(
                q1 - q0
            )
        )

        score = (
            20.0
            * z_err

            +
            2.0
            * metrics[
                "critical_xy"
            ]

            +
            0.5
            * metrics[
                "max_xy"
            ]

            +
            0.4
            * metrics[
                "max_rpy"
            ]

            +
            0.08
            * q_move

            +
            10.0
            * backward
        )

        candidates.append(
            (
                score,
                q1,
                metrics,
                bool(
                    sol.success
                ),
            )
        )

    if not candidates:
        raise RuntimeError(
            "动态 IK "
            "没有得到任何候选解。"
        )

    candidates.sort(
        key=lambda x: x[0]
    )

    accepted = []

    for (
        score,
        q1,
        m,
        success,
    ) in candidates:

        z_err = abs(
            m["final_dz"]
            -
            float(
                distance_mm
            )
        )

        if (
            z_err
            >
            float(
                final_z_tol_mm
            )
        ):
            continue

        if (
            m["max_xy"]
            >
            float(
                max_xy_drift_mm
            )
        ):
            continue

        if (
            m["critical_xy"]
            >
            float(
                critical_xy_drift_mm
            )
        ):
            continue

        if (
            m["max_rpy"]
            >
            float(
                max_rpy_drift_deg
            )
        ):
            continue

        if (
            m["min_dz"]
            <
            -5.0
        ):
            continue

        accepted.append(
            (
                score,
                q1,
                m,
            )
        )

    if not accepted:
        (
            best_score,
            best_q,
            best_m,
            _,
        ) = candidates[0]

        raise RuntimeError(
            "没有候选路径满足动态拔出约束。"

            f" best_final_dz="
            f"{best_m['final_dz']:.2f} mm,"

            f" best_critical_xy="
            f"{best_m['critical_xy']:.2f} mm,"

            f" best_max_xy="
            f"{best_m['max_xy']:.2f} mm,"

            f" best_max_rpy="
            f"{best_m['max_rpy']:.2f} deg."

            " 请调整 pull 约束或抓取构型，"
            "不要直接真机硬跑。"
        )

    (
        score,
        q1,
        m,
    ) = accepted[0]

    fk_final = np.asarray(
        m["final_pose"],
        dtype=float,
    )

    display_pose = (
        start_pose.copy()
    )

    display_pose[:3] += (
        fk_final[:3]
        -
        fk_start[:3]
    )

    display_pose[3:6] += (
        _wrap_deg(
            fk_final[3:6]
            -
            fk_start[3:6]
        )
    )

    return PullPlan(
        target_joint_deg=(
            q1.tolist()
        ),

        target_pose_mm_deg=(
            display_pose.tolist()
        ),

        dh_is_offset=int(
            dh
        ),

        final_dz_mm=float(
            m["final_dz"]
        ),

        final_xy_drift_mm=float(
            m["final_xy"]
        ),

        max_xy_drift_mm=float(
            m["max_xy"]
        ),

        critical_xy_drift_mm=float(
            m["critical_xy"]
        ),

        max_rpy_drift_deg=float(
            m["max_rpy"]
        ),

        min_dz_mm=float(
            m["min_dz"]
        ),

        score=float(
            score
        ),
    )
