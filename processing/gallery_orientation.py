"""Reconstruct the archived gallery's reference-frame orientation errors.

The geometry below is the joint chain from the experiment's
``urdf/panda_no_gripper.urdf`` (SHA-256 recorded in MODEL_SOURCE). Only its
kinematic origins and axes are needed; no URDF, ROS, or Pinocchio is required.
The export step verifies both FK positions and orientation-error norms against
the existing processed archive before releasing the additional components.
"""

from __future__ import annotations

import numpy as np


MODEL_SOURCE = {
    "source": "urdf/panda_no_gripper.urdf",
    "sha256": "8c150cec81f16c549b2f6514783f96e61a94c53c7e7015a7deb60613ad86a3d4",
    "base_frame": "panda_link0",
    "end_frame": "panda_link8",
    "joint_names": [f"panda_joint{i}" for i in range(1, 9)],
    "joint_types": ["revolute"] * 7 + ["fixed"],
    "origin_xyz_m": [
        [0.0, 0.0, 0.333], [0.0, 0.0, 0.0], [0.0, -0.316, 0.0],
        [0.0825, 0.0, 0.0], [-0.0825, 0.384, 0.0], [0.0, 0.0, 0.0],
        [0.088, 0.0, 0.0], [0.0, 0.0, 0.107],
    ],
    "origin_rpy_rad": [
        [0.0, 0.0, 0.0], [-1.5707963267948966, 0.0, 0.0],
        [1.5707963267948966, 0.0, 0.0], [1.5707963267948966, 0.0, 0.0],
        [-1.5707963267948966, 0.0, 0.0], [1.5707963267948966, 0.0, 0.0],
        [1.5707963267948966, 0.0, 0.0], [0.0, 0.0, 0.0],
    ],
    "revolute_axes": [[0.0, 0.0, 1.0]] * 7,
}

ORIENTATION_COLUMNS = (
    "ee_orientation_rx_deg", "ee_orientation_ry_deg", "ee_orientation_rz_deg",
    "ee_orientation_max_abs_component_deg",
)


def _rotation(axis: tuple[float, float, float], angle: np.ndarray) -> np.ndarray:
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    angle = np.asarray(angle)
    return (np.eye(3) + np.sin(angle)[..., None, None] * cross
            + (1.0 - np.cos(angle))[..., None, None] * (cross @ cross))


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Return fixed-base panda_link8 transforms for an N-by-7 joint array."""
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all():
        raise ValueError("Expected finite N-by-7 joint positions")
    transform = np.repeat(np.eye(4)[None, :, :], len(q), axis=0)
    for i, (xyz, rpy) in enumerate(zip(MODEL_SOURCE["origin_xyz_m"],
                                       MODEL_SOURCE["origin_rpy_rad"])):
        roll, pitch, yaw = rpy
        fixed = np.eye(4)
        fixed[:3, :3] = (_rotation((0, 0, 1), yaw)
                         @ _rotation((0, 1, 0), pitch)
                         @ _rotation((1, 0, 0), roll))
        fixed[:3, 3] = xyz
        transform = transform @ fixed
        if i < 7:
            moving = np.repeat(np.eye(4)[None, :, :], len(q), axis=0)
            moving[:, :3, :3] = _rotation(MODEL_SOURCE["revolute_axes"][i], q[:, i])
            transform = transform @ moving
    return transform


def orientation_error_deg(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return log(R_reference.T @ R_actual) in degrees, in the reference frame.

    This NumPy logarithm covers the archived tracking errors. It rejects the
    singular 180-degree neighborhood instead of silently choosing an axis.
    """
    relative = reference.transpose(0, 2, 1) @ actual
    skew = np.stack((relative[:, 2, 1] - relative[:, 1, 2],
                     relative[:, 0, 2] - relative[:, 2, 0],
                     relative[:, 1, 0] - relative[:, 0, 1]), axis=1) / 2.0
    sine = np.linalg.norm(skew, axis=1)
    cosine = np.clip((np.trace(relative, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arctan2(sine, cosine)
    if np.any(np.pi - angle < 1e-6):
        raise ValueError("Orientation error too close to 180 degrees for gallery reconstruction")
    scale = np.divide(angle, sine, out=np.ones_like(angle), where=sine > 1e-15)
    return np.rad2deg(skew * scale[:, None])
