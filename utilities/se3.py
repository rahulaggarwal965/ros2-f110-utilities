from typing import Union
from geometry_msgs.msg import Transform, TransformStamped, Pose, PoseStamped, Quaternion, Vector3, Point
import numpy as np

__all__ = ["from_rotation_translation", "from_quat_pos", "from_msg", "inverse", "transform_points"]

def from_rotation_translation(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Creates an SE3 (4x4 matrix) from a rotation and a translation

    Parameters
    ----------
    R : np.ndarray
        3x3 Rotation SO(3) matrix
    t : np.ndarray
        3 element translation vector

    Returns
    -------
    np.ndarray
        4x4 SE(3) matrix representation
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def from_quat_pos(quat: Quaternion, pos: Union[Vector3, Point]) -> np.ndarray:
    """Creates an SE3 (4x4 matrix representation) from a quaternion and a position

    Parameters
    ----------
    quat : Quaternion
        A 4-element quaternion (x, y, z, w)
    pos : Union[Vector3, Point]
        A 3-element position [x, y, z]

    Returns
    -------
    np.ndarray
        4x4 SE(3) matrix representation
    """
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
    t = np.asarray([pos.x, pos.y, pos.z])
    return from_rotation_translation(R.as_matrix(), t)

def from_msg(msg: Union[Transform, TransformStamped, Pose, PoseStamped]) -> np.ndarray:
    """Creates an SE3 (4x4 matrix representation) from a variety of ROS2 transform/pose message types

    Parameters
    ----------
    msg : Union[Transform, TransformStamped, Pose, PoseStamped]
        The message which should be transformed into an SE3 matrix

    Returns
    -------
    np.ndarray
        4x4 SE(3) matrix representation
    """
    # remove stamps
    if type(msg) == TransformStamped:
        msg = msg.transform
    elif type(msg) == PoseStamped:
        msg = msg.pose

    if type(msg) == Transform:
        q, t = msg.rotation, msg.translation
    elif type(msg) == Pose:
        q, t = msg.orientation, msg.position

    return from_quat_pos(q, t)

def inverse(T: np.ndarray) -> np.ndarray:
    """Fast inverse of an SE3 (4x4 matrix representation)

    Parameters
    ----------
    T : np.ndarray
        4x4 matrix representing an SE3

    Returns
    -------
    np.ndarray
        T⁻¹, the inverse of T
    """
    R = T[:3, :3]
    t = T[:3, [3]]

    return np.block([[R.T, -R.T @ t], [0, 0, 0, 1]])

def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Uses an SE3 matrix to transform an array of points

    Parameters
    ----------
    T : np.ndarray
        4x4 matrix representing an SE3
    points : np.ndarray
        (N, 2), (N, 3) or (N, 4) array of points

    Returns
    -------
    np.ndarray
        the transformed points with equivalent dimensions to `points`
    """
    N, D = points.shape
    if D == 2:
        points = np.column_stack((points, np.zeros(N), np.ones(N)))
    elif D == 3:
        points = np.column_stack((points, np.ones(N)))

    return (points @ T.T)[:, :D]
