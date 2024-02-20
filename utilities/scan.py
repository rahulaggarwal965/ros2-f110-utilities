import numpy as np

__all__ = ["index_at_angle", "angle_at_index"]

def index_at_angle(angle_in_degrees: float, angle_min: float, angle_increment: float) -> int:
    """Returns the index associated with a specific angle in degrees for a laser scan msg

    Parameters
    ----------
    angle_in_degrees : float
        The angle in degrees
    angle_min : float
        The minimum angle of the laser scan message
    angle_increment : float
        The angle increment of the laser scan message

    Returns
    -------
    int
        The corresponding index of the angle
    """
    return int((np.deg2rad(angle_in_degrees) - angle_min) / angle_increment)

def angle_at_index(index: int, angle_min: float, angle_increment: float) -> float:
    """Converts an angle to an index into a ranges array

    Parameters
    ----------
    index : int
        The index to convert
    angle_min : float
        The minimum angle of the laser scan message
    angle_increment : float
        The angle increment of the laser scan message

    Returns
    -------
    float
        The angle in radians
    """
    return angle_min + angle_increment * index
