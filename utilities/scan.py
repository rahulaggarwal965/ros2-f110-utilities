import numpy as np

__all__ = ["index_at_angle", "angle_at_index"]

def index_at_angle(angle_in_degrees: float, angle_min: float, angle_increment: float):
    return (np.deg2rad(angle_in_degrees) - angle_min) / angle_increment

def angle_at_index(index: int, angle_min: float, angle_increment: float):
    return angle_min + angle_increment * index
