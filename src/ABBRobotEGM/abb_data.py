from typing import NamedTuple


class Orientation(NamedTuple):
    """
    A class used to represent an Orientation in 3D space using quaternions.

    Attributes
    ----------
    u0 : float
        The scalar part of the quaternion.
    u1 : float
        The first component of the vector part of the quaternion.
    u2 : float
        The second component of the vector part of the quaternion.
    u3 : float
        The third component of the vector part of the quaternion.
    """
    u0: float
    u1: float
    u2: float
    u3: float


class Euler(NamedTuple):
    """
    A class to represent Euler angles.

    Attributes:
        rx (float): Rotation around the x-axis in radians.
        ry (float): Rotation around the y-axis in radians.
        rz (float): Rotation around the z-axis in radians.
    """
    rx: float
    ry: float
    rz: float


class Pos(NamedTuple):
    """
    Pose is a NamedTuple that represents a pose in 3D space.

    Attributes:
        x (float): The x coordinate.
        y (float): The y coordinate.
        z (float): The z coordinate
    """
    x: float
    y: float
    z: float


class Pose(NamedTuple):
    """
    A class to represent a 3D pose with x, y, and z coordinates.

    Attributes:
        pos (Position): The position component of the pose.
        orient (Orientation): The orientation component of the pose.
        euler (Euler): The Euler angles representing the orientation.
    """
    pos: Pos
    orient: Orientation
    euler: Euler
