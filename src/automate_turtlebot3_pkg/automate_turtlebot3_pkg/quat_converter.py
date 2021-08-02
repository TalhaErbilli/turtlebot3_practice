from math import asin, atan2
import numpy as np

def quaternion2rpy(q):
    """
    Roll-pitch-yaw angles of a given quaternion.    Parameters
    ----------
    q : numpy.ndarray or list
        Quaternion in [w x y z] format.    Returns
    -------
    rpy : numpy.ndarray
        Array of yaw-pitch-roll angles, in radian.    Examples
    --------
    >>> from skrobot.coordinates.math import quaternion2rpy
    >>> quaternion2rpy([1, 0, 0, 0])
    (array([ 0., -0.,  0.]), array([3.14159265, 3.14159265, 3.14159265]))
    >>> quaternion2rpy([0, 1, 0, 0])
    (array([ 0.        , -0.        ,  3.14159265]),
     array([3.14159265, 3.14159265, 0.        ]))    """
    roll = atan2(
        2 * q[2] * q[3] + 2 * q[0] * q[1],
        q[3] ** 2 - q[2] ** 2 - q[1] ** 2 + q[0] ** 2,
    )
    pitch = -asin(2 * q[1] * q[3] - 2 * q[0] * q[2])
    yaw = atan2(
        2 * q[1] * q[2] + 2 * q[0] * q[3],
        q[1] ** 2 + q[0] ** 2 - q[3] ** 2 - q[2] ** 2,
    )
    rpy = np.array([yaw, pitch, roll])
    return rpy, np.pi - rpy
    
def rpy2quaternion(rpy):
    """
    Return Quaternion from yaw-pitch-roll angles.    Parameters
    ----------
    rpy : numpy.ndarray or list
        Vector of yaw-pitch-roll angles in radian.    Returns
    -------
    quat : numpy.ndarray
        Quaternion in [w x y z] format.    Examples
    --------
    >>> import numpy as np
    >>> from skrobot.coordinates.math import rpy2quaternion
    >>> rpy2quaternion([0, 0, 0])
    array([1., 0., 0., 0.])
    >>> yaw = np.pi / 3.0
    >>> rpy2quaternion([yaw, 0, 0])
    array([0.8660254, 0.       , 0.       , 0.5      ])
    >>> rpy2quaternion([np.pi * 2 - yaw, 0, 0])
    array([-0.8660254, -0.       ,  0.       ,  0.5      ])    """
    yaw, pitch, roll = rpy
    cr, cp, cy = np.cos(roll / 2.0), np.cos(pitch / 2.0), np.cos(yaw / 2.0)
    sr, sp, sy = np.sin(roll / 2.0), np.sin(pitch / 2.0), np.sin(yaw / 2.0)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            -cr * sp * sy + cp * cy * sr,
            cr * cy * sp + sr * cp * sy,
            cr * cp * sy - sr * cy * sp,
        ]
    )