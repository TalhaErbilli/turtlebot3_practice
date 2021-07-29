#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


def from_tf2_msg(tf_msg):
    p = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    position = [p.x, p.y, p.z]
    rotation = [r.x, r.y, r.z, r.w]
    return (position, rotation)


def quaternion_to_rotation(quaternion: list):
    r = Rotation.from_quat(quaternion)
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = r.as_matrix()
    return mat


def tf2_msg_to_rotation(tf_msg):
    p, q = from_tf2_msg(tf_msg)
    r = quaternion_to_rotation(quaternion=q)
    r[:3, 3] = p
    return r


class TFUtils(object):
    def __init__(self, node, spin_thread: bool = False):
        self._tf_buffer = Buffer(node=node)
        self._listener = TransformListener(
            buffer=self._tf_buffer, node=node, spin_thread=spin_thread
        )
        self._broadcast = TransformBroadcaster(node=node)
        self._node = node

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        convert: bool = True,
        when=None,
        duration=rclpy.duration.Duration(seconds=1)
    ) -> TransformStamped:
        """
        Lookup the transform matrix from target frame to source frame,
        which will be used to transform the point in source frame to target frame.
        """
        when = rclpy.time.Time() if when is None else when
        if not self._tf_buffer.can_transform(
                target_frame, source_frame, when, duration
        ):
            raise LookupException(
                f'Failed to find transform {source_frame} to {target_frame} {when}'
            )
        future = self._tf_buffer.wait_for_transform_async(
            target_frame, source_frame, when
        )
        assert future
        transform = self._tf_buffer.lookup_transform(
            target_frame, source_frame, when, duration
        )
        return tf2_msg_to_rotation(transform) if convert else transform

    def lookup_transform_async(
        self,
        target_frame: str,
        source_frame: str,
        convert: bool = True,
        when=rclpy.time.Time(),
    ) -> TransformStamped:
        transform = self._tf_buffer.lookup_transform_async(
            target_frame, source_frame, when
        )
        return tf2_msg_to_rotation(transform) if convert else transform

    def send_transform(self, transform: TransformStamped) -> None:
        self._broadcast.sendTransform(transform=transform)
