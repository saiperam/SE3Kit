"""
ROS compatibility layer for SE3Kit modules.

This module provides lightweight imports and utilities for
handling ROS1 vs ROS2 message type differences.
"""

use_geomsg = False

try:
    from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
    import rclpy  # ROS2

    ROS_VERSION = 2
    use_geomsg = True
except ModuleNotFoundError:
    try:
        from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
        import rospy  # ROS1

        ROS_VERSION = 1
        use_geomsg = True
    except ModuleNotFoundError:
        # Not running in a ROS environment at all
        Pose = Point = Quaternion = Vector3 = None
        ROS_VERSION = 0


def get_ros_geometry_msgs():
    """
    Returns the geometry message types (Point, Quaternion, Pose, Vector3)
    depending on which ROS environment is active.

    :return: Tuple of message classes.
    :rtype: (Point, Quaternion, Pose, Vector3)
    """
    return Point, Quaternion, Pose, Vector3
