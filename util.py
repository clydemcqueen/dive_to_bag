import math
import os

import builtin_interfaces.msg
import geometry_msgs.msg
import rclpy.serialization
import rosbag2_py
import tf2_msgs.msg

# Time
# ROS2:         https://github.com/ros2/rcl_interfaces/blob/humble/builtin_interfaces/msg/Time.msg


def default_bag_path(in_path: str):
    dirname, basename = os.path.split(in_path)
    print(f'TEST dirname {dirname}, basename {basename}')
    root, _ = os.path.splitext(basename)
    return os.path.join(dirname, root + '_bag')


def time_s_to_ros(time_s: float, stamp: builtin_interfaces.msg.Time):
    """
    Time in seconds to ROS stamp
    """
    stamp.sec = int(time_s)
    stamp.nanosec = int((time_s - stamp.sec) * 1e9)


def time_ros_to_ns(stamp: builtin_interfaces.msg.Time) -> int:
    """
    Timestamp to nanoseconds
    """
    return int(stamp.sec * 1e9 + stamp.nanosec)


def time_us_to_ros(time_us: int, stamp: builtin_interfaces.msg.Time):
    """
    Time in microseconds (int) to ROS stamp (ArduSub -> ROS)
    """
    stamp.sec = int(time_us / 1e6)
    stamp.nanosec = int((time_us % 1e6) * 1e3)


def create_topic(topic: str, writer: rosbag2_py.SequentialWriter, type: str):
    topic_info = rosbag2_py.TopicMetadata(topic, type, 'cdr')
    writer.create_topic(topic_info)
    print(f'Create topic /{topic}')


def ensure_topic_created(topic: str, topics: list[str], writer: rosbag2_py.SequentialWriter, type: str):
    if topic not in topics:
        create_topic(topic, writer, type)
        topics.append(topic)


def haversine_enu(gps1: tuple[float, float], gps2: tuple[float, float]) -> tuple[float, float]:
    """
    Calculate the difference between two GPS coordinates, return the result in meters
    :param gps1: (latitude, longitude)
    :param gps2: (latitude, longitude)
    :return: (x, y) in meters, where positive x is east and positive y is north
    """
    # Earth radius in meters
    R = 6371000

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(gps1[0])
    lon1 = math.radians(gps1[1])
    lat2 = math.radians(gps2[0])
    lon2 = math.radians(gps2[1])

    # Difference in radians
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula to calculate the great-circle distance between two points
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    # Bearing between the two points
    bearing = math.atan2(math.sin(dlon) * math.cos(lat2),
                         math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

    # Return the difference in cartesian coordinates
    return distance * math.sin(bearing), distance * math.cos(bearing)


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def write_transform_message(writer: rosbag2_py.SequentialWriter, transform: geometry_msgs.msg.TransformStamped):
    tf_message = tf2_msgs.msg.TFMessage()
    tf_message.transforms.append(transform)
    writer.write('tf', rclpy.serialization.serialize_message(tf_message), time_ros_to_ns(transform.header.stamp))
