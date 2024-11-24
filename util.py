import os

import builtin_interfaces.msg
import rosbag2_py

# Time
# ROS2:         https://github.com/ros2/rcl_interfaces/blob/humble/builtin_interfaces/msg/Time.msg


def default_bag_path(in_path: str):
    dirname, basename = os.path.split(in_path)
    root, _ = os.path.splitext(basename)
    return os.path.join(dirname, root + '_bag')


def time_us_to_ros(time_us: int, stamp: builtin_interfaces.msg.Time):
    """
    Both ArduSub and ROS time are in the UNIX epoch (0 is Jan 1st, 1970 UTC). ArduSub time is microseconds.
    ROS time is split between seconds (int32) and nanoseconds (int32).
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
