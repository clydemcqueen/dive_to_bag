#!/usr/bin/env python3

"""
Read a MAVLink telemetry log (tlog file) and write the GPS and DVL messages to a ROS2 bag.
"""

import argparse

import geometry_msgs.msg
import rclpy.serialization
import rosbag2_py
import sensor_msgs.msg
from pymavlink import mavutil

import util

# GPS_INPUT
# MAVLink:      https://mavlink.io/en/messages/common.html#GPS_INPUT
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatStatus.msg

# VISION_POSITION_DELTA
# MAVLink:      https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TwistStamped.msg


def tlog_to_bag(in_path: str, out_path: str):
    # Create bag, open for writing (append is not supported)
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=out_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # Logs may contain information for multiple GPS sensors, but only 1 DVL sensor
    util.create_topic('dvl', writer, 'geometry_msgs/msg/TwistStamped')
    gps_topics = []

    num_gps_msgs = 0
    num_dvl_msgs = 0

    # Avoid creating zillions of objects
    ros_gps_msg = sensor_msgs.msg.NavSatFix()
    ros_dvl_msg = geometry_msgs.msg.TwistStamped()

    # Open tlog file for reading
    reader = mavutil.mavlink_connection(in_path)

    while True:
        mav_msg = reader.recv_match(type=['GPS_INPUT', 'VISION_POSITION_DELTA'], blocking=False)
        if mav_msg is None:
            break

        # Ignore GPS_INPUT.time_usec and VISION_POSITION_DELTA.time_usec, they are always 0 for WL devices
        # _timestamp is laptop time if QGC-generated, or Pi time if mavlink_router-generated
        time_us = int(getattr(mav_msg, '_timestamp', 0) * 1e6)

        if mav_msg.get_type() == "GPS_INPUT":
            topic = f'gps{mav_msg.gps_id}'
            util.ensure_topic_created(topic, gps_topics, writer, 'sensor_msgs/msg/NavSatFix')

            util.time_us_to_ros(time_us, ros_gps_msg.header.stamp)
            ros_gps_msg.header.frame_id = topic
            # Leave status as 0's
            ros_gps_msg.latitude = mav_msg.lat / 1e7
            ros_gps_msg.longitude = mav_msg.lon / 1e7
            ros_gps_msg.altitude = mav_msg.alt / 1e3  # mm -> m
            ros_gps_msg.position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_UNKNOWN

            writer.write(topic, rclpy.serialization.serialize_message(ros_gps_msg), int(time_us * 1e3))
            num_gps_msgs += 1

        else:
            util.time_us_to_ros(time_us, ros_dvl_msg.header.stamp)
            ros_dvl_msg.header.frame_id = 'dvl'
            # Convert FRD (forward-right-down) to FLU (forward-left-up)
            time_delta_s = mav_msg.time_delta_usec / 1e6
            ros_dvl_msg.twist.linear.x = mav_msg.position_delta[0] / time_delta_s
            ros_dvl_msg.twist.linear.y = -mav_msg.position_delta[1] / time_delta_s
            ros_dvl_msg.twist.linear.z = -mav_msg.position_delta[2] / time_delta_s
            ros_dvl_msg.twist.angular.x = mav_msg.angle_delta[0] / time_delta_s
            ros_dvl_msg.twist.angular.y = -mav_msg.angle_delta[1] / time_delta_s
            ros_dvl_msg.twist.angular.z = -mav_msg.angle_delta[2] / time_delta_s

            writer.write('dvl', rclpy.serialization.serialize_message(ros_dvl_msg), int(time_us * 1e3))
            num_dvl_msgs += 1

    print(f'Wrote {num_gps_msgs} GPS messages and {num_dvl_msgs} DVL messages')


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    parser.add_argument('--out', default=None, help='output bag path')
    parser.add_argument('in_path')
    args = parser.parse_args()
    out_path = util.default_bag_path(args.in_path) if args.out is None else args.out
    tlog_to_bag(args.in_path, out_path)


if __name__ == '__main__':
    main()
