#!/usr/bin/env python3

"""
Read a dataflash log (BIN file) and write the IMU and MAG messages to a ROS2 bag.
"""

import argparse

import rclpy.serialization
import rclpy.time
import rosbag2_py
import sensor_msgs.msg
from pymavlink import mavutil

import util

# IMU
# ArduPilot:    https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/LogStructure.h
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg

# MAG
# ArduPilot:    https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/MagneticField.msg


def df_to_bag(in_path: str, out_path: str, time_shift_s: float):
    time_shift_us = int(time_shift_s * 1e6)

    # Create bag, open for writing (append is not supported)
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=out_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # Logs may contain information for multiple IMUs and MAGs
    imu_topics = []
    mag_topics = []

    num_imu_msgs = 0
    num_mag_msgs = 0

    # Avoid creating zillions of objects
    ros_imu_msg = sensor_msgs.msg.Imu()
    ros_mag_msg = sensor_msgs.msg.MagneticField()

    # Open BIN file for reading
    reader = mavutil.mavlink_connection(in_path)

    while True:
        df_msg = reader.recv_match(type=['IMU', 'MAG'], blocking=False)
        if df_msg is None:
            break

        # Use the _timestamp attribute:
        # -- if there is a GPS source with valid time, then mavlink will fill in _timestamp with UNIX Epoch time
        # -- otherwise this will be the same as df_msg.TimeUS, but in seconds
        time_us = int(getattr(df_msg, '_timestamp', 0) * 1e6) + time_shift_us

        if df_msg.get_type() == 'IMU':
            topic = f'imu{df_msg.I}'
            util.ensure_topic_created(topic, imu_topics, writer, 'sensor_msgs/msg/Imu')

            util.time_us_to_ros(time_us, ros_imu_msg.header.stamp)
            ros_imu_msg.header.frame_id = topic
            ros_imu_msg.angular_velocity.x = df_msg.GyrX
            ros_imu_msg.angular_velocity.y = df_msg.GyrY
            ros_imu_msg.angular_velocity.z = df_msg.GyrZ
            ros_imu_msg.linear_acceleration.x = df_msg.AccX
            ros_imu_msg.linear_acceleration.y = df_msg.AccY
            ros_imu_msg.linear_acceleration.z = df_msg.AccZ
            ros_imu_msg.orientation_covariance[0] = -1  # No orientation info

            writer.write(topic, rclpy.serialization.serialize_message(ros_imu_msg), int(time_us * 1e3))
            num_imu_msgs += 1

        else:
            topic = f'mag{df_msg.I}'
            util.ensure_topic_created(topic, mag_topics, writer, 'sensor_msgs/msg/MagneticField')

            util.time_us_to_ros(time_us, ros_mag_msg.header.stamp)
            ros_mag_msg.header.frame_id = topic
            ros_mag_msg.magnetic_field.x = float(df_msg.MagX)  # TODO check units
            ros_mag_msg.magnetic_field.y = float(df_msg.MagY)
            ros_mag_msg.magnetic_field.z = float(df_msg.MagZ)

            writer.write(topic, rclpy.serialization.serialize_message(ros_mag_msg), int(time_us * 1e3))
            num_mag_msgs += 1

    print(f'Wrote {num_imu_msgs} IMU messages and {num_mag_msgs} MAG messages')


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    parser.add_argument('--out', default=None, help='output bag path')
    parser.add_argument('--boot', type=float, default=0.0, help='boot time, default is 0')
    parser.add_argument('in_path')
    args = parser.parse_args()
    out_path = util.default_bag_path(args.in_path) if args.out is None else args.out
    df_to_bag(args.in_path, out_path, args.boot)


if __name__ == '__main__':
    main()
