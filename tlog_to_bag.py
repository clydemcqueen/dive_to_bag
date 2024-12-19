#!/usr/bin/env python3

"""
Read a MAVLink telemetry log (tlog file) and write the GPS and DVL messages to a ROS2 bag.
"""

import argparse

import geometry_msgs.msg
import rclpy.serialization
import rosbag2_py
import sensor_msgs.msg
import transforms3d
from pymavlink import mavutil

import util

# GPS_INPUT
# MAVLink:      https://mavlink.io/en/messages/common.html#GPS_INPUT
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatStatus.msg

# VISION_POSITION_DELTA
# MAVLink:      https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
# ROS2:         https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TwistStamped.msg

# TODO also write GLOBAL_POSITION_INT and LOCAL_POSITION_NED as poses
# TODO get the origin from GLOBAL_ORIGIN
# TODO origin should include all 6DoF


def tlog_to_bag(in_path: str, out_path: str, origin_str: str | None, start_s: float):
    if start_s > 0:
        print(f'Skipping ahead to {start_s}')

    # Get the map frame origin
    if origin_str is None:
        origin = None
    else:
        strs = origin_str.split(',')
        origin = (int(strs[0]), int(strs[1]))
        print(f'Origin was provided: {origin}')

    # Create bag, open for writing (append is not supported)
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=out_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # Logs may contain information for multiple GPS sensors, but only 1 DVL sensor
    util.create_topic('dvl_meas', writer, 'geometry_msgs/msg/TwistStamped')
    util.create_topic('dvl_pose', writer, 'geometry_msgs/msg/PoseStamped')
    gps_topics = []

    num_gps_msgs = 0
    num_dvl_msgs = 0

    # The ROS2 message that corresponds to GPS_INPUT is NavSatFix
    ros_gps_msg = sensor_msgs.msg.NavSatFix()

    # The DVL produces a velocity vector, so we'll use TwistStamped
    ros_dvl_msg = geometry_msgs.msg.TwistStamped()

    # Write a GPS pose, which is gps_meas - gps_meas0
    ros_gps_pose_msg = geometry_msgs.msg.PoseStamped()
    ros_gps_pose_msg.header.frame_id = 'map'

    # Write a DVL pose, which is a simple accumulation of the deltas
    ros_dvl_pose_msg = geometry_msgs.msg.PoseStamped()
    ros_dvl_pose_msg.header.frame_id = 'map'
    ros_dvl_angle = geometry_msgs.msg.Vector3()

    # Open tlog file for reading
    reader = mavutil.mavlink_connection(in_path)

    while True:
        mav_msg = reader.recv_match(type=[
            # 'ATTITUDE',
            # 'GLOBAL_POSITION_INT',
            # 'GPS_GLOBAL_ORIGIN',
            'GPS_INPUT',
            # 'LOCAL_POSITION_NED',
            'VISION_POSITION_DELTA',
        ], blocking=False)
        if mav_msg is None:
            break

        # Ignore GPS_INPUT.time_usec and VISION_POSITION_DELTA.time_usec, they are always 0 for WL devices
        # _timestamp is laptop time if QGC-generated, or Pi time if mavlink_router-generated
        time_us = int(getattr(mav_msg, '_timestamp', 0) * 1e6)

        # Skip ahead if requested
        if start_s > 0 and time_us < start_s * 1e6:
            continue

        msg_type = mav_msg.get_type()
        if msg_type == "GPS_INPUT":
            topic = f'gps{mav_msg.gps_id}'
            if topic not in gps_topics:
                util.create_topic(topic + '_meas', writer, 'sensor_msgs/msg/NavSatFix')
                util.create_topic(topic + '_pose', writer, 'geometry_msgs/msg/PoseStamped')
                gps_topics.append(topic)

            # Write the measurement
            util.time_us_to_ros(time_us, ros_gps_msg.header.stamp)
            ros_gps_msg.header.frame_id = topic
            # Leave status as 0's
            ros_gps_msg.latitude = mav_msg.lat / 1e7
            ros_gps_msg.longitude = mav_msg.lon / 1e7
            ros_gps_msg.altitude = mav_msg.alt / 1e3  # mm -> m
            ros_gps_msg.position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_UNKNOWN
            writer.write(topic + '_meas', rclpy.serialization.serialize_message(ros_gps_msg), int(time_us * 1e3))

            # Don't set the origin to (0, 0)
            if ros_gps_msg.latitude != 0.0 and ros_gps_msg.longitude != 0.0:
                # Set the map origin, it not set already
                if origin is None:
                    origin = (ros_gps_msg.latitude, ros_gps_msg.longitude)
                    print(f'Origin is the first GPS measurement: {origin}')

                # Write the pose (gps - origin)
                util.time_us_to_ros(time_us, ros_gps_pose_msg.header.stamp)
                rov_f_map = util.haversine_enu(origin, (ros_gps_msg.latitude, ros_gps_msg.longitude))
                ros_gps_pose_msg.pose.position.x = rov_f_map[0]
                ros_gps_pose_msg.pose.position.y = rov_f_map[1]
                writer.write(topic + '_pose', rclpy.serialization.serialize_message(ros_gps_pose_msg), int(time_us * 1e3))

            num_gps_msgs += 1

        elif msg_type == "VISION_POSITION_DELTA":
            # Write the measurement
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
            writer.write('dvl_meas', rclpy.serialization.serialize_message(ros_dvl_msg), int(time_us * 1e3))

            # Accumulate the deltas and write a pose (FRD to FLU)
            util.time_us_to_ros(time_us, ros_dvl_pose_msg.header.stamp)
            ros_dvl_pose_msg.pose.position.x += mav_msg.position_delta[0]
            ros_dvl_pose_msg.pose.position.y += -mav_msg.position_delta[1]
            ros_dvl_pose_msg.pose.position.z += -mav_msg.position_delta[2]
            ros_dvl_angle.x = util.norm_angle(ros_dvl_angle.x + mav_msg.angle_delta[0])
            ros_dvl_angle.y = util.norm_angle(ros_dvl_angle.y - mav_msg.angle_delta[1])
            ros_dvl_angle.z = util.norm_angle(ros_dvl_angle.z - mav_msg.angle_delta[2])
            ros_dvl_quat = transforms3d.euler.euler2quat(ros_dvl_angle.x, ros_dvl_angle.y, ros_dvl_angle.z)
            # transforms3d quaternion is [w, x, y, z]
            ros_dvl_pose_msg.pose.orientation.w = ros_dvl_quat[0]
            ros_dvl_pose_msg.pose.orientation.x = ros_dvl_quat[1]
            ros_dvl_pose_msg.pose.orientation.y = ros_dvl_quat[2]
            ros_dvl_pose_msg.pose.orientation.z = ros_dvl_quat[3]
            writer.write('dvl_pose', rclpy.serialization.serialize_message(ros_dvl_pose_msg), int(time_us * 1e3))

            num_dvl_msgs += 1

    print(f'Wrote {num_gps_msgs} GPS messages and {num_dvl_msgs} DVL messages')


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    parser.add_argument('--out', default=None, help='output bag path')
    parser.add_argument('--origin', default=None, help='map frame origin, form: lat,lon')
    parser.add_argument('--start', type=float, default=0.0, help='skip to start time')
    parser.add_argument('in_path')
    args = parser.parse_args()
    out_path = util.default_bag_path(args.in_path) if args.out is None else args.out
    tlog_to_bag(args.in_path, out_path, args.origin, args.start)


if __name__ == '__main__':
    main()
