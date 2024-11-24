#!/usr/bin/env python3

"""
Use OpenCV to open a video file and write image messages to a ROS2 bag.
"""

import argparse

import cv2
import rclpy.serialization
import rclpy.time
import rosbag2_py
import sensor_msgs.msg

import util


def video_to_bag(in_path: str, out_path: str, time_shift_s: float, topic: str, frame_id: str,
                 width: int|None, height: int|None, mono: bool, max_frames: int):
    reader = cv2.VideoCapture(in_path)
    if not reader.isOpened():
        raise IOError(f"Error opening video file: {in_path}")

    # read() might return BGR or YUV, and there isn't a getter for colorspace (?!)
    # Force conversion to RGB
    reader.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)

    if mono:
        print('Convert to monochrome')

    scale = width is not None or height is not None
    if width is not None and height is not None:
        # User specified both width and height
        print(f'Scale to ({width}, {height})')

    # Create the bag and the topic
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=out_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)
    util.create_topic(topic, writer, 'sensor_msgs/msg/Image')

    image_msg = sensor_msgs.msg.Image()
    time_shift_us = int(time_shift_s * 1e6)
    num_frames = 0

    while reader.isOpened():
        if max_frames is not None and num_frames >= max_frames:
            print(f'Hit {max_frames} frames, stopping')
            break

        ret, frame = reader.read()
        if not ret:
            print('Hit end of file, stopping')
            break

        if scale:
            # Handle cases where the user specified width or height, but not both
            # OpenCV uses Numpy, so frame.shape is (height, width)
            if height is None:
                height = int(frame.shape[0] / frame.shape[1] * width)
                print(f'Scale to ({width}, {height})')

            if width is None:
                width = int(frame.shape[1] / frame.shape[0] * height)
                print(f'Scale to ({width}, {height})')

            # But resize wants (width, height)
            # Assume we're scaling down, use a different interpolation if we're scaling up
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        if mono:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Build the image message
        time_us = int(reader.get(cv2.CAP_PROP_POS_MSEC) * 1e3) + time_shift_us
        util.time_us_to_ros(time_us, image_msg.header.stamp)
        image_msg.header.frame_id = frame_id
        image_msg.width = frame.shape[1]
        image_msg.height = frame.shape[0]
        image_msg.encoding = "mono8" if mono else "rgb8"
        image_msg.data = frame.tobytes()
        image_msg.step = image_msg.width if mono else image_msg.width * 3

        # Serialize and write
        writer.write(topic, rclpy.serialization.serialize_message(image_msg), int(time_us * 1e3))
        num_frames += 1

    reader.release()
    print(f'Wrote {num_frames} images')


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    parser.add_argument('--out', default=None, help='output bag path')
    parser.add_argument('--start', type=float, default=0.0, help='video start time, default is 0')
    parser.add_argument('--topic', type=str, default='image_raw', help='topic, default is "image_raw"')
    parser.add_argument('--frame_id', type=str, default='camera_frame', help='camera frame_id, default is "camera_frame"')
    parser.add_argument('--width', type=int, default=None, help='scale width')
    parser.add_argument('--height', type=int, default=None, help='scale height')
    parser.add_argument('--mono', action='store_true', help='convert to monochrome')
    parser.add_argument('--max', type=int, default=None, help='maximum number of frames to read')
    parser.add_argument('in_path')
    args = parser.parse_args()
    out_path = util.default_bag_path(args.in_path) if args.out is None else args.out
    video_to_bag(args.in_path, out_path, args.start, args.topic, args.frame_id, args.width, args.height, args.mono, args.max)


if __name__ == '__main__':
    main()
