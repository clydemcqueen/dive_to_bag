# Generate a ROS2 Bag from ArduSub Logs

These tools can be used to create ROS2 bags from dive logs, including:
* Dataflash (BIN) files
* MAVLink telemetry (tlog) files
* Video (mp4) files

The general process looks like this:
* Examine the logs and select the files to process. For each BIN file, figure out the time-shift value to use
* Generate bags from the logs, one bag per log
* Use `ros2 bag convert` to merge the bags into a single bag

## Tools

### [df_to_bag](df_to_bag.py)

Read a single dataflash log (BIN file) and write sensor messages to a bag. These messages are converted:
* `IMU` to `sensor_msgs::msg::Imu` on topics `/imu0`, `/imu1`, ...
* `MAG` to `sensor_msgs::msg::MagneticField` on topics `/mag0`, `/mag1`, ...

Usage:
~~~
usage: df_to_bag.py [-h] [--out OUT] [--boot BOOT] in_path

Read a dataflash log (BIN file) and write the IMU and MAG messages to a ROS2 bag.

A note on time: while it is possible for TimeUS fields to be UNIX Epoch time, for the vast majority of ArduSub dataflash
logs this field is always time-since-boot. You must provide a time-shift value on the command line to get UNIX Epoch
times in the ROS2 bag. You may be able to get this information from SYSTEM_TIME messages in the tlog file(s).

positional arguments:
  in_path

options:
  -h, --help   show this help message and exit
  --out OUT    output bag path
  --boot BOOT  boot time, default is 0
~~~

### [tlog_to_bag](tlog_to_bag.py)

Read a single telemetry log (tlog file) and write sensor messages to a bag. These messages are converted:
* `GPS_INPUT` to `sensor_msgs::msg::NavSatFix` on topics `/gps0`, `/gps1`, ...
* `VISION_POSITION_DELTA` to `geometry_msgs::msg::TwistStamped` on topic `/dvl`

Usage:
~~~
$ ./tlog_to_bag.py --help
usage: tlog_to_bag.py [-h] [--out OUT] in_path

Read a MAVLink telemetry log (tlog file) and write the GPS and DVL messages to a ROS2 bag.

positional arguments:
  in_path

options:
  -h, --help  show this help message and exit
  --out OUT   output bag path
~~~

### mp4_to_bag (proposed)

Open a video (mp4 file) and write images to a bag.

### dive_to_bag (proposed)

Examine all dive logs and generate a script that will process the files and generate a merged bag.