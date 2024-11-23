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

positional arguments:
  in_path

options:
  -h, --help   show this help message and exit
  --out OUT    output bag path
  --boot BOOT  boot time, default is 0
~~~

#### A note on time:

This tool uses the mavutil-provided `_timestamp` attribute to get message time. The `_timestamp` attribute contains
UNIX Epoch time _iff_ there is a GPS message in the log with a valid time, otherwise it contains time-since-boot.
(Note that the WL UGPS extension does not provide valid time.) In this case, you can set the `--boot` value to
set the boot time in the UNIX Epoch.

TL;DR If your bags seem to start on January 1st, 1970, then you need to provide a `--boot` option.

The best source of information for `--boot` is a [SYSTEM_TIME](https://mavlink.io/en/messages/common.html#SYSTEM_TIME)
message from a tlog file, which contains 2 fields:
* `time_unix_usec` -- this will the Raspberry Pi time, which _should be_ in UNIX Epoch time -- check it!
* `time_boot_ms` -- this is time-since-boot

A good value for `--boot` is `SYSTEM_TIME.time_unix_usec - SYSTEM_TIME.time_boot_ms * 1e3`.

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