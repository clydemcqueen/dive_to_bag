#!/bin/bash

# Example script: create wl_ugps_csv_bag and tlog_bag, merge them, and play the merged bag through rviz2.

TOOL_PATH=~/projects/dive_to_bag
LOG_PATH=~/dive_logs/seaq_logs/2024_12_05_nmea

# Work in the /tmp dir
cd /tmp

echo "---------------"
echo "Delete old bags:"
echo "---------------"
rm -rf merged_bag tlog_bag wl_ugps_csv_bag

echo "---------------"
echo "Create wl_ugps_csv_bag:"
echo "---------------"
${TOOL_PATH}/wl_ugps_csv_to_bag.py ${LOG_PATH} 2024-12-05_22-12-34_nmea_GGA.csv 2024-12-05_22-12-34_nmea_HDT.csv 2024-12-05_22-12-34_g2_acoustic.csv

echo "---------------"
echo "Create tlog_bag:"
echo "---------------"
${TOOL_PATH}/tlog_to_bag.py --out /tmp/tlog_bag --tf --lat0 48.684828081666666 --lon0 -123.19206343 ${LOG_PATH}/logs/00102-2024-12-05_22-03-12.tlog

echo "---------------"
echo "Merge bags:"
echo "---------------"
ros2 bag convert -i tlog_bag/ -i wl_ugps_csv_bag/ -o ${TOOL_PATH}/merge.yaml

echo "---------------"
echo "In terminal 1:"
echo "---------------"
echo "rviz2"
echo "---------------"
echo "In terminal 2:"
echo "---------------"
echo "ros2 param set /rviz use_sim_time true"
echo "ros2 run tf2_ros static_transform_publisher 1.57 -1.273 -0.377 0 0 0 vessel antenna"
echo "---------------"
echo "In terminal 3:"
echo "---------------"
echo "ros2 bag play --start-offset 900 --clock 60 /tmp/merged_bag"
echo "---------------"
