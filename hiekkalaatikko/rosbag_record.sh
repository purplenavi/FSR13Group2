#!/bin/sh
rosbag record camera/depth_registered/image_raw camera/depth_registered/camera_info camera/rgb/image_raw camera/rgb/camera_info tf map RosAria/pose RosAria/cmd map_metadata slam_gmapping/entropy -O bagi
