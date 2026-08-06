#!/bin/bash

cd /media/wise/8908794e-faf6-421c-8a31-a72e82a370e0/bag/251015_fusion_bag1/5_person3
ros2 bag play rosbag2_2025_10_15-16_40_13_0.db3 --topics /lidar_11_201/points /lidar_12_201/points /lidar_13_201/points /merge_201/points /image_rect /tf_static -l
