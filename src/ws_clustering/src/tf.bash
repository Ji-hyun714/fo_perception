#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 2.2 --y 1.0 --z 0.0 \
  --yaw 1.1698 --pitch 0.0 --roll 0.0 \
  --frame-id vehicle \
  --child-frame-id velo1 &

ros2 run tf2_ros static_transform_publisher \
  --x 2.2 --y -1.0 --z 0.0 \
  --yaw -1.21 --pitch 0.0 --roll 0.0 \
  --frame-id vehicle \
  --child-frame-id velo2 &

ros2 run tf2_ros static_transform_publisher \
  --x -2.0 --y 0.3 --z 0.0 \
  --yaw -2.6 --pitch 0.0 --roll 0.0 \
  --frame-id vehicle \
  --child-frame-id velo3