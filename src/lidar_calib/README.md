## dependency

- pip3 install ros2-numpy


## output 위치 바꾸기

multi_lidar_calibrator/multi_lidar_calibrator.py 에서 output 위치 바꾸기

self.output_dir = "/home/wise/adsp_perception_lidar/src/lidar_calib/src/Multi_LiCa/output/" 로 되어 있음


## 실행 과정 (seyond)

ros2 launch frameid_change run_change.launch.py

또는 ros2 launch frameid_change frameid_ransac.launch.py 


export PYTHONPATH=$PYTHONPATH:/home/wise/adsp_perception_lidar/src/lidar_calib/src/TEASER-plusplus/python

ros2 launch multi_lidar_calibrator calibration.launch.py


ros2 run lidar_tf iv_points_tf

- frame_id: seyond_left

- topic: /iv_points_left_change, /iv_points_right_change_applyTF



## 실행 과정 (velo)

export PYTHONPATH=$PYTHONPATH:/home/wise/fo_perception/src/lidar_calib/src/TEASER-plusplus/python
python3 -c "import teaserpp_python; print('teaserpp_python OK')"

### raw point -> nonground
<!-- ros2 launch linefit_ground_segmentation_ros fl_segmentation.launch.py
ros2 launch linefit_ground_segmentation_ros fr_segmentation.launch.py
ros2 launch linefit_ground_segmentation_ros r_segmentation.launch.py -->
ros2 launch velo16_ground multi_ground_removal.launch.py

### nonground -> TFnonground
python ~/fo_perception/src/Documents/lidar_frame_change/lidar_frame_change_11_xyz.py
python ~/fo_perception/src/Documents/lidar_frame_change/lidar_frame_change_12_xyz.py
python ~/fo_perception/src/Documents/lidar_frame_change/lidar_frame_change_13_xyz.py

### TFnonground -> velo11 / velo12 / velo13
ros2 launch frameid_change frameid_ransac.launch.py

lidar_calib/src/Multi_LiCa/config/params_velo.yaml 수정
- lidar_topics, velo11, velo12, velo13, etc.

ros2 launch multi_lidar_calibrator velo_calibration.launch.py

### result
python ~/fo_perception/src/Documents/lidar_frame_change/result_12.py


## TEASER-plusplus 빌드 & 실행

mkdir build && cd build
cmake .. \
-DBUILD_PYTHON_BINDINGS=ON \
-DPYTHON_EXECUTABLE=$(which python3)
make -j$(nproc)

export PYTHONPATH=$PYTHONPATH:/home/wise/fo_perception/src/lidar_calib/src/TEASER-plusplus/python

python3 -c "import teaserpp_python; print('teaserpp_python OK')"

ros2 launch multi_lidar_calibrator calibration.launch.py
