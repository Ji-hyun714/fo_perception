source ~/fo_perception/install/setup.bash


1. bag -> excel 데이터 변환   (gnssbag_to_excel.py)

python3 ~/fo_perception/src/ws_gnss/map_visualizer/gnssbag_to_excel.py ~/zedf9p_bag/withmsc_0729/rosbag2_2026_07_28-18_30_47


2. lat, long 표시

python3 ~/fo_perception/src/ws_gnss/map_visualizer/plot_points.py ~/fo_perception/src/ws_gnss/map_visualizer/bag_log/rosbag2_2026_07_28-18_30_47_gnss.xlsx


3. heading 표시

python3 ~/fo_perception/src/ws_gnss/map_visualizer/plot_heading.py ~/fo_perception/src/ws_gnss/map_visualizer/bag_log/rosbag2_2026_07_28-18_30_47_gnss.xlsx


<!---------------------------------------------------------->
python3 ~/fo_perception/src/ws_gnss/map_visualizer/gnssbag_to_excel.py ~/zedf9p_bag/

python3 ~/fo_perception/src/ws_gnss/map_visualizer/plot_points.py ~/fo_perception/src/ws_gnss/map_visualizer/bag_log/

python3 ~/fo_perception/src/ws_gnss/map_visualizer/plot_heading.py ~/fo_perception/src/ws_gnss/map_visualizer/bag_log/

