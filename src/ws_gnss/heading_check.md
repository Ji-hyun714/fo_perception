ros2 topic echo /rover/ubx_nav_rel_pos_ned --field rel_pos_valid


### rel_pos_valid = 0일 때, 아래 두 개 토픽 비교

ros2 topic echo /rover/ubx_nav_rel_pos_ned --field rel_pos_heading

ros2 topic echo /cal_heading
