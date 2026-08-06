std_msgs/Header header

uint32 can_id
uint8  len         # CAN payload length
uint8  pad         # padding (for alignment)
uint8  res0        # reserved
uint8  len8_dlc    # DLC for 8-byte payload length (optional, 9~15)
uint8[8] data      # CAN payload bytes (up to 8)