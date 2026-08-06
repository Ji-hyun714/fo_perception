uint8 TRACK_STATUS_NEW=0
uint8 TRACK_STATUS_TRACKED=1
uint8 TRACK_STATUS_LOST=2
uint8 TRACK_STATUS_REMOVED=3

int32 class_id     # bbox class id
float32 score      # confidence
float32 x          # bbox xmin
float32 y          # bbox ymin
float32 w          # bbox width
float32 h          # bbox height

bool validity      # object validity defined by score, 0: valid 1: invalid
float32 x_m        # object relative x, vehicle coord system
float32 y_m        # object relative y, vehicle coord system
float32 range_m    # object range, polar coord system
float32 angle_deg  # object angle, polar coord system

uint32 track_id    
uint32 track_age   
uint8 track_status 
