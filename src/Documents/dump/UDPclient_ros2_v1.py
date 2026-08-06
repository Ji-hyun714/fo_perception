import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from cam2_msgs.msg import AggregatedCam2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np

# Server IP and Port Setting
server_ip = "192.168.1.78"
server_port = 3000
server_addr_port = (server_ip, server_port)

# Set UDP Socket
udp_client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# UDP Packet Length
PACKET_SIZE = 5360
index = 0

# 1. Float >> Bit Transform 
def float_to_10bit_signed(value, scale):
    int_value = int(value / scale)
    if int_value < -512:
        int_value = -512
    elif int_value > 511:
        int_value = 511
    if int_value < 0:
        int_value = (1 << 10) + int_value
    return int_value

def float_to_11bit_unsigned(value, scale):
    int_value = int(value / scale)
    if int_value < 0:
        int_value = 0
    elif int_value > 2047:
        int_value = 2047
    return int_value

def float_to_11bit_signed(value, scale):
    int_value = int(value / scale)
    if int_value < -(1 << 10):
        int_value = -(1 << 10)
    elif int_value > (1 << 10) - 1:
        int_value = (1 << 10) - 1
    if int_value < 0:
        int_value = (1 << 11) + int_value
    return int_value

def float_to_14bit_unsigned(value, scale):
    int_value = int(value / scale)
    if int_value < 0:
        int_value = 0
    elif int_value > (1 << 14) - 1:
        int_value = (1 << 14) - 1
    return int_value

def float_to_14bit_signed(value, scale):
    int_value = int(value / scale)
    if int_value < -(1 << 13):
        int_value = -(1 << 13)
    elif int_value > (1 << 13) - 1:
        int_value = (1 << 13) - 1
    if int_value < 0:
        int_value = (1 << 14) + int_value
    return int_value

def parse_data(data):
    """ Key:Value Data to Dict """
    data_dict = {}
    pairs = data.split(';')
    for pair in pairs:
        key, value = pair.split(':')
        data_dict[key] = float(value)
    return data_dict

# Bit to Packet
def set_bits(packet, value, start_bit, bit_length):
    for i in range(bit_length):
        bit_value = (value >> (bit_length - 1 - i)) & 1  
        byte_index = (start_bit + i) // 8 
        bit_index = (start_bit + i) % 8    
        packet[byte_index] |= (bit_value << (7 - bit_index)) 

# 2. ROS2 message Subscribe Term
class UDPClientNode(Node):
    def __init__(self):
        super().__init__('udp_client_node')

        # Cam
        self.cam_sub = self.create_subscription(AggregatedCam2, '/cam2_msgs', self.callback_cam, 10)

        # LiDAR
        # TD
        self.lidar_11_sub = Subscriber(self, Detection3DArray, '/lidar_11_201/TD')
        self.lidar_12_sub = Subscriber(self, Detection3DArray, '/lidar_12_201/TD')
        self.lidar_13_sub = Subscriber(self, Detection3DArray, '/lidar_13_201/TD')
        self.td = ApproximateTimeSynchronizer([self.lidar_11_sub, self.lidar_12_sub, self.lidar_13_sub], 10, 0.5, allow_headerless=True)
        self.td.registerCallback(self.callback_lidar_TD)
        # CD
        self.lidar_left_sub = Subscriber(self, String, '/left_lane_coefficients')
        self.lidar_right_sub = Subscriber(self, String, '/right_lane_coefficients')
        self.cd = ApproximateTimeSynchronizer([self.lidar_left_sub, self.lidar_right_sub], 10, 0.1, allow_headerless=True)
        self.cd.registerCallback(self.callback_lidar_CD)

        # 20ms Period UDP Timer
        self.timer = self.create_timer(0.02, self.send_udp_messages)

        # Set Value
        self.variables = {}

    # 3. Callback about ROS message
    def callback_cam(self, data):
        # Cam Data
        # LD
        # LD Left Lane A
        self.variables['Cam2LeftLaneA_LaneMarkType'] = data.ld.cam2_left_lane_a.lane_mark_type
        self.variables['Cam2LeftLaneA_LaneMarkQuality'] = data.ld.cam2_left_lane_a.lane_mark_quality
        self.variables['Cam2LeftLaneA_LaneMarkWidth'] = data.ld.cam2_left_lane_a.lane_mark_width
        self.variables['Cam2LeftLaneA_LaneMarkModelA'] = data.ld.cam2_left_lane_a.lane_mark_model_a
        self.variables['Cam2LeftLaneA_LaneMarkPosition'] = data.ld.cam2_left_lane_a.lane_mark_position
        # LD Right Lane A
        self.variables['Cam2RightLaneA_LaneMarkType'] = data.ld.cam2_right_lane_a.lane_mark_type
        self.variables['Cam2RightLaneA_LaneMarkQuality'] = data.ld.cam2_right_lane_a.lane_mark_quality
        self.variables['Cam2RightLaneA_LaneMarkWidth'] = data.ld.cam2_right_lane_a.lane_mark_width
        self.variables['Cam2RightLaneA_LaneMarkModelA'] = data.ld.cam2_right_lane_a.lane_mark_model_a
        self.variables['Cam2RightLaneA_LaneMarkPosition'] = data.ld.cam2_right_lane_a.lane_mark_position
        # LD Left Lane B
        self.variables['Cam2LeftLaneB_LaneMarkHeadingAngle'] = data.ld.cam2_left_lane_b.lane_mark_heading_angle
        self.variables['Cam2LeftLaneB_LaneMarkModelViewRange'] = data.ld.cam2_left_lane_b.lane_mark_model_view_range
        self.variables['Cam2LeftLaneB_LaneMarkModelViewRangeAvailability'] = data.ld.cam2_left_lane_b.lane_mark_model_view_range_availability
        self.variables['Cam2LeftLaneB_LaneMarkModelDa'] = data.ld.cam2_left_lane_b.lane_mark_model_da
        # LD Right Lane B
        self.variables['Cam2RightLaneB_LaneMarkHeadingAngle'] = data.ld.cam2_right_lane_b.lane_mark_heading_angle
        self.variables['Cam2RightLaneB_LaneMarkModelViewRange'] = data.ld.cam2_right_lane_b.lane_mark_model_view_range
        self.variables['Cam2RightLaneB_LaneMarkModelViewRangeAvailability'] = data.ld.cam2_right_lane_b.lane_mark_model_view_range_availability
        self.variables['Cam2RightLaneB_LaneMarkModelDa'] = data.ld.cam2_right_lane_b.lane_mark_model_da
        # LD Lane Additional Data
        self.variables['Cam2LaneAdditionalData_AliveCounter'] = data.ld.cam2_lane_additional_data_1.alive_counter
        self.variables['Cam2LaneAdditionalData_LhGuardrail'] = data.ld.cam2_lane_additional_data_1.lh_guardrail
        self.variables['Cam2LaneAdditionalData_RhGuardrail'] = data.ld.cam2_lane_additional_data_1.rh_guardrail
        self.variables['Cam2LaneAdditionalData_LeftLaneColorInformation'] = data.ld.cam2_lane_additional_data_1.left_lane_color_information
        self.variables['Cam2LaneAdditionalData_RightLaneColorInformation'] = data.ld.cam2_lane_additional_data_1.right_lane_color_information
        # OD
        # OD Static Obstacle Data A
        for i in range(0, 16):
            self.variables['Cam2StaticObstacleDataA_RollingCount' + str(i)] = data.od.od_tracks[i].rolling_count_1
            self.variables['Cam2StaticObstacleDataA_StaticObjType' + str(i)] = data.od.od_tracks[i].camera2_static_obj_type
            self.variables['Cam2StaticObstacleDataA_StaticObjStatus' + str(i)] = data.od.od_tracks[i].static_object_status
            self.variables['Cam2StaticObstacleDataA_StaticObjPosY' + str(i)] = data.od.od_tracks[i].static_object_pos_y
            self.variables['Cam2StaticObstacleDataA_StaticObjPosX' + str(i)] = data.od.od_tracks[i].static_object_pos_x
            self.variables['Cam2StaticObstacleDataA_StaticObjPos2Y' + str(i)] = data.od.od_tracks[i].static_object_pos2_y
            self.variables['Cam2StaticObstacleDataA_StaticObjPos2X' + str(i)] = data.od.od_tracks[i].static_object_pos2_x
        # TD
        # TD Obstacle Data A
        for i in range(0, 16):
            self.variables['Cam2ObstacleDataA_AliveCounter' + str(i)] = data.td.td_tracks_a[i].alive_counter
            self.variables['Cam2ObstacleDataA_ObjectAge' + str(i)] = data.td.td_tracks_a[i].object_age
            self.variables['Cam2ObstacleDataA_AngleRate' + str(i)] = data.td.td_tracks_a[i].angle_rate
            self.variables['Cam2ObstacleDataA_AngleLeft' + str(i)] = data.td.td_tracks_a[i].angle_left
            self.variables['Cam2ObstacleDataA_AngleRight' + str(i)] = data.td.td_tracks_a[i].angle_right
            self.variables['Cam2ObstacleDataA_ObjectLane' + str(i)] = data.td.td_tracks_a[i].object_lane
            self.variables['Cam2ObstacleDataA_Cam2ObstacleBrakeLights' + str(i)] = data.td.td_tracks_a[i].cam2_obstacle_brake_lights
            self.variables['Cam2ObstacleDataA_MotionStatus' + str(i)] = data.td.td_tracks_a[i].motion_status
        # TD Obstacle Data B
        for i in range(0, 16):
            self.variables['Cam2ObstacleDataB_AliveCounter' + str(i)] = data.td.td_tracks_b[i].alive_counter
            self.variables['Cam2ObstacleDataB_Range' + str(i)] = data.td.td_tracks_b[i].range
            self.variables['Cam2ObstacleDataB_ObjectVaridity' + str(i)] = data.td.td_tracks_b[i].object_validity
            self.variables['Cam2ObstacleDataB_RangeRate' + str(i)] = data.td.td_tracks_b[i].range_rate
            self.variables['Cam2ObstacleDataB_Cam2ObstaclePhysicalWidth' + str(i)] = data.td.td_tracks_b[i].cam2_obstacle_physical_width
            self.variables['Cam2ObstacleDataB_Cam2TrackID' + str(i)] = data.td.td_tracks_b[i].cam2_track_id
            self.variables['Cam2ObstacleDataB_ObjectType' + str(i)] = data.td.td_tracks_b[i].object_type
        # TL
        # TL Traffic Light Data
        self.variables['Cam2TrafficLightData_TrafficLightData'] = data.tl.traffic_light_data
        self.variables['Cam2TrafficLightData_TrafficLightValidFlag'] = data.tl.traffic_light_valid_flag
        self.variables['Cam2TrafficLightData_TrafficLightAccuracy'] = data.tl.traffic_light_accuracy
        
        # print(self.variables)



    def callback_lidar_TD(self, lidar_11, lidar_12, lidar_13):
        # LiDAR 11_201 Data
        # TD
        # TD Track A FL
        print("Callback Start")
        i = 0
        for detection in lidar_11.detections:
            self.variables['Lidar2TrackAFL_TrackStatus' + str(i)] = int(detection.results[0].pose.covariance[0])
            self.variables['Lidar2TrackAFL_TrackValid' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAFL_TrackPosY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.1)
            self.variables['Lidar2TrackAFL_TrackPosX' + str(i)] = float_to_11bit_unsigned(detection.bbox.center.position.x,0.1)
            self.variables['Lidar2TrackAFL_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAFL_TrackType' + str(i)] = int(detection.results[0].hypothesis.class_id)
            self.variables['Lidar2TrackAFL_TrackVelY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.01)   ## NTC
            self.variables['Lidar2TrackAFL_TrackVelX' + str(i)] = float_to_14bit_signed(detection.bbox.center.position.x, 0.05)   ## NTC
            i += 1
        # TD Track A FR
        i = 0
        for detection in lidar_12.detections:
            self.variables['Lidar2TrackAFR_TrackStatus' + str(i)] = int(detection.results[0].pose.covariance[0])
            self.variables['Lidar2TrackAFR_TrackValid' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAFR_TrackPosY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.1)
            self.variables['Lidar2TrackAFR_TrackPosX' + str(i)] = float_to_11bit_unsigned(detection.bbox.center.position.x,0.1)
            self.variables['Lidar2TrackAFR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAFR_TrackType' + str(i)] = int(detection.results[0].hypothesis.class_id)
            self.variables['Lidar2TrackAFR_TrackVelY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.01)   ## NTC
            self.variables['Lidar2TrackAFR_TrackVelX' + str(i)] = float_to_14bit_signed(detection.bbox.center.position.x, 0.05)   ## NTC
            i += 1
        # TD Track A R
        i = 0
        for detection in lidar_13.detections:
            self.variables['Lidar2TrackAR_TrackStatus' + str(i)] = int(detection.results[0].pose.covariance[0])
            self.variables['Lidar2TrackAR_TrackValid' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAR_TrackPosY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.1)
            self.variables['Lidar2TrackAR_TrackPosX' + str(i)] = float_to_11bit_unsigned(detection.bbox.center.position.x,0.1)
            self.variables['Lidar2TrackAR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackAR_TrackType' + str(i)] = int(detection.results[0].hypothesis.class_id)
            self.variables['Lidar2TrackAR_TrackVelY' + str(i)] = float_to_10bit_signed(detection.bbox.center.position.y, 0.01)   ## NTC
            self.variables['Lidar2TrackAR_TrackVelX' + str(i)] = float_to_14bit_signed(detection.bbox.center.position.x, 0.05)   ## NTC
            i += 1
        # TD Track B FL
        i = 0
        for detection in lidar_11.detections:
            self.variables['Lidar2TrackBFL_TrackID' + str(i)] = int(detection.id)
            self.variables['Lidar2TrackBFL_TrackAddInfoStatus' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBFL_TrackWidth' + str(i)] = float_to_10bit_signed(detection.bbox.size.y, 0.1)
            self.variables['Lidar2TrackBFL_TrackLength' + str(i)] = float_to_11bit_unsigned(detection.bbox.size.x, 0.1)
            self.variables['Lidar2TrackBFL_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBFL_CharPointFirstY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y + detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackBFL_CharPointFirstX' + str(i)] = float_to_11bit_signed((detection.bbox.center.position.x - detection.bbox.size.x/2), 0.01)
            i += 1
        # TD Track B FR
        i = 0
        for detection in lidar_12.detections:
            self.variables['Lidar2TrackBFR_TrackID' + str(i)] = int(detection.id)
            self.variables['Lidar2TrackBFR_TrackAddInfoStatus' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBFR_TrackWidth' + str(i)] = float_to_10bit_signed(detection.bbox.size.y, 0.1)
            self.variables['Lidar2TrackBFR_TrackLength' + str(i)] = float_to_11bit_unsigned(detection.bbox.size.x, 0.1)
            self.variables['Lidar2TrackBFR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBFR_CharPointFirstY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y - detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackBFR_CharPointFirstX' + str(i)] = float_to_11bit_signed((detection.bbox.center.position.x - detection.bbox.size.x/2), 0.01)
            i += 1
        # TD Track B R
        i = 0
        for detection in lidar_13.detections:
            self.variables['Lidar2TrackBR_TrackID' + str(i)] = int(detection.id)
            self.variables['Lidar2TrackBR_TrackAddInfoStatus' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBR_TrackWidth' + str(i)] = float_to_10bit_signed(detection.bbox.size.y, 0.1)
            self.variables['Lidar2TrackBR_TrackLength' + str(i)] = float_to_11bit_unsigned(detection.bbox.size.x, 0.1)
            self.variables['Lidar2TrackBR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackBR_CharPointFirstY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y - detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackBR_CharPointFirstX' + str(i)] = float_to_11bit_signed((detection.bbox.center.position.x + detection.bbox.size.x/2), 0.01)
            i += 1
        # TD Track C FL
        i = 0
        for detection in lidar_11.detections:
            self.variables['Lidar2TrackCFL_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackCFL_CharPointSecondY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y - detection.bbox.size.y/2), 0.1)
            self.variables['Lidar2TrackCFL_CharPointSecondX' + str(i)] = float_to_14bit_unsigned((detection.bbox.center.position.x - detection.bbox.size.x/2), 0.1)
            self.variables['Lidar2TrackCFL_CharPointThirdY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y + detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackCFL_CharPointThirdX' + str(i)] = float_to_14bit_signed((detection.bbox.center.position.x + detection.bbox.size.x/2), 0.01)
            i += 1
        # TD Track C FR
        i = 0
        for detection in lidar_12.detections:
            self.variables['Lidar2TrackCFR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackCFR_CharPointSecondY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y + detection.bbox.size.y/2), 0.1)
            self.variables['Lidar2TrackCFR_CharPointSecondX' + str(i)] = float_to_14bit_unsigned((detection.bbox.center.position.x - detection.bbox.size.x/2), 0.1)
            self.variables['Lidar2TrackCFR_CharPointThirdY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y - detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackCFR_CharPointThirdX' + str(i)] = float_to_14bit_signed((detection.bbox.center.position.x + detection.bbox.size.x/2), 0.01)
            i += 1
        # TD Track C R
        i = 0
        for detection in lidar_13.detections:
            self.variables['Lidar2TrackCR_TrackRollingCount' + str(i)] = 1   ## NTC
            self.variables['Lidar2TrackCR_CharPointSecondY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y + detection.bbox.size.y/2), 0.1)
            self.variables['Lidar2TrackCR_CharPointSecondX' + str(i)] = float_to_14bit_unsigned((detection.bbox.center.position.x + detection.bbox.size.x/2), 0.1)
            self.variables['Lidar2TrackCR_CharPointThirdY' + str(i)] = float_to_10bit_signed((detection.bbox.center.position.y - detection.bbox.size.y/2), 0.05)
            self.variables['Lidar2TrackCR_CharPointThirdX' + str(i)] = float_to_14bit_signed((detection.bbox.center.position.x - detection.bbox.size.x/2), 0.01)
            i += 1

        # print(self.variables)

    def callback_lidar_CD(self, lidar_left_lane, lidar_right_lane):
        # CD
        left_lane = parse_data(lidar_left_lane.data)
        right_lane = parse_data(lidar_right_lane.data)

        # CD Left Curb A
        self.variables['Lidar2LeftCurbA_CurbMarkType'] = int(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Type"])
        self.variables['Lidar2LeftCurbA_CurbMarkQuality'] = int(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Quality"])
        self.variables['Lidar2LeftCurbA_CurbMarkModelA'] = np.uint32(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Model_A"] * (1<<32) + 0x7FFFFFFF)
        self.variables['Lidar2LeftCurbA_CurbMarkPosition'] = np.int16(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Position"] * 256)
        self.variables['Lidar2LeftCurbA_CurbMarkWidth'] = 15
        # CD Right Curb A
        self.variables['Lidar2RightCurbA_CurbMarkType'] = int(right_lane["Lidar2_Right_Curb_A_Curb_Mark_Type"])
        self.variables['Lidar2RightCurbA_CurbMarkQuality'] = int(right_lane["Lidar2_Right_Curb_A_Curb_Mark_Quality"])
        self.variables['Lidar2RightCurbA_CurbMarkModelA'] = np.uint32((right_lane["Lidar2_Right_Curb_A_Curb_Mark_Model_A"] * (1<<32)) + 0x7FFFFFFF)
        self.variables['Lidar2RightCurbA_CurbMarkPosition'] = np.int16(right_lane["Lidar2_Right_Curb_A_Curb_Mark_Position"] * 256)
        self.variables['Lidar2RightCurbA_CurbMarkWidth'] = 15
        # CD Left Curb B
        self.variables['Lidar2LeftCurbB_CurbMarkHeading'] = np.uint16(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Heading_Angle"] * (0x10000) + 0x7FFF)
        self.variables['Lidar2LeftCurbB_CurbMarkModelViewRange'] = 20
        self.variables['Lidar2LeftCurbB_CurbMarkModelViewRangeAvailability'] = 1
        self.variables['Lidar2LeftCurbB_CurbMarkModelDa'] = np.uint32(left_lane["Lidar2_Left_Curb_A_Curb_Mark_Model_dA"] * (1<<36) + 0x7FFFFFFF)
        # CD Right Curb B
        self.variables['Lidar2RightCurbB_CurbMarkHeading'] = np.uint16(right_lane["Lidar2_Right_Curb_A_Curb_Mark_Heading_Angle"] * (0x10000) + 0x7FFF)
        self.variables['Lidar2RightCurbB_CurbMarkModelViewRange'] = 20
        self.variables['Lidar2RightCurbB_CurbMarkModelViewRangeAvailability'] = 1
        self.variables['Lidar2RightCurbB_CurbMarkModelDa'] = np.uint32(right_lane["Lidar2_Right_Curb_A_Curb_Mark_Model_dA"] * (1<<36) + 0x7FFFFFFF)

        # print(self.variables)

    # 4. Bit into Packet
    def send_udp_messages(self):
        packet = bytearray(PACKET_SIZE)  # Packet Init

        # Value >> Packet
        global index
        index = 0
        # Cam LD Left & Right Lane A
        # Left
        index += 4 if 'Cam2LeftLaneA_LaneMarkType' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneA_LaneMarkType'], index, 4) else 0
        index += 4 if 'Cam2LeftLaneA_LaneMarkQuality' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneA_LaneMarkQuality'], index, 2) else 0
        index += 16 if 'Cam2LeftLaneA_LaneMarkPosition' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneA_LaneMarkPosition'], index, 16) else 0
        index += 32 if 'Cam2LeftLaneA_LaneMarkModelA' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneA_LaneMarkModelA'], index, 32) else 0
        index += 8 if 'Cam2LeftLaneA_LaneMarkWidth' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneA_LaneMarkWidth'], index, 8) else 0
        # Right
        index += 4 if 'Cam2RightLaneA_LaneMarkType' in self.variables and set_bits(packet, self.variables['Cam2RightLaneA_LaneMarkType'], index, 4) else 0
        index += 4 if 'Cam2RightLaneA_LaneMarkQuality' in self.variables and set_bits(packet, self.variables['Cam2RightLaneA_LaneMarkQuality'], index, 2) else 0
        index += 16 if 'Cam2RightLaneA_LaneMarkPosition' in self.variables and set_bits(packet, self.variables['Cam2RightLaneA_LaneMarkPosition'], index, 16) else 0
        index += 32 if 'Cam2RightLaneA_LaneMarkModelA' in self.variables and set_bits(packet, self.variables['Cam2RightLaneA_LaneMarkModelA'], index, 32) else 0
        index += 8 if 'Cam2RightLaneA_LaneMarkWidth' in self.variables and set_bits(packet, self.variables['Cam2RightLaneA_LaneMarkWidth'], index, 8) else 0
        # Cam LD Left & Right Lane B
        # Left
        index += 16 if 'Cam2LeftLaneB_LaneMarkHeadingAngle' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneB_LaneMarkHeadingAngle'], index, 16) else 0
        index += 15 if 'Cam2LeftLaneB_LaneMarkModelViewRange' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneB_LaneMarkModelViewRange'], index, 15) else 0
        index += 1 if 'Cam2LeftLaneB_LaneMarkModelViewRangeAvailability' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneB_LaneMarkModelViewRangeAvailability'], index, 1) else 0
        index += 32 if 'Cam2LeftLaneB_LaneMarkModelDa' in self.variables and set_bits(packet, self.variables['Cam2LeftLaneB_LaneMarkModelDa'], index, 32) else 0
        # Right
        index += 16 if 'Cam2RightLaneB_LaneMarkHeadingAngle' in self.variables and set_bits(packet, self.variables['Cam2RightLaneB_LaneMarkHeadingAngle'], index, 16) else 0
        index += 15 if 'Cam2RightLaneB_LaneMarkModelViewRange' in self.variables and set_bits(packet, self.variables['Cam2RightLaneB_LaneMarkModelViewRange'], index, 15) else 0
        index += 1 if 'Cam2RightLaneB_LaneMarkModelViewRangeAvailability' in self.variables and set_bits(packet, self.variables['Cam2RightLaneB_LaneMarkModelViewRangeAvailability'], index, 1) else 0
        index += 32 if 'Cam2RightLaneB_LaneMarkModelDa' in self.variables and set_bits(packet, self.variables['Cam2RightLaneB_LaneMarkModelDa'], index, 32) else 0
        # Cam LD Additional Data
        index += 18 if 'Cam2LaneAdditionalData_AliveCounter' in self.variables and set_bits(packet, self.variables['Cam2LaneAdditionalData_AliveCounter'], index, 8) else 0
        index += 1 if 'Cam2LaneAdditionalData_RhGuardrail' in self.variables and set_bits(packet, self.variables['Cam2LaneAdditionalData_RhGuardrail'], index, 1) else 0
        index += 9 if 'Cam2LaneAdditionalData_LhGuardrail' in self.variables and set_bits(packet, self.variables['Cam2LaneAdditionalData_LhGuardrail'], index, 1) else 0
        index += 2 if 'Cam2LaneAdditionalData_RightLaneColorInformation' in self.variables and set_bits(packet, self.variables['Cam2LaneAdditionalData_RightLaneColorInformation'], index, 2) else 0
        index += 34 if 'Cam2LaneAdditionalData_LeftLaneColorInformation' in self.variables and set_bits(packet, self.variables['Cam2LaneAdditionalData_LeftLaneColorInformation'], index, 2) else 0
        # Cam TD Obstacle Data A
        for i in range(0, 16):
            index += 8 if 'Cam2ObstacleDataA_AliveCounter' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_AliveCounter' + str(i)], index, 2) else 0
            index += 8 if 'Cam2ObstacleDataA_ObjectAge' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_ObjectAge' + str(i)], index, 8) else 0
            index += 12 if 'Cam2ObstacleDataA_AngleRate' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_AngleRate' + str(i)], index, 12) else 0
            index += 12 if 'Cam2ObstacleDataA_AngleLeft' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_AngleLeft' + str(i)], index, 12) else 0
            index += 12 if 'Cam2ObstacleDataA_AngleRight' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_AngleRight' + str(i)], index, 12) else 0
            index += 4 if 'Cam2ObstacleDataA_MotionStatus' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_MotionStatus' + str(i)], index, 4) else 0
            index += 5 if 'Cam2ObstacleDataA_ObjectLane' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_ObjectLane' + str(i)], index, 4) else 0
            index += 3 if 'Cam2ObstacleDataA_Cam2ObstacleBrakeLights' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataA_Cam2ObstacleBrakeLights' + str(i)], index, 1) else 0
        # Cam TD Obstacle Data B
        for i in range(0, 16):
            index += 2 if 'Cam2ObstacleDataB_AliveCounter' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_AliveCounter' + str(i)], index, 2) else 0
            index += 12 if 'Cam2ObstacleDataB_Range' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_Range' + str(i)], index, 12) else 0
            index += 4 if 'Cam2ObstacleDataB_ObjectVaridity' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_ObjectVaridity' + str(i)], index, 2) else 0
            index += 14 if 'Cam2ObstacleDataB_RangeRate' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_RangeRate' + str(i)], index, 12) else 0
            index += 8 if 'Cam2ObstacleDataB_Cam2ObstaclePhysicalWidth' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_Cam2ObstaclePhysicalWidth' + str(i)], index, 8) else 0
            index += 8 if 'Cam2ObstacleDataB_Cam2TrackID' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_Cam2TrackID' + str(i)], index, 8) else 0
            index += 16 if 'Cam2ObstacleDataB_ObjectType' + str(i) in self.variables and set_bits(packet, self.variables['Cam2ObstacleDataB_ObjectType' + str(i)], index, 4) else 0
        # Cam TL Traffic Light Data A
        index += 6 if 'Cam2TrafficLightData_TrafficLightData' in self.variables and set_bits(packet, self.variables['Cam2TrafficLightData_TrafficLightData'], index, 4) else 0
        index += 2 if 'Cam2TrafficLightData_TrafficLightValidFlag' in self.variables and set_bits(packet, self.variables['Cam2TrafficLightData_TrafficLightValidFlag'], index, 1) else 0
        index += 56 if 'Cam2TrafficLightData_TrafficLightAccuracy' in self.variables and set_bits(packet, self.variables['Cam2TrafficLightData_TrafficLightAccuracy'], index, 16) else 0
        # Cam Static Object Data A
        for i in range(0, 16):
            index += 2 if 'Cam2StaticObstacleDataA_RollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_RollingCount' + str(i)], index, 2) else 0
            index += 6 if 'Cam2StaticObstacleDataA_StaticObjType' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjType' + str(i)], index, 4) else 0
            index += 3 if 'Cam2StaticObstacleDataA_StaticObjStatus' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjStatus' + str(i)], index, 3) else 0
            index += 10 if 'Cam2StaticObstacleDataA_StaticObjPosY' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjPosY' + str(i)], index, 10) else 0
            index += 17 if 'Cam2StaticObstacleDataA_StaticObjPosX' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjPosX' + str(i)], index, 11) else 0
            index += 10 if 'Cam2StaticObstacleDataA_StaticObjPos2Y' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjPos2Y' + str(i)], index, 10) else 0
            index += 16 if 'Cam2StaticObstacleDataA_StaticObjPos2X' + str(i) in self.variables and set_bits(packet, self.variables['Cam2StaticObstacleDataA_StaticObjPos2X' + str(i)], index, 11) else 0
        # Lidar TD Track A FL
        for i in range(0, 32):
            index += 3 if 'Lidar2TrackAFL_TrackStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackStatus' + str(i)], index, 3) else 0
            index += 3 if 'Lidar2TrackAFL_TrackValid' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackValid' + str(i)], index, 1) else 0
            index += 10 if 'Lidar2TrackAFL_TrackPosY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackPosY' + str(i)], index, 10) else 0
            index += 12 if 'Lidar2TrackAFL_TrackPosX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackPosX' + str(i)], index, 11) else 0
            index += 6 if 'Lidar2TrackAFL_TrackType' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackType' + str(i)], index, 4) else 0
            index += 4 if 'Lidar2TrackAFL_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackRollingCount' + str(i)], index, 2) else 0
            index += 12 if 'Lidar2TrackAFL_TrackVelY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackVelY' + str(i)], index, 10) else 0
            index += 14 if 'Lidar2TrackAFL_TrackVelX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFL_TrackVelX' + str(i)], index, 14) else 0
        # Lidar TD Track A FR
        for i in range(0, 32):
            index += 3 if 'Lidar2TrackAFR_TrackStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackStatus' + str(i)], index, 3) else 0
            index += 3 if 'Lidar2TrackAFR_TrackValid' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackValid' + str(i)], index, 1) else 0
            index += 10 if 'Lidar2TrackAFR_TrackPosY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackPosY' + str(i)], index, 10) else 0
            index += 12 if 'Lidar2TrackAFR_TrackPosX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackPosX' + str(i)], index, 11) else 0
            index += 6 if 'Lidar2TrackAFR_TrackType' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackType' + str(i)], index, 4) else 0
            index += 4 if 'Lidar2TrackAFR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackRollingCount' + str(i)], index, 2) else 0
            index += 12 if 'Lidar2TrackAFR_TrackVelY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackVelY' + str(i)], index, 10) else 0
            index += 14 if 'Lidar2TrackAFR_TrackVelX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAFR_TrackVelX' + str(i)], index, 14) else 0
        # Lidar TD Track A R
        for i in range(0, 32):
            index += 3 if 'Lidar2TrackAR_TrackStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackStatus' + str(i)], index, 3) else 0
            index += 3 if 'Lidar2TrackAR_TrackValid' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackValid' + str(i)], index, 1) else 0
            index += 10 if 'Lidar2TrackAR_TrackPosY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackPosY' + str(i)], index, 10) else 0
            index += 12 if 'Lidar2TrackAR_TrackPosX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackPosX' + str(i)], index, 11) else 0
            index += 6 if 'Lidar2TrackAR_TrackType' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackType' + str(i)], index, 4) else 0
            index += 4 if 'Lidar2TrackAR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackRollingCount' + str(i)], index, 2) else 0
            index += 12 if 'Lidar2TrackAR_TrackVelY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackVelY' + str(i)], index, 10) else 0
            index += 14 if 'Lidar2TrackAR_TrackVelX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackAR_TrackVelX' + str(i)], index, 14) else 0
        # Lidar TD Track B FL
        for i in range(0, 32):
            index += 8 if 'Lidar2TrackBFL_TrackID' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_TrackID' + str(i)], index, 8) else 0
            index += 3 if 'Lidar2TrackBFL_TrackAddInfoStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_TrackAddInfoStatus' + str(i)], index, 3) else 0
            index += 10 if 'Lidar2TrackBFL_TrackWidth' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_TrackWidth' + str(i)], index, 10) else 0
            index += 15 if 'Lidar2TrackBFL_TrackLength' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_TrackLength' + str(i)], index, 11) else 0
            index += 1 if 'Lidar2TrackBFL_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackBFL_CharPointFirstY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_CharPointFirstY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackBFL_CharPointFirstX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFL_CharPointFirstX' + str(i)], index, 11) else 0
        # Lidar TD Track B FR
        for i in range(0, 32):
            index += 8 if 'Lidar2TrackBFR_TrackID' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_TrackID' + str(i)], index, 8) else 0
            index += 3 if 'Lidar2TrackBFR_TrackAddInfoStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_TrackAddInfoStatus' + str(i)], index, 3) else 0
            index += 10 if 'Lidar2TrackBFR_TrackWidth' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_TrackWidth' + str(i)], index, 10) else 0
            index += 15 if 'Lidar2TrackBFR_TrackLength' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_TrackLength' + str(i)], index, 11) else 0
            index += 1 if 'Lidar2TrackBFR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackBFR_CharPointFirstY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_CharPointFirstY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackBFR_CharPointFirstX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBFR_CharPointFirstX' + str(i)], index, 11) else 0
        # Lidar TD Track B R
        for i in range(0, 32):
            index += 8 if 'Lidar2TrackBR_TrackID' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_TrackID' + str(i)], index, 8) else 0
            index += 3 if 'Lidar2TrackBR_TrackAddInfoStatus' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_TrackAddInfoStatus' + str(i)], index, 3) else 0
            index += 10 if 'Lidar2TrackBR_TrackWidth' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_TrackWidth' + str(i)], index, 10) else 0
            index += 15 if 'Lidar2TrackBR_TrackLength' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_TrackLength' + str(i)], index, 11) else 0
            index += 1 if 'Lidar2TrackBR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackBR_CharPointFirstY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_CharPointFirstY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackBR_CharPointFirstX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackBR_CharPointFirstX' + str(i)], index, 11) else 0
        # Lidar TD Track C FL
        for i in range(0, 32):
            index += 5
            index += 1 if 'Lidar2TrackCFL_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFL_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackCFL_CharPointSecondY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFL_CharPointSecondY' + str(i)], index, 10) else 0
            index += 17 if 'Lidar2TrackCFL_CharPointSecondX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFL_CharPointSecondX' + str(i)], index, 11) else 0
            index += 15 if 'Lidar2TrackCFL_CharPointThirdY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFL_CharPointThirdY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackCFL_CharPointThirdX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFL_CharPointThirdX' + str(i)], index, 11) else 0

        # Lidar TD Track C FR
        for i in range(0, 32):
            index += 5
            index += 1 if 'Lidar2TrackCFR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFR_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackCFR_CharPointSecondY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFR_CharPointSecondY' + str(i)], index, 10) else 0
            index += 17 if 'Lidar2TrackCFR_CharPointSecondX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFR_CharPointSecondX' + str(i)], index, 11) else 0
            index += 15 if 'Lidar2TrackCFR_CharPointThirdY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFR_CharPointThirdY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackCFR_CharPointThirdX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCFR_CharPointThirdX' + str(i)], index, 11) else 0

        # Lidar TD Track C R
        for i in range(0, 32):
            index += 5
            index += 1 if 'Lidar2TrackCR_TrackRollingCount' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCR_TrackRollingCount' + str(i)], index, 1) else 0
            index += 15 if 'Lidar2TrackCR_CharPointSecondY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCR_CharPointSecondY' + str(i)], index, 10) else 0
            index += 17 if 'Lidar2TrackCR_CharPointSecondX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCR_CharPointSecondX' + str(i)], index, 11) else 0
            index += 15 if 'Lidar2TrackCR_CharPointThirdY' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCR_CharPointThirdY' + str(i)], index, 10) else 0
            index += 11 if 'Lidar2TrackCR_CharPointThirdX' + str(i) in self.variables and set_bits(packet, self.variables['Lidar2TrackCR_CharPointThirdX' + str(i)], index, 11) else 0
        
        # Lidar CD Left Curb A
        index += 4 if 'Lidar2LeftCurbA_CurbMarkType' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbA_CurbMarkType'], index, 4) else 0
        index += 4 if 'Lidar2LeftCurbA_CurbMarkQuality' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbA_CurbMarkQuality'], index, 2) else 0
        index += 16 if 'Lidar2LeftCurbA_CurbMarkPosition' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbA_CurbMarkPosition'], index, 16) else 0
        index += 32 if 'Lidar2LeftCurbA_CurbMarkModelA' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbA_CurbMarkModelA'], index, 32) else 0
        index += 8 if 'Lidar2LeftCurbA_CurbMarkWidth' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbA_CurbMarkWidth'], index, 8) else 0
        # Lidar CD Right Curb A
        index += 4 if 'Lidar2RightCurbA_CurbMarkType' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbA_CurbMarkType'], index, 4) else 0
        index += 4 if 'Lidar2RightCurbA_CurbMarkQuality' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbA_CurbMarkQuality'], index, 2) else 0
        index += 16 if 'Lidar2RightCurbA_CurbMarkPosition' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbA_CurbMarkPosition'], index, 16) else 0
        index += 32 if 'Lidar2RightCurbA_CurbMarkModelA' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbA_CurbMarkModelA'], index, 32) else 0
        index += 8 if 'Lidar2RightCurbA_CurbMarkWidth' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbA_CurbMarkWidth'], index, 8) else 0
        # Lidar CD Left Curb B
        index += 16 if 'Lidar2LeftCurbB_CurbMarkHeading' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbB_CurbMarkHeading'], index, 16) else 0
        index += 15 if 'Lidar2LeftCurbB_CurbMarkModelViewRange' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbB_CurbMarkModelViewRange'], index, 15) else 0
        index += 1 if 'Lidar2LeftCurbB_CurbMarkModelViewRangeAvailability' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbB_CurbMarkModelViewRangeAvailability'], index, 1) else 0
        index += 32 if 'Lidar2LeftCurbB_CurbMarkModelDa' in self.variables and set_bits(packet, self.variables['Lidar2LeftCurbB_CurbMarkModelDa'], index, 32) else 0
        # Lidar CD Right Curb B
        index += 16 if 'Lidar2RightCurbB_CurbMarkHeading' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbB_CurbMarkHeading'], index, 16) else 0
        index += 15 if 'Lidar2RightCurbB_CurbMarkModelViewRange' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbB_CurbMarkModelViewRange'], index, 15) else 0
        index += 1 if 'Lidar2RightCurbB_CurbMarkModelViewRangeAvailability' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbB_CurbMarkModelViewRangeAvailability'], index, 1) else 0
        index += 32 if 'Lidar2RightCurbB_CurbMarkModelDa' in self.variables and set_bits(packet, self.variables['Lidar2RightCurbB_CurbMarkModelDa'], index, 32) else 0
        # Fill to 0 about Remain Packet
        if index < PACKET_SIZE * 8:
            remaining_bits = (PACKET_SIZE * 8) - index
            for i in range(remaining_bits):
                set_bits(packet, 0, index, 1)
                index += 1

        # 5. Send to UDP 
        # if self.variables['Lidar2TrackAFL_TrackPosX0'] is not None:
        #     udp_client_socket.sendto(packet, server_addr_port)
        #     self.get_logger().info("UDP packet sent")
        #     print(packet)
        udp_client_socket.sendto(packet, server_addr_port)
        self.get_logger().info("UDP packet sent")
        
        print(packet[3456:])

def main(args=None):
    rclpy.init(args=args)
    udp_client_node = UDPClientNode()
    rclpy.spin(udp_client_node)
    udp_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()