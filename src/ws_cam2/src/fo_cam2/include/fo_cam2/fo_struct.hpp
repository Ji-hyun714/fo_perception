#include <cstdint>

#define CAM2_FIELD_SIZE    406
#define CAM_TRACK_NUM 16

#pragma pack(push, 1)
/// Cam2_LD
struct Cam2_X_Lane_A // L, R
{
    uint64_t lane_mark_type     : 4;
    uint64_t lane_mark_quality  : 2;
    uint64_t                    : 2; //pad
    uint64_t lane_mark_position : 16;
    uint64_t lane_mark_model_a  : 32;
    uint64_t lane_mark_width    : 8;
}; // total 8 bytes(64 bits) = data 62 bits + pad 2bits

struct Cam2_X_Lane_B // L, R
{
    uint64_t lane_mark_heading_angle                 : 16;
    uint64_t lane_mark_model_view_range              : 15;
    uint64_t lane_mark_model_view_range_availability : 1;
    uint64_t lane_mark_model_da                      : 32;
}; // total 8 bytes(64 bits)

struct Cam2_Lane_Additional_Data_1
{
    uint8_t alive_counter                : 8;
    uint8_t                              : 2;
    uint8_t rh_guardrail                 : 1;
    uint8_t lh_guardrail                 : 1;
    uint8_t                              : 4;
    uint8_t                              : 4;
    uint8_t right_lane_color_information : 2;
    uint8_t left_lane_color_information  : 2;
}; // total 3 bytes(24 bits) = data 14 bits + pad 10 bits

struct Cam2_LD
{
    struct Cam2_X_Lane_A cam2_left_lane_a;                          // 8  
    struct Cam2_X_Lane_A cam2_right_lane_a;                         // 8  
    struct Cam2_X_Lane_B cam2_left_lane_b;                          // 8
    struct Cam2_X_Lane_B cam2_right_lane_b;                         // 8 
    struct Cam2_Lane_Additional_Data_1 cam2_lane_additional_data_1; // 3
}; // total 35 bytes

/// Cam2_TD

struct Cam2_Obstacle_Data_A // 16 tracks
{
    uint64_t alive_counter              : 2;  //rolling counter
    uint64_t                            : 6;
    uint64_t object_age                 : 8;
    uint64_t angle_rate                 : 12;
    uint64_t angle_left                 : 12;
    uint64_t angle_right                : 12;
    uint64_t motion_status              : 4;
    uint64_t object_lane                : 4;
    uint64_t                            : 1;
    uint64_t cam2_obstacle_brake_lights : 1;
    uint64_t                            : 2;
}; // total 8 bytes(64 bits) = data 55 bits + pad 9 bits

struct Cam2_Obstacle_Data_B // 16 tracks
{
    uint8_t alive_counter                : 2;
    uint8_t range_0                      : 6;
    uint8_t range_1                      : 6; // range : 12 bits
    uint8_t object_vaildity              : 2;
    uint8_t                              : 2;
    uint8_t range_rate_0                 : 6;
    uint8_t range_rate_1                 : 6; // range_rate : 12 bits
    uint8_t                              : 2;
    uint8_t cam2_obstacle_physical_width : 8;
    uint8_t cam2_track_id                : 8;
    uint8_t object_type                  : 4;
    uint8_t                              : 4;
}; // total 7 bytes(56 bits) = data 48 bits + pad 8 bits
   // range, range_rate는 utin8_t로 메모리 단위를 낮추기 위해서 프로토콜 상 layout에 맞춰서 분리 -> app 단에서 masking 하기

struct Cam2_TD
{
    struct Cam2_Obstacle_Data_A cam2_obstacle_data_a[CAM_TRACK_NUM]; // 8*16 = 128 bytes
    struct Cam2_Obstacle_Data_B cam2_obstacle_data_b[CAM_TRACK_NUM]; // 7*16 = 112 bytes
}; // total 240 bytes


/// Cam2_TL
struct Cam2_TL_Cam2_Traffic_Light_Data
{
    uint8_t traffic_light_data       : 4;
    uint8_t                          : 2;
    uint8_t traffic_light_valid_flag : 2; 
    uint8_t traffic_light_accuracy_0 : 8;
    uint8_t traffic_light_accuracy_1 : 8; // traffic_light_accuracy : 16 bits

}; // total 3 bytes(24 bits) = data 22 bits + pad 2 bits

// Cam2_OD
struct Camera2_Static_Object_Data // 16 tracks
{
    uint64_t rolling_count_1         : 2;
    uint64_t camera2_static_obj_type : 4;
    uint64_t                         : 2;
    uint64_t static_object_status    : 3;
    uint64_t static_object_pos_y     : 10;
    uint64_t static_object_pos_x     : 11;
    uint64_t                         : 6;
    uint64_t static_object_pos2_y    : 10;
    uint64_t static_object_pos2_x    : 11;
    uint64_t                         : 5;
}; // total 8 bytes(64 bits) = data 51 bits + pad 13 bits

struct Cam2_OD
{
    struct Camera2_Static_Object_Data camera2_static_object_data[CAM_TRACK_NUM]; // 8*16 = 128 bytes
};

struct Cam2_Data
{
    Cam2_LD                         ld; // 35
    Cam2_TD                         td; // 240
    Cam2_TL_Cam2_Traffic_Light_Data tl; // 3
    Cam2_OD                         od; // 128
}; // total 406 bytes
#pragma pack(pop)

static_assert(sizeof(Cam2_LD)                         == 35,              "Struct size mismatch : Cam2_LD");
static_assert(sizeof(Cam2_TD)                         == 240,             "Struct size mismatch : Cam2_TD");
static_assert(sizeof(Cam2_TL_Cam2_Traffic_Light_Data) == 3,               "Struct size mismatch : Cam2_TL_Cam2_Traffic_Light_Data");
static_assert(sizeof(Cam2_OD)                         == 128,             "Struct size mismatch : Cam2_OD");
static_assert(sizeof(Cam2_Data)                       == CAM2_FIELD_SIZE, "Struct size mismatch : Cam2_Data");