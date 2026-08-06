from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic_ld = LaunchConfiguration("output_topic_ld")
    output_topic_cam2_data = LaunchConfiguration("output_topic_cam2_data")
    output_topic_sf = LaunchConfiguration("output_topic_sf")
    bbox_image_topic = LaunchConfiguration("bbox_image_topic")
    yaml_path = LaunchConfiguration("yaml_path")
    ld_cfg = LaunchConfiguration("ld_cfg")
    debug_mode = LaunchConfiguration("debug_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/image_rect",
            description="Input rectified image topic",
        ),
        DeclareLaunchArgument(
            "output_topic_ld",
            default_value="/camera/lane_result",
            description="Output Cam2LD lane result topic",
        ),
        DeclareLaunchArgument(
            "output_topic_cam2_data",
            default_value="/camera/cam2data",
            description="Output Cam2Data topic",
        ),
        DeclareLaunchArgument(
            "output_topic_sf",
            default_value="/camera/sf_objs",
            description="Output SF helper topic",
        ),
        DeclareLaunchArgument(
            "bbox_image_topic",
            default_value="/camera/det_bboxes",
            description="Debug bbox image topic used when debug_mode is true",
        ),
        DeclareLaunchArgument(
            "yaml_path",
            default_value="/home/wise/fo_perception/src/ws_cam2/src/camera_stream/yaml/test_rtsp_25.yaml",
            description="Camera calibration YAML path. Empty uses the built-in default.",
        ),
        DeclareLaunchArgument(
            "ld_cfg",
            default_value="/home/wise/fo_perception/src/ws_cam2/src/lane_detection/yaml/kcity.yaml",
            description="Lane detection config YAML path. Empty uses the built-in default.",
        ),
        DeclareLaunchArgument(
            "debug_mode",
            # default_value="false",
            default_value="true",
            description="Enable debug bbox image and lane marker publishers",
        ),
        Node(
            package="lane_detection",
            executable="yolopv2_cam2_node",
            name="yolopv2_cam2_node",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic_ld": output_topic_ld,
                "output_topic_cam2_data": output_topic_cam2_data,
                "output_topic_sf": output_topic_sf,
                "bbox_image_topic": bbox_image_topic,
                "yaml_path": yaml_path,
                "ld_cfg": ld_cfg,
                "debug_mode": debug_mode,
            }],
        ),
    ])
