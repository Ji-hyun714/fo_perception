""" Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  """Generate launch description for ublox_dgnss components."""

  device_family = LaunchConfiguration("device_family")
  device_serial_string = LaunchConfiguration('device_serial_string')

  log_level_arg = DeclareLaunchArgument(
    "log_level", default_value=TextSubstitution(text="INFO")
  )
  device_family_arg = DeclareLaunchArgument(
    "device_family", default_value=TextSubstitution(text="F9P")
  )
  # TODO: Review - default changed to "" so an unreliable-iSerial F9P
  # (reports garbage serial) matches the first found device instead of failing.
  device_serial_string_arg = DeclareLaunchArgument(
    "device_serial_string",
    default_value="",
    description="Serial string of the device to use (empty = first device found)"
  )
  # """ TODO: Review - Original default
  # device_serial_string_arg = DeclareLaunchArgument(
  #   "device_serial_string",
  #   default_value="Test Rover",
  #   description="Serial string of the device to use"
  # )
  # """

  params_rover = [
            {"DEVICE_FAMILY": device_family},
            {'DEVICE_SERIAL_STRING': device_serial_string},
            {'FRAME_ID': "rover"},

            {'CFG_RATE_MEAS': 0x64},    # currently: 100ms (base 10Hz / rover 5Hz)
            # {'CFG_RATE_MEAS': 0xc8},  # original: 200ms(5Hz)
            # {'CFG_RATE_MEAS': 0x3e8}, # heading only: 1000ms(1Hz)
            {'CFG_RATE_NAV': 0x1},

            # disable all messages on UART1
            {'CFG_UART1INPROT_NMEA': False},
            {'CFG_UART1INPROT_RTCM3X': False},
            {'CFG_UART1INPROT_UBX': False},
            {'CFG_UART1OUTPROT_NMEA': False},
            {'CFG_UART1OUTPROT_RTCM3X': False},
            {'CFG_UART1OUTPROT_UBX': False},

            # set UART2 baud rate to 115200 (must match moving base 4072 output)
            {'CFG_UART2_BAUDRATE': 115200},

            # receive RTCM messages only (from base) on UART2
            {'CFG_UART2INPROT_NMEA': False},
            {'CFG_UART2INPROT_RTCM3X': True},
            {'CFG_UART2INPROT_UBX': False},
            {'CFG_UART2OUTPROT_NMEA': False},
            {'CFG_UART2OUTPROT_RTCM3X': False},
            {'CFG_UART2OUTPROT_UBX': False},

            # send/receive UBX messages only on USB
            {'CFG_USBINPROT_NMEA': False},
            {'CFG_USBINPROT_RTCM3X': False},
            {'CFG_USBINPROT_UBX': True},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_USBOUTPROT_RTCM3X': False},
            {'CFG_USBOUTPROT_UBX': True},

            # messages required for navsatfix calcs by ROS node
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
            {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},

            # output relative position messages
            {'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 0x1},

            # rover가 UART2(base 링크)로 받은 RTCM 수신/사용 여부 확인용 → /rover/ubx_rxm_rtcm (msg_used)
            # (MRD 반영은 base_rtk_monitor.py의 /base/carr_soln로 판정)
            {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 0x1},
            ]

  container_rover = ComposableNodeContainer(
    name='ublox_dgnss_rover',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        namespace='rover',
        parameters=params_rover
      )
    ]
  )

  container_navsatfix = ComposableNodeContainer(
    name='ublox_nav_sat_fix_hp_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        namespace='rover',
        name='ublox_nav_sat_fix_hp'
      )
    ]
  )

  return launch.LaunchDescription([
    log_level_arg,
    device_family_arg,
    device_serial_string_arg,
    container_rover,
    container_navsatfix,
    ])
