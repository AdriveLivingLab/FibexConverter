from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
      return LaunchDescription([
          # Declare arguments with default values
          DeclareLaunchArgument('port_num_gw',          default_value='19227'),
          DeclareLaunchArgument('ip_address_gw',          default_value='169.254.200.123'),

          # ****************************************************************** 
          # Node
          # ****************************************************************** 
          Node(
                name='ixxat_gw_mlbevo_flexray',
                namespace='ixxat_gw',
                package='ixxat_gw_mlbevo_flexray',
                executable='ixxat_gw_mlbevo_flexray',
                parameters=[
                  {
                    # Required parameters used to connect to the NTRIP server
                    'port_num_gw': LaunchConfiguration('port_num_gw'),
                    'ip_address_gw': LaunchConfiguration('ip_address_gw'),
                  }
                ]
          )
      ])