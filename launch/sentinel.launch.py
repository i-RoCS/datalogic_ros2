from  launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import os
from launch_ros.actions import Node
import pathlib

parameters_file_name = 'param.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/config/' + parameters_file_name
    print(parameters_file_path)
    return LaunchDescription([
        Node(
            package='datalogic',
            node_executable='sentinel_node',
            node_name='sentinel_node',
            parameters=[
                parameters_file_path
            ],
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='tf_master',
            arguments=['-0.0525', '0', '0.1143', '0', '0', '0', '1', 'base_link', 'laser_master_frame']
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='tf_slave',
            arguments=['0.0525', '0', '0.1143', '3.1415', '0', '0',  'base_link', 'laser_slave1_frame']
        )
    ])

