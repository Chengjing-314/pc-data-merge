import os

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    input_points_topics_list_param = DeclareLaunchArgument(
        "input_points_topics_list",
        default_value="['/livox/lidar/front_left', '/livox/lidar/front_center', '/livox/lidar/front_right']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )
    
    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")
    
    concat_node = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_filter",
        remappings=[("output", "points_raw/concatenated")],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_points_topics_list"),
                "output_frame": LaunchConfiguration("tf_output_frame"),
                "timeout_sec": 0.0333,
            }
        ],)
    
    return launch.LaunchDescription(
        [   
            input_points_topics_list_param,
            tf_output_frame_param,
            concat_node,
        ]
    )