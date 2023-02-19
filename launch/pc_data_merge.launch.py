import os

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    input_points_topics_list_param = DeclareLaunchArgument(
        "input_points_topics_list",
        default_value="['/luminar_left_points', '/luminar_right_points', '/luminar_front_points']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )
    
    tf_output_frame_param = DeclareLaunchArgument("output_frame", default_value="merged_points")
    
    concat_node = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_filter",
        remappings=[("output", "points_raw/concatenated")],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_points_topics_list"),
                "output_frame": LaunchConfiguration("output_frame"),
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