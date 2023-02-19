source /opt/ros/gala/setup.bash
source /path_to_colcon_ws/install/setup.bash


# launch the concatanator
echo "launching pointcloud concatenator"
ros2 launch pointcloud_preprocessor pc_data_merge.launch.py 

# launch the recorder
echo "launching recorder"
ros2 bag record -o merged_pointcloud.bag /merged_points