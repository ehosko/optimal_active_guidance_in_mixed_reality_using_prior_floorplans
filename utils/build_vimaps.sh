#!/bin/bash

# activate the maplab ROS environment
source /home/michbaum/maplab_ws/devel/setup.bash

# Define the path to the directory containing your rosbag files
rosbag_dir="/home/michbaum/optag_ws/data/sim_bags/query_trajectory_split"

# Define the package containing your launch file
package="maplab_node"

# Define the name of your launch file
launch_file="maze-maplab-node-w-rovioli.launch"

# Change to the rosbag directory
cd "$rosbag_dir"

# Get a list of all rosbag files in the directory
rosbag_files=(*.bag)

# Loop through each rosbag file
for rosbag_file in "${rosbag_files[@]}"; do
    echo "Playing rosbag: $rosbag_file"
    
    # Start the launch file in the background
    roslaunch "$package" "$launch_file" &
    launch_pid=$!
    
    # Start playing the rosbag
    rosbag play "$rosbag_file"
    
    # Stop the launch file
    kill -INT $launch_pid
    wait $launch_pid
done
