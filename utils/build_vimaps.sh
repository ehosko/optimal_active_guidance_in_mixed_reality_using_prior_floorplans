#!/bin/bash

# activate the maplab ROS environment
source /home/michbaum/Projects/maplab/devel/setup.bash # Workstation

# Define the path to the directory containing your rosbag files
rosbag_dir="/home/michbaum/Projects/optag_EH/data/sim_bags/maze_slowdown_gt_split/"

# Define the package containing your launch file
package="maplab_node"

# Define the name of your launch file
launch_file="rosbag-maplab-node-w-rovioli.launch"

# Change to the rosbag directory
cd "$rosbag_dir"

# Get a list of all rosbag files in the directory
rosbag_files=(*.bag)

echo "Found ${#rosbag_files[@]} rosbag files in $rosbag_dir"

# Loop through each rosbag file
for rosbag_file in "${rosbag_files[@]}"; do
    # Extract the number from the filename using awk
    number=$(echo "$rosbag_file" | awk -F '[^0-9]*' '$0=$2')

    # Check if the number is greater than 174
    if (( number >= 0 )); then
        echo "Playing rosbag: $rosbag_file"
        
        # Start the launch file in the background
        roslaunch "$package" "$launch_file" &
        launch_pid=$!

        sleep 7
        
        # Start playing the rosbag
        rosbag play "$rosbag_file" /cam0/image_raw:=/isaac/isaac_ros_client/isaac_gray_image_out /imu0:=/firefly/imu
        
        # Stop the launch file
        kill -INT $launch_pid
        wait $launch_pid
    else
        echo "Skipping rosbag: $rosbag_file (Number is not greater equal 0)"
    fi
done
