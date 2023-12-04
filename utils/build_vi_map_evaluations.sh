#!/bin/bash

# activate the maplab ROS environment
# source /home/michbaum/Projects/maplab/devel/setup.bash # Workstation
source /home/michbaum/maplab_ws/devel/setup.bash # Laptop

# Define the path to the directory containing your query maps
vimap_dir="/home/michbaum/maplab_ws/data/vimaps/warehouse/warehouse_query_maps_without_windows" # Warehouse
# vimap_dir="/home/michbaum/maplab_ws/data/vimaps/maze/maze_query_maps" # Maze

to_be_evaluated_map="/home/michbaum/maplab_ws/data/vimaps/warehouse/warehouse_slowdown_drifty_reconstruction_planner_without_windows" # CHANGE


# Change to the rosbag directory
cd "$vimap_dir"

# Get a list of all vimap directories in the current folder
vimap_folders==$(find . -maxdepth 1 -type d ! -path . -exec basename {} \;)

# Loop through each rosbag file
for vimap_folder in $vimap_folders; do
    # Extract the number from the folder names using awk
    number=$(echo "$vimap_folder" | awk -F '[^0-9]*' '$0=$2')

    # Start the maplab console
    rosrun maplab_console maplab_console -v 1 &
    launch_pid=$!

    sleep 5

    # TODO: Apparently not possible?

    # Load the respective maps
    echo -e 'load --map-folder "$to_be_evaluated_map"'

    # Load the respective query map
    echo load --map-folder "$vimap_folder"

    # Merge the two maps
    echo join_all_maps --target_map_key "warehouse_reconstruction_planner" # CHANGE

    sleep 200

    # # Check if the number is greater than 174
    # if (( number >= 0 )); then
    #     echo "Playing rosbag: $rosbag_file"
        
    #     # Start the launch file in the background
    #     roslaunch "$package" "$launch_file" &
    #     launch_pid=$!

    #     sleep 7
        
    #     # Start playing the rosbag
    #     rosbag play "$rosbag_file"
        
    #     # Stop the launch file
    #     kill -INT $launch_pid
    #     wait $launch_pid
    # else
    #     echo "Skipping rosbag: $rosbag_file (Number is not greater equal 0)"
    # fi
done
