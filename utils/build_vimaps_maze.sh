#!/bin/bash

# activate the maplab ROS environment
source /home/michbaum/Projects/maplab/devel/setup.bash

# Define the path to the directory containing your rosbag files
rosbag_example_config="/home/michbaum/Projects/optag/data/sim_bags/maze_slowdown_drifty_example_config.bag"
rosbag_exploration_planner="/home/michbaum/Projects/optag/data/sim_bags/maze_slowdown_drifty_exploration_planner.bag"
rosbag_reconstruction_planner="/home/michbaum/Projects/optag/data/sim_bags/maze_slowdown_drifty_reconstruction_planner.bag"

# Define the package containing your launch file
package="maplab_node"

# Define the name of your launch file
launch_file="rosbag-maplab-node-w-rovioli.launch"

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_example_config"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_example_config" -u 920

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_example_config"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_example_config" -s 880

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_exploration_planner"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_exploration_planner" -u 920

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_exploration_planner"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_exploration_planner" -s 880

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_reconstruction_planner"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_reconstruction_planner" -u 920

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10

#-----------------------------------------------------------------------

echo "Playing rosbag: $rosbag_reconstruction_planner"
        
# Start the launch file in the background
roslaunch "$package" "$launch_file" &
launch_pid=$!

sleep 7

# Start playing the rosbag
rosbag play "$rosbag_reconstruction_planner" -s 880

# Stop the launch file
kill -INT $launch_pid
wait $launch_pid

sleep 10