#!/bin/bash
# Function to send Ctrl+C command to a Terminator window by name
send_ctrl_c_to_window() {
    local window_name="$1"
    local window_id=$(xdotool search --name "$window_name" | tail -n 1)
    xdotool windowactivate --sync $window_id key Ctrl+c
    sleep 5 # Give it time to process the map and save it
    xdotool windowactivate --sync $window_id key Ctrl+Shift+Q
}

# activate the maplab ROS environment
source /home/michbaum/Projects/maplab/devel/setup.bash

# Define the path to the directory containing your rosbag files
# rosbag_example_config="/home/michbaum/Projects/optag/data/sim_bags/maze_slowdown_drifty_example_config.bag" # This should be the SECOND sim bag
#rosbag_exploration_planner="/home/michbaum/Projects/optag_EH/data/sim_bags/maze_slowdown_drifty_exploration_planner.bag" # This should be the FIRST sim bag
# rosbag_reconstruction_planner="/home/michbaum/Projects/optag/data/sim_bags/maze_slowdown_drifty_reconstruction_planner.bag"
# rosbag_drift_aware_planner="/home/michbaum/Projects/optag_EH/data/sim_bags/sim_bags_2024-02-16-09-29-42.bag"
# rosbag_drift_aware_floorplan_planner="/home/michbaum/Projects/optag_EH/data/sim_bags/sim_bags_2024-02-16-09-29-42.bag"
# rosbag_drift_aware_TSP_planner="/home/michbaum/Projects/optag_EH/data/sim_bags/sim_bags_2024-02-16-09-29-42.bag"
# rosbag_drift_aware_floorplan_TSP_planner="/home/michbaum/Projects/optag_EH/data/sim_bags/sim_bags_2024-02-16-09-29-42.bag"

# Define the package containing your launch file
package="maplab_node"

# Define the name of your launch file
launch_file="rosbag-maplab-node-w-rovioli.launch"

#-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_example_config"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# rosbag play "$rosbag_example_config" -u 920

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

#-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_example_config"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# rosbag play "$rosbag_example_config" -s 880

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

#-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_exploration_planner"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# # rosbag play "$rosbag_exploration_planner" -u 920
# rosbag play "$rosbag_exploration_planner"

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

#-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_exploration_planner"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# rosbag play "$rosbag_exploration_planner" -s 880

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

#-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_reconstruction_planner"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# rosbag play "$rosbag_reconstruction_planner" -u 920

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

# #-----------------------------------------------------------------------

# echo "Playing rosbag: $rosbag_reconstruction_planner"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# rosbag play "$rosbag_reconstruction_planner" -s 880

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10

#-----------------------------------------------------------------------
# echo "Playing rosbag: $rosbag_drift_aware_planner"
        
# # Start the launch file in the background
# roslaunch "$package" "$launch_file" &
# launch_pid=$!

# sleep 7

# # Start playing the rosbag
# # rosbag play "$rosbag_exploration_planner" -u 920
# rosbag play "$rosbag_drift_aware_planner" -u 120
# # rosbag play "$rosbag_drift_aware_planner"

# # Stop the launch file
# kill -INT $launch_pid
# wait $launch_pid

# sleep 10


# Set Environment
Environment="maze"
# Environment="warehouse"

# Set Planners
# planners=("drift_aware" "drift_aware_floorplan" "drift_aware_floorplan_TSP" "drift_aware_TSP")
# planners=("reconstruction_planner" "drift_aware_planner" "drift_aware_floorplan_planner" "drift_aware_floorplan_TSP_planner" "drift_aware_TSP_planner" "exploration_planner" "example_config")
planners=("drift_aware_floorplan_TSP_planner")


number_runs=5

for planner in "${planners[@]}"
do
    for i in $(seq 5 $number_runs)
    do

        folder_name="/home/michbaum/Projects/optag_EH/data/${Environment}/${planner}/run_${i}/"
        mkdir -p $folder_name

        # terminator -T "eval_${planner}_${i}_1" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
        # roslaunch active_3d_planning_app_reconstruction run_experiment_rosbag.launch output_folder:="$folder_name" part:="1"; bash'"

        # sleep 7

        rosbag="/home/michbaum/Projects/optag_EH/data/sim_bags/${planner}_${i}.bag"

        echo "Playing rosbag: $rosbag first part"

        # Start the launch file in the background
        # roslaunch "$package" "$launch_file" output_folder:="$folder_name" &
        # launch_pid=$!

        # sleep 7

        # # Start playing the rosbag
        # rosbag play "$rosbag" -u 920

        # # Stop the launch file
        # kill -INT $launch_pid
        # wait $launch_pid

        # sleep 10

        # # Send Ctrl+C command to the second Terminator tab by name
        # send_ctrl_c_to_window "eval_${planner}_${i}_1"

        # sleep 10
        #-----------------------------------------------------------------------

            
        terminator -T "eval_${planner}_${i}_2" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
        roslaunch active_3d_planning_app_reconstruction run_experiment_rosbag.launch output_folder:="$folder_name" part:="2"; bash'"

        sleep 7

        echo "Playing rosbag: $rosbag second part"

        # Start the launch file in the background
        roslaunch "$package" "$launch_file" output_folder:="$folder_name" &
        launch_pid=$!

        sleep 7

        # Start playing the rosbag
        rosbag play "$rosbag" -s 880

        # Stop the launch file
        kill -INT $launch_pid
        wait $launch_pid

        sleep 10

        # Send Ctrl+C command to the second Terminator tab by name
        send_ctrl_c_to_window "eval_${planner}_${i}_2"

        sleep 60

    done
done