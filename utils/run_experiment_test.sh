#!/bin/bash
# Function to send Ctrl+C command to a Terminator window by name
send_ctrl_c_to_window() {
    local window_name="$1"
    local window_id=$(xdotool search --name "$window_name" | tail -n 1)
    xdotool windowactivate --sync $window_id key Ctrl+c
    sleep 30 # Give it time to process the map and save it
    xdotool windowactivate --sync $window_id key Ctrl+Shift+Q
}

folder_name="/home/michbaum/Projects/optag_EH/data/maze_test_eval/run_1/"
mkdir -p $folder_name

# Test Experiment
terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="$folder_name" planner_config:="planners/reconstruction_planner.yaml"'" &
sleep 7
terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
roslaunch maplab_node optag-maplab-node-w-rovioli.launch output_folder:="$folder_name"; bash'" 


sleep 400 # Wait 6+ minutes

# Send Ctrl+C command to the second Terminator tab by name
send_ctrl_c_to_window "rovioli"

sleep 40