#!/bin/bash
# Function to send Ctrl+C command to a Terminator window by name
send_ctrl_c_to_window() {
    local window_name="$1"
    local window_id=$(xdotool search --name "$window_name" | tail -n 1)
    xdotool windowactivate --sync $window_id key Ctrl+c
    sleep 30 # Give it time to process the map and save it
    xdotool windowactivate --sync $window_id key Ctrl+Shift+Q
}

# Experiment 1
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_file:="maze_drift_evaluation_reconstruction_planner.csv" planner_config:="planners/reconstruction_planner.yaml"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" &

# sleep 3660  # Wait an hour
    
# # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40


# Experiment 2
#  terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
#  roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_file:="maze_drift_evaluation_exploration_planner.csv" planner_config:="planners/exploration_planner.yaml"'" &
#  sleep 7
#  terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
#  roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

#  sleep 3660  # Wait an hour
#  # sleep 140 # Wait 2 minutes

#  # Send Ctrl+C command to the second Terminator tab by name
#  send_ctrl_c_to_window "rovioli"
#  sleep 40


# # Experiment 3
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_file:="maze_drift_evaluation_example_config.csv" planner_config:="planners/example_config.yaml"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" &

# # #sleep 3660  # Wait an hour
# sleep 140 # Wait 2 minutes
    
# # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40

# Experiment 4
terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_file:="maze_drift_evaluation_drift_aware_planner.csv" planner_config:="planners/drift_aware_planner.yaml"'" &
sleep 7
terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

sleep 3660  # Wait an hour
# sleep 140 # Wait 2 minutes
   
# Send Ctrl+C command to the second Terminator tab by name
send_ctrl_c_to_window "rovioli"

sleep 40






