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
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_reconstruction_eval/" planner_config:="planners/reconstruction_planner.yaml"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

# sleep 120  # Wait an hour
# send_ctrl_c_to_window "rovioli"

# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_reconstruction_eval/" planner_config:="planners/reconstruction_planner.yaml" use_floorplan:="false"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

# #sleep 3660  # Wait an hour

# #sleep 2500 # Wait 40+ minutes
# sleep 660   
# # # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40


# # Experiment 2
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_exploration_eval/" planner_config:="planners/exploration_planner.yaml" use_floorplan:="false"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

# # sleep 3660  # Wait an hour
# # # #  # sleep 140 # Wait 2 minutes
# # sleep 2500

# sleep 660   

# # #  # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"
# sleep 40


# # Experiment 3
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_example_eval/" planner_config:="planners/example_config.yaml" use_floorplan:="false"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" &

# # sleep 3660  # Wait an hour
# # sleep 140 # Wait 2 minutes
# sleep 660 

# # # # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40

# TODO (ehosko) : ADD make dir statement if folder does not exist

# # Experiment 4
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_drift_aware_eval/" planner_config:="planners/drift_aware_planner.yaml" use_floorplan:="false"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

# sleep 3660  # Wait an hour
# #sleep 1880  # Wait half an hour
# #sleep 400 # Wait 6+ minutes
# # sleep 660 
# # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40

# # Experiment 5
# terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
# roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_drift_aware_floorplan_eval/" planner_config:="planners/floorplan_drift_aware_planner.yaml" use_floorplan:="true"'" &
# sleep 7
# terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
# roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

# sleep 3660  # Wait an hour
# # sleep 1880  # Wait half an hour
# # sleep 400 # Wait 6+ minutes
# # sleep 660  

# # Send Ctrl+C command to the second Terminator tab by name
# send_ctrl_c_to_window "rovioli"

# sleep 40

# Experiment 6
terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_drift_aware_TSP_eval/" planner_config:="planners/drift_aware_planner_TSP.yaml" use_floorplan:="false"'" &
sleep 7
terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

sleep 3660  # Wait an hour
# sleep 1880  # Wait half an hour
# sleep 400 # Wait 6+ minutes
# sleep 660  

# Send Ctrl+C command to the second Terminator tab by name
send_ctrl_c_to_window "rovioli"

sleep 40

# Experiment 7
terminator -T "planner" -e "bash -c 'source /home/michbaum/Projects/optag_EH/devel/setup.bash && 
roslaunch active_3d_planning_app_reconstruction run_experiment_isaac_rovioli.launch output_folder:="/home/michbaum/Projects/optag_EH/data/maze_drift_aware_floorplan_TSP_eval/" planner_config:="planners/drift_aware_planner_floorplan_TSP.yaml" use_floorplan:="true"'" &
sleep 7
terminator -T "rovioli" -e "bash -c 'source /home/michbaum/Projects/maplab/devel/setup.bash && 
roslaunch maplab_node optag-maplab-node-w-rovioli.launch; bash'" 

sleep 3660  # Wait an hour
# sleep 1880  # Wait half an hour
# sleep 400 # Wait 6+ minutes
# sleep 660  

# Send Ctrl+C command to the second Terminator tab by name
send_ctrl_c_to_window "rovioli"

sleep 40



