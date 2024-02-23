#!/bin/bash

# activate the maplab ROS environment
source /home/michbaum/Projects/maplab/devel/setup.bash # Workstation

cd /home/michbaum/Projects/maplab/

# echo "Running re-localization evaluation for drift aware planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-eval.yaml 

# echo "Running re-localization evaluation for drift aware floorplan planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-floorplan-eval.yaml

# echo "Running re-localization evaluation for TSP planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-TSP-eval.yaml

# echo "Running re-localization evaluation for floorplan TSP planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-floorplan-TSP-eval.yaml

echo "Running re-localization evaluation for reconstruction planner"
rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-eval-reconstruction.yaml 

# echo "Running re-localization evaluation for exploration planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-eval-exploration.yaml 

# echo "Running re-localization evaluation for example planner"
# rosrun maplab_console re_localization_evaluation --batch_control_file /home/michbaum/Projects/maplab/src/maplab/applications/maplab-console/share/optag-re-localization-eval-example.yaml 

