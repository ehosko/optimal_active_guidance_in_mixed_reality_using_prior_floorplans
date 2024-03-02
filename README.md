# Optimal Active Guidance Using Prior Floorplans
**Optimal Active Guidance Using Prior Floorplans** is the main framework of a semester project extending the work of a former semester thesis [optimal_active_guidance_in_mixed_reality_using_prior_floorplans](https://github.com/michbaum/optimal_active_guidance_in_mixed_reality_using_prior_floorplans). The project is initially based upon the modular sample-based active path planning approach [Mav Active 3d Planning](https://github.com/ethz-asl/mav_active_3d_planning/).
  
# Setup

The project is built and run on Ubuntu 18.04 using ROS Melodic.

## Install

**Nvidia Omniverse Isaac Sim**

This project utilizes Isaac Sim as a graphics simulator. To that end, a custom plugin is used: [Isaac CV Ros](https://github.com/michbaum/isaac_cv_ros/), which is installed automatically through later steps.

1. Isaac Sim has to be installed by following the official documentation: [Isaac Sim Install](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#isaac-sim-setup-native-workstation-launcher).
2. After successful installation, the Ros bridge extension needs to be enabled : [Isaac Sim Ros bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension)

**mav_active_3d_planning package**

1. Setup mav_active_3d_planning package by following the documentation in [mav_active_3d_planning](mav_active_3d_planning/)  
(**Important**: Follow the readme in this repository instead of the one in the original mav_active_3d_planning repository. Dependencies and build instructions have changed.)

# Debugging

## VSCode

To be able to run the debugger on Python 2.7 code utilized in ROS, you need to downgrade your Python extension to version: v2021.9.124654278

# Data 

Two simulation environments, a maze, and a warehouse, are made available for Isaac Sim : [Environments](https://polybox.ethz.ch/index.php/s/SPR7wtBlBgyCn26)

# Test Installation

1. Launch Isaac Sim and load the maze environment. Press the play button in Isaac Sim.
2. Launch the sample experiment to verify a correct setup.

```
roslaunch active_3d_planning_app_reconstruction example_isaac.launch planner_config:=planners/example_config.yaml
```
You should see the Isaac Sim Game Play changing views, and rviz showing the planned trajectory and moving agents.

<!-- # Test Installation

## Unreal

1. Open the project (e.g., Maze) in Unreal 4.25 as explained above

1. Launch an active_3d_planning experiment for the project (e.g., Maze). You can currently choose between 3 different planners: example_config (simple frontier based), exploration_planner (RRT* based) and reconstruction_planner (from the original mav_active_3d_planning repository and paper).

Run this command to check for a correct setup:

```
roslaunch active_3d_planning_app_reconstruction example.launch planner_config:=planners/example_config.yaml
```

Run this command to collect data of a run with a certain planner (here exploration_planner):

```
roslaunch active_3d_planning_app_reconstruction run_experiment.launch planner_config:=planners/exploration_planner.yaml data_directory:=/path/to/data/directory
```

2. You can see Unreal Game Play changing views, and rviz shows planned trajectory and moving agents. If the ros node crash, try rebuilding "unreal_cv_ros" package

3. Note that - to be able to use the footage for later map building with a VIO system - the simulation time has been slowed down tenfold. To change this time factor, open
the gazebo_empty.world XML file under catkin_ws/src/unreal_cv_ros/unreal_cv_ros/content/ and change the real_time_update_rate on line 33 (! not the max_step_size !) alongside
the real_time_factor (needs to be max_step_size * real_time_update_rate). With a real-time factor of 1.0, you can expect around 1.5hz of gray image data.

## Isaac Sim

TBD -->


