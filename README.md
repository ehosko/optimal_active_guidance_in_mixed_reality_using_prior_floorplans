# optimal\_active\_guidance\_in\_mixed\_reality\_using\_prior\_floorplans
**optimal\_active\_guidance\_in\_mixed\_reality\_using\_prior\_floorplans** is a Semester Thesis project based upon the modular sample based active path planning approach [Mav Active 3d Planning](https://github.com/ethz-asl/mav_active_3d_planning/).

The goal of this project is:
- To incorporate strong prior knowledge of an environment in the form of a floorplan into the active path planning approach.
- To develop a novel approach to incorporate the floorplans in a suitable format into the information gain formulation.
- To evaluate the system with regards to current state of the art approaches that do not rely on prior knowledge.
- To build and showcase a simple mixed reality application that uses the developed system to guida a user to explore an environment.
  
# Setup

The project was intended to run on a Windows machine with WSL2 support. One would run the Simulation environment in Windows, whilst building & running the code in WSL2 on an Ubuntu 18.04 build.

## Windows

**Unreal**

1. Download Unreal 4.25

2. Download [Maze Environment]()
<!-- TODO: Make somehow available -->

3. Open the project in the Unreal 4.25

4. For a performance boost, go into the editor settings and disable the "Use Less CPU when in Background" option.

4. Play the game

**Nvidia Omniverse Isaac Sim**

TBD

## WSL 2

1. Setup mav_active_3d_planning package by following the documentation in [mav_active_3d_planning](mav_active_3d_planning/)  
(**Important**: Follow the readme in this repository instead of the one in the original mav_active_3d_planning repository. Dependencies and build instructions have changed.)

2. Setup unrealcv for Unreal 4.25 environment by substituting original unreal_cv_ros dependency with [michbaum/unreal_cv_ros: Unreal CV ROS Perception Simulator (github.com)](https://github.com/michbaum/unreal_cv_ros)
(This has already been taken care of if you followed the instructions above)

3. Setup IP of unreal_cv_ros
```
rosed unreal_cv_ros unreal_ros_client.py
# change ip of unreal_ros_client to its host ip: client = Client(('HOST_IP',9000))
IMPORTANT: This is really the host ip of your machine, not(!) localhost
```

# Debugging

## VSCode

To be able to run the debugger on Python 2.7 code utilized in ROS, you need to downgrade your Pyhton extension to version: v2021.9.124654278

# Run Experiments

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

TBD


