# Follow the Robot: Modeling Coupled Human-Robot Dyads During Navigation
Licensed under MIT License - Transportation, Bots, and Disability lab, Carnegie Mellon University


This is accomponanied repository for the system described in the IROS 2019, *Follow the Robot: Modeling Coupled Human-Robot Dyads During Navigation* by Amal Nanavati, Xiang Zhi Tan, Joe Connolly and Aaron Steinfeld.

This is not a production version of the code and also contains large number of dependencies on other project. Please use it at your own risk.

## Description

This repository contains the final coupling model (along with the trained parameters, preset as default rosparam values) in `podi_navigation_helpers`, as well as the coupled global planner in `podi_robot_human_coupled_planner`. For the coupling model, there is both a C++ class which can be called from another class (e.g. the global planner incorporating the coupling model into its forward-simulation) and a ROS node that could be run in-real-time (e.g. to predict where the human is as the robot is executing its trajectory).

Note that we changed the interface of the global planner to account for multiple goal poses. It should be easy to change it back so that this planner can be readily plugged into the standard ROS navigation stack.

## Inquiries

If you have any questions or would like additional code released (for example, the code to actually train the coupling model or the code for the simulation experiment we ran) please contact Amal Nanavati <arnanava@alumni.cmu.edu>.
