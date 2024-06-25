# High-Speed Obstacle Avoidance in Unstructured Environments
This repository contains a simulation environment in which to test obstacle avoidance algorithms on MAVs. The simulation environment is built using a proposed environmental complexity framework. The framework is used to define how the simulation environment is built. The simulation environment is built such that it increases in complexity as the MAV navigates the course. The are two courses, both of which contain ten levels, such that each level is incrementally more complex. An obstacle avoidance algorithm called FALCO is used to autonomously navigate the simulation environment. The MAV starts at a starting area and moves through each level towards the goal position. 

## Setup Instructions

#### <u>Operating System</u>
Ubuntu 20.04 was used for this project so the instructions below are for setup on Ubuntu 20.04. 

#### <u>Install Nvidia Drivers</u>
If you have an Nvidia graphics card ensure that the latest proprietary drivers are installed. 
This helps with performance when the simulation is being run or when inside the Unreal Editor.

#### <u>Install Unreal Engine 5.2</u>
Note: To download Unreal Engine an Epic Games account is required, so you may need to create one.

Download UE 5.2 [here](https://www.unrealengine.com/en-US/auth?state=https://www.unrealengine.com/en-US/linux)

Create a directory and unzip the UE5.2 folder into that directory.

Go into the directory and open the Unreal Engine editor to make sure that Unreal Engine works.
```bash
# create Directory
mkdir ~/unreal_engine_5_2

# go into directory
cd ~/unreal_engine_5_2/Engine/Binaries/Linux

# open the Unreal Editor
./UnrealEditor
```
After the Unreal Engine Editor opens and is working, close the Unreal Engine editor.
Note: On Ubuntu the editor takes a few minutes to open. (Around 2min on the computer used for this project) 

#### <u>Install ROS Noetic</u>
Install the recommended version of ROS Noetic (instructions can be found [here](https://wiki.ros.org/noetic/Installation/Ubuntu))

#### <u>Install AirSim/Colosseum</u>
Colosseum is a fork of AirSim and was used because AirSim was archived in the middle of 2022. Colosseum provides a stable release of AirSim using Unreal Engine 5 on Ubuntu 20.04.

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
./setup.sh
./build.sh
```

Build the ROS package.
```bash
cd ros
catkin_make #or catkin build
```
If you have problems building the ROS package refer to the ROS section of the AirSim [documentation](https://microsoft.github.io/AirSim/airsim_ros_pkgs/).

Install AirSim's python API (airsim) and msgpack-rpc-python
```bash
pip3 install msgpack-rpc-python
pip3 install airsim
```

If you are having problems installing Colosseum, refer to the AirSim [documentation](https://microsoft.github.io/AirSim/build_linux/).


Copy "settings.json" from the root of this repo to the following directory "~/Documents/AirSim/".
This file is used to configure the MAV in AirSim for use in the simulation environment. 

```bash
cp path/to/hs_obs_avoid_in_us_envs/settings.json ~/Documents/AirSim/
```

#### <u>Install FALCO</u>
FALCO is a collision avoidance algorithm which you can read more about [here](https://www.ri.cmu.edu/publications/falco-fast-likelihood%E2%80%90based-collision-avoidance-with-extension-to-human%E2%80%90guided-navigation/).

Installation instructions for FLACO can be found [here](https://github.com/caochao39/aerial_navigation_development_environment/tree/melodic-noetic).

A lot of fine-tuning has been done on the existing default parameters. While the original repo is referenced in the link above the "aerial_navigation_development_environment" folder in this repo contains the parameters used for experimentation. Aside from the updated parameters, the logging node written for the simulation environment can be found in "aerial_navigation_development_environment/src/airsim_utils/scripts/log_metrics.py"  

The logging node captures all the positional and timing information as the MAV moves through the simulation environment.

To build the tuned FALCO repo contained in this repo:
```bash
cd path/to/aerial_navigation_development_environment/

catkin_make
```

#### <u>Download Simulation Environment</u>
The simulation environment can be found [here](https://drive.google.com/drive/folders/1Mc4ExDlhbInf63pH7voFm6nMLcstyY6h?usp=drive_link).

Note that this is the Unreal Engine project which contains the simulation environment. 

The file is a tar.bz2 file and can be unzipped as follows: 

```bash
tar -xjvf sim_env.tar.bz2
```

## Running the Simulation in UE using FALCO

Open the simulation environment in Unreal Engine Editor.

Open a new terminal.

```bash
# go unreal engine directory
cd ~/unreal_engine_5_2/Engine/Binaries/Linux

# open the sim_env project
./UnrealEditor ~/your_path/hs_obs_avoid_in_us_envs/sim_env/sim_env.uproject
```

Click on the green play button in the Unreal Editor toolbar to run the simulation. 

Run the AirSim ROS Node.

Start a new terminal.
```bash
# run the AirSim ROS node 
cd ~/your_path/hs_obs_avoid_in_us_envs/Colosseum/ros/ && source devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node.launch
```

Run FALCO.

Start a new terminal.
```bash
# launch FALCO
cd ~/your_path/hs_obs_avoid_in_us_envs/aerial_navigation_development_environment/ && source devel/setup.sh && roslaunch vehicle_simulator system_airsim.launch
```
Run the logging node to begin recording MAV data. 

Start a new terminal.
```bash
# run the logging node
cd ~/your_path/aerial_navigation_development_environment/src/airsim_utils/scripts/ && python ./log_metrics.py
```

The simulation should now be running and in the Pillars Course. To run the simulation in the Trees course, move the PlayerStart position in the Unreal Engine Outliner to the center of the start position in the Trees course.

## Experiment Data
All raw data logged and processed for both courses can be found in the **raw_experiment_data** folder.