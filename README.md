# COMP0240 Multi-Drone Challenge CW2

This repository contains the second coursework for the UCL Aerial Robotics Module COMP0240.

![scenario1](docs/scenario1.png)

This challenge has been developed on top of the aerostack2 platform 

## Challenge

This challenge revolves around swarming of drones and formation flight. We are asking you to investigate the difference in performance and application of decentralised swarm based approaches versus centralised approaches to performing formation flight with a group of 5 drones in a number of different scenarios. 

We have created a competition style course with 4 different stages to complete one after another.  

1. **Stage 1: Changing Formations**: Implementing the formation flight algorithms which have the ability to changing the formation periodically whilst maintaining a circular trajectory. 
2. **Stage 2: Window Traversal**: Using your formation flying methods attempt to maneouever your swarm of drones through two windows slits. 
3. **Stage 3: Forest Traversal**: Using your formation flying methods attempt to maneouever your swarm of drones through a forest of trees.
4. **Stage 4: Dynamic Obstacles**: Using your formation flying methods attempt to maneouever your swarm of drones through a set of dynamically moving obstacles.  

![schematic](docs/schematic.png)

You will be investigating, developing and testing your algorithm primarily in simulation. Hopefully we will get a chance to run these on the real crazyflies. Points are awarded on the completion of each stage, and performance within. The winning group will be the group with the most points at the end of the competition.

## Installation

To install this project, create your ros workspace and clone the following into the source repo:

```bash
mkdir -p /challenge_multi_drone_cw/src
cd /challenge_multi_drone_cw/src
git clone https://github.com/UCL-MSC-RAI-COMP0240/aerostack2.git
git clone https://github.com/UCL-MSC-RAI-COMP0240/as2_platform_crazyflie.git # If intending to also fly on the crazyflie
```

> *Note*: Aerostack is our own branch as has addition for multi-drone stuff. This means you should build it from scratch using our repository to reduce potential issues. 

> *Note*: Crazyflie AS2 interface has been augmented with LED Control


Also then clone the repository into the **src** folder:

```bash
cd /challenge_multi_drone_cw/src
git clone https://github.com/UCL-MSC-RAI-COMP0240/challenge_multi_drone.git
```

Please go to the root folder of the project and build it:

```bash
cd /challenge_multi_drone_cw
colcon build
```

Once built, all of the following commads can be run from inside the root of this repository

```bash
cd /challenge_multi_drone_cw/src/challenge_multi_drone
```

## Execution

### 1. Launch aerostack2 nodes for each drone
To launch aerostack2 nodes for each drone, execute once the following command:

```bash
./launch_as2.bash
```

The flags for the components launcher are:
- **-s**: scenario file to load from. Default is 'scenarios/scenario1.yaml'"
- **-w**: world config file to use as base template. Default is 'config_sim/config/world.yaml'"
- **-n**: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file
- **-s**: if set, the simulation will not be launched. Default launch simulation
- **-g**: launch using gnome-terminal instead of tmux. Default not set

### 2. Launch aerostack2 nodes for the ground station
To launch aerostack2 nodes for the ground station, execute once the following command:

```bash
./launch_ground_station.bash
```

The flags for the components launcher are:

- **-m**: multi agent. Default not set
- **-t**: launch keyboard teleoperation. Default not launch
- **-v**: open rviz. Default not launch
- **-r**: record rosbag. Default not launch
- **-n**: drone namespaces, comma separated. Default get from world description config file
- **-g**: launch using gnome-terminal instead of tmux. Default not set

### 3. Launch a mission
There are several missions that can be executed:

- **AS2 keyboard teleoperation control**: You can use the keyboard teleoperation launched with the ground station, using the flag `-t`:
  ```bash
  ./launch_ground_station.bash -t
  ```
  You can launch a **swarm of drones** with the flag `-m` and control them with the keyboard teleoperation, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m -t
  ```
- **AS2 Multi Drone**: You can explicitly specify the names of the drones to monitor
  ```bash
  ./launch_ground_station.bash -n drone0,drone1,drone2
  ```

  ```bash
  python3 mission_swarm.py -n drone0 drone1 drone2
  ```

- **AS2 Python API single drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission.py
  ```
- **AS2 Python API single drone mission using GPS**: You can execute a mission that used AS2 Python API with GPS, launching the mission with:
  ```bash
  python3 mission_gps.py
  ```
- **AS2 Python API swarm of drones mission**: You can execute a mission with a swarm of drones that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission_swarm.py
  ```
  You must launch a **swarm of drones** with the flag `-m`, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m
  ```
- **AS2 Mission Interpreter single drone mission**: You can execute a mission that used AS2 Mission Interpreter, launching the mission with:
  ```bash
  python3 mission_interpreter.py
  ```
- **AS2 Behavior Trees single drone mission**: You can execute a mission that used AS2 Behavior Trees, launching the mission with:
  ```bash
  python3 mission_behavior_tree.py
  ```

### 4. End the execution

If you are using tmux, you can end the execution with the following command:

```bash
./stop.bash
```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.

> Note sometimes you may find gazebo stays running for some reason. It is recommended that you install hte `htop` utility. Running htop use F4 to search for gazebo. Select the running gazebo process and press F9. Then select `SIGKILL` and that will kill it. 


## Developers guide

**Slightly out of date**

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.
