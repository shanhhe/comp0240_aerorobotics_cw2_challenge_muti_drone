# COMP0240 Multi-Drone Challenge CW2

This repository contains the second coursework for the UCL Aerial Robotics Module COMP0240.

![scenario1](docs/scenario1.png)

This challenge has been developed on top of the aerostack2 platform 

## Challenge

This challenge revolves around swarming of drones and formation flight. We are asking you to investigate the difference in performance and application of decentralised swarm based approaches versus centralised approaches to performing formation flight with a group of 5 drones in a number of different scenarios. 

### Swarming and Formation Flight

**Swarming** refers to the collective behavior of multiple agents (drones) operating together using local rules without a centralized controller. These behaviors emerge from interactions between individual drones and their environment.

Swarm robotics takes inspiration from nature—such as birds, fish, and insects—to design scalable, flexible, and robust robotic systems. Swarm behaviors are often decentralized and self-organized, meaning that individual drones follow simple local rules that collectively result in a global pattern of behavior.

Common decentralized swarm control strategies:

- Boids Model (Flocking Behavior) – Uses three simple rules: separation, alignment, and cohesion.
- Potential Fields – Assigns virtual attractive/repulsive forces to goals and obstacles to guide movement.
- Reinforcement Learning-Based Swarms – Uses machine learning to optimize local decision-making.
- Bio-Inspired Methods – Such as pheromone-based navigation or genetic algorithms.

**Formation flight** is a more structured approach to multi-agent coordination where drones maintain a specific geometric arrangement while moving. Unlike general swarming, formation flying often requires precise positioning and coordination.

Common formation flight strategies:

- Centralized Approaches
    - Leader-Follower – One drone acts as the leader while others maintain a relative position.
    - Multi-Agent Path Planning (MAPF) – Centralized planning optimizes collision-free paths.
    - Virtual Structures – The entire formation is treated as a rigid body and controlled as one unit.
- Decentralized Approaches
    - Boids with Formation Constraints – Similar to flocking but with additional formation control.
    - Consensus-Based Control – Drones agree on formation changes based on local communication.
    - Distributed Potential Fields – Drones use attraction/repulsion forces while maintaining formation.

Swarming and formation flight have numerous real-world applications across various industries. 

- In aerial surveillance and search-and-rescue, swarming drones can quickly cover large areas, scan for missing persons, or assess disaster zones without relying on a single point of failure. 
- In logistics and delivery, drone swarms can efficiently transport packages in coordinated formations, optimizing airspace usage and reducing delivery times. 
- In environmental monitoring, swarms of drones can track wildlife migrations, detect deforestation, or monitor air and water quality over vast regions. 
- In entertainment and art, synchronized drone light shows use precise formation flight to create complex aerial displays, offering an innovative alternative to fireworks. 

These examples highlight the versatility of swarm robotics in enhancing efficiency, scalability, and adaptability in real-world operations.

### Your Challenge

We have created a competition style course with 4 different stages to complete one after another.  

1. **Stage 1: Changing Formations**: 
    - Implementing the formation flight algorithms which have the ability to changing the formation periodically whilst maintaining a circular trajectory. 
    - Cmpare different formation shapes (Line, V-shape, Diamond, Circular Orbit, Grid, Staggered)

2. **Stage 2: Window Traversal**: 
    - Using your formation flying methods attempt to maneouever your swarm of drones through two narrow windows slits. 
    - Consider how to split, rejoin, or compress the formation to pass through gaps.

3. **Stage 3: Forest Traversal**: 
    - Using your formation flying methods attempt to maneouever your swarm of drones through a forest of trees.
    - Your swarm should avoid collisions and maintain efficiency in movement.

4. **Stage 4: Dynamic Obstacles**: 
    - Using your formation flying methods attempt to maneouever your swarm of drones through a set of dynamically moving obstacles.  
    - You may need adaptive formation control to respond to changes in real time.

![schematic](docs/schematic.png)

You will be investigating, developing and testing your algorithm primarily in simulation. Hopefully we will get a chance to run these on the real crazyflies. Points are awarded on the completion of each stage, and performance within. The winning group will be the group with the most points at the end of the competition.

## Installation

To install this project, create your ros workspace and clone the following into the source repo:

```bash
mkdir -p challenge_multi_drone_cw/src
cd challenge_multi_drone_cw/src
git clone https://github.com/UCL-MSC-RAI-COMP0240/aerostack2.git
git clone https://github.com/UCL-MSC-RAI-COMP0240/as2_platform_crazyflie.git # If intending to also fly on the crazyflie
```

> *Note*: Aerostack is our own branch as has addition for multi-drone stuff. This means you should build it from scratch using our repository to reduce potential issues. 

> *Note*: Crazyflie AS2 interface has been augmented with LED Control


Also then clone the repository into the **src** folder:

```bash
cd challenge_multi_drone_cw/src
git clone https://github.com/UCL-MSC-RAI-COMP0240/challenge_multi_drone.git
```

Please go to the root folder of the project and build it:

```bash
cd challenge_multi_drone_cw
colcon build
```

> **Note**: This repo contains a gazebo plugin to simulate the crazyflie LED deck. There have been some reports of issues with JSONCPP. If you get this issue, I have provided a modified `cmake` file to replace the existing one.
> 
> ```sudo cp challenge_multi_drone/config_sim/gazebo/plugins/led_ring_plugin/jsoncpp-namespaced-targets.cmake /usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncpp-namespaced-targets.cmake```
>
> Hopefully the `/usr/lib` directory is correct, if not then you will need to find the location of your jsoncpp library. 
>
> See [This Issue for more details](https://github.com/UCL-MSC-RAI-COMP0240/challenge_multi_drone/issues/1)


Once built, all of the following commads can be run from inside the root of this repository

```bash
cd challenge_multi_drone_cw/src/challenge_multi_drone
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
- **-c**: if set, the real crazyflie interface will be launched instead of the simulation. Defaults to false"
- **-g**: launch using gnome-terminal instead of tmux. Default not set

### 2. Launch aerostack2 nodes for the ground station
To launch aerostack2 nodes for the ground station, execute once the following command:

```bash
./launch_ground_station.bash
```

The flags for the components launcher are:

- **-m**: multi agent. Default not set
- **-c**: if set, the real crazyflie interface will be launched instead of the simulation. Defaults to false"
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
