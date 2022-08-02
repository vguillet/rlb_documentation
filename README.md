# Introduction
The **R**obot **L**ab **B**ridge (**RLB**) framework is a generic robotics muti-agent interface based on ROS2 Foxy. 

It aims at providing a framework and tools for running and testing single or multi-agent control and/or task allocation algorithms, using a combination of real and simulated data sources. The toolkit provides all necessary packages for fullfuling most simple experimental cases, from pre-implemented goto control laws for higher level coordination experiments to lower-level control. It also provides visualisation tools, a set of emulator for simulating robots/comunications/etc.

Each package in the RLB toolkit was build to be independent of each other, with the exception of the RLB_utils (*more in the installation section*).

The documentation contains an **Overview** of the framework architecture, an **Installation** guide, and a number of **Tutorials** covering various aspects of the framework.

# Overview

## General RLB structure 
Each RLB run is functionaly split into a few key structural elements, each responsible for different aspect of the run. Not all elements are necessary depending on the experiment type, and any element can be swaped for a custom one if necessary as long as the **key topics** are maintained. 

Notable elements include:
- **Agent instances**: Responsible of emiting agent's pose to */Agent/pose*
- A **source simulation**: Responsible of emitting goals to */goals_backlog*. Used with the rlb_controller package for experimenting with task allocation systems.
- **Controllers**: Responsible of controlling agents, such as performing gotos etc… Controllers emit *Twist* messages to the agent's */cmd_vel* 
- **Visualisers**: Responsible of agregating and displaying information from all other nodes
- **Syncronisers**: Allow of syncronising states accross multiple simulations (examples include the gazebo_syncroniser, allowing for syncronising the pose of rlb agents with a gazebo simulation).

## Key topics
The RLB framework is build around a few key *Topics*, through which all packages access and retreive all necessary information. To ensure compatibility of all RLB packages, it is necessary to go through those topics when passing certain data. 

Those include:
- **/goals_backlog:** Used to pass Goal messages
```
string robot_id
string goal_sequence_id
string meta_action
float64 priority

geometry_msgs/Point[] sequence
```

## RLB message types
RLB uses a few custom messages, defined as follows:

- **Goal** messages: Used to send a goal position (or sequence of positions). These can then be picked up by controlers responsible for performing the goto.
```
string robot_id
string goal_sequence_id
string meta_action
float64 priority

geometry_msgs/Point[] sequence
```
> [!Note]
> In the case of the RLB controller, a larger priority value equals a larger priority. Goals will be undertaken by order of priority and FIFO within each priority level. Goals are not preemptive.


- **TeamComm** messages: Generic message used internally by rlb to pass around relevant information to update various aspect of the framework. 
```
string robot_id
string type
string memo
```

# Installation
As all packages can work independently of each other, it is possible to only clone the desired ones.
The only exception being the *rlb_utils* packages, containing all the relevant message definitions. It is as such necessary to install in addition to any other packages.

All rlb packages can be found on github/gitlab.

# Tutorials
## Getting started
The rlb framework can be used to run various types of experiments. Any combination of packages in possible, and experiments can always be ran with real robots, emulated robots, or a combination of both.

When running an experiement, it is first necessary to determine what test condition and experimental setup will be most optimal, and which packages will be necessary. This will help determining the scheme used and the sources of data.

*A number of example experimental setup are presented further down. For each case, the elements to be substituted is highlighted in green.*

## Real-world vs simulated
rlb supports both real-world and simulated runs. The only difference being in the source of the pose /sensors data retreived and the execution of the *Twist* instructions. 

### Running an RLB simulation
To perform a fully simulated run, simply specify the desired configuration for the robot team, and run the *rlb_turtles_emulator* packages.

*Note: See "Package: RLB Turtle emulator" for more information on the package*

### Running an RLB real-world/hybrid experiement (Voliere specific)
To perform a run using real robots, a number of steps must be undertaken to prepare the room:
1) Power on all the optitrack cameras, and make sure the optitrack computer is booted up with motive running.
2) Connect to the optitrack networking computer through SSH and launch natnet lara (alias: natnet).
3) Insert a charge battery into every Turtlebot to be used, place all turtlebots in the testing area and power them on.
4) Check if all turtlebots are visible in motive and properly tracked (re-callibration of the optitrack cameras must be done on a regular basis, follow this [link](https://www.youtube.com/watch?v=cNZaFEghTBU) for more information of the calibration process).

*Note: The location of the robots should now be streamed to the cooresponding topics*

5) Connect to every robot through SSH and launch a turtlebot natnet client on each of them (tb3_lara)

The room should now be ready to go to launch an experiement.

## Package: RLB viz
RLB viz is the standard visualisation interface provided for the rlb framework. It allows for a very simplistic visualisation of an ongoing experiement, and the state of the robots. It was primarly designed for real-world experiements and for monitoring turtlebots running in ONERA's Voliere.

To run:
```
ros2 run rlb_viz rlb_viz
```

Any modification of the **robot_parameters.py** script requires a re-launch of the controllers and visualiser nodes to be reflected

## Package: RLB Gazebo syncroniser
Used for syncronising a model's pose in the gazebo turtlebot simulation with a pose obtained from real-world robots (or emulated ones).

To run:
```
ros2 run rlb_viz rlb_viz
```

*Note: Gazebo turtlebot3 simulation must be running to retreive data, the syncroniser is only responsible of keeping the position of the turtle model in the simulation up to date.*

## Package: RLB Controller
Basic controller implementation. Provided to enable testing task allocation algorithms. The controller retreives goal messages, and performs a goto on all goals received in order of priority/FIFO (goals are not preemptive). The controller also contains some basic sensor-based collision detection and avoidance logic.

All properties of the controller can be adjusted in the **robot_parameters.py** script (every modification however requires a re-launch of the controllers and visualiser nodes to be reflected)

To run:
```
ros2 launch rlb_controller rlb_<# turtles>_launch.py
```

*Note: One controller must be launched per agent/turtle*

## Package: RLB Turtle emulator
Used ot emulated the phisical behavior of the Turtlebot3 robot. The emulator only simulates movement dynamics, and does not simulate sensors. A single emulator node can emulate multiple turtlebots

To run:
```
ros2 run rlb_viz rlb_viz
```

*Note: While the emulator does not simulate any sensors, it can be made to return an empty scan as placeholder for compatibility's sake if desired*

## RLB experiment example: Custom controller
![[RLB stack - Controller experimental setup with virtual environment and sensors in gazebo.svg]]

1) The controller determines *Twist* instructions based on pose and sensors input. The instruction is then published on the */Turtle_1/cmd_vel* topic.
2) The instruction is retreived by the agent node and executed
3) The resulting pose is retreived by the Syncroniser node and applied to the simulated model in Gazebo.

---
Packages used:
- **Misc**: rlb_utils
- **Controller**: Custom controller (user provided)
- **Visualiser**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
---


## RLB experiment example: Simple Goto task allocation
![[RLB stack - Simple goto experimental setup.svg]]
1) The source sim is responsible for emitting *goal* messages. These will then be picked up by the agents controllers.
2) The agents receive each goal, determine whether it is addressed to the agent they respectively are in charge of, and if so add the goal to their goal backlog. In parallel, each controller will follow its defined goto protocol to start undertaking goals as soon as possible, and will emit*Twist* instructions to the target agent's */cmd_vel* topic. The "state" of each controller is relayed to the visualised through *TeamComm* messages.
3) The agents continously publish their pose as as *PoseStamped* message to their respective pose topic */Turtle_…/pose*. It is then picked up by the source sim, controllers, and visualiser.

---
Packages used:
- **Misc**: rlb_utils
- **Source sim**: Custom task allocation algorithm (user provided)
- **Controller**: rlb_controller
- **Visualiser**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
---

## RLB experiment example: Goto task allocation with virtual environment and sensors
![[RLB stack - Goto experimental setup with virtual environment and sensors in gazebo.svg]]

The key difference between this experimental setup and the one described previously is the introduction of a syncroniser and a gazebo simulation. 
1) The syncroniser is responsible for syncronising the pose of the robots with the ones in the gazebo simulation. 
2) The gazebo simulation then uses the provided virtual environment and sensor suite to provide sensor data, which can be used in the source sim.
>[!Note]
> It is possible to also use the gazebo virtual environment to simulate terrain elevation or roughness. All that is needed is to feed the relevant gazebo simulation data back into the controller to be accounted for. Note that the provided rlb controller does not do that, a custom controller would as such be necessary.

---
Packages used:
- **Misc**: rlb_utils
- **Source sim**: Custom task allocation algorithm (user provided)
- **Controller**: rlb_controller
- **Visualiser**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
- ==**Syncroniser**: rlb_gazebo_bridge==
- ==**Simulation**: Turtlebot3 gazebo simulation==
- ==**Virtual environment/sensor suite**:  (user provided)==
---
