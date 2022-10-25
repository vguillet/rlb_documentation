> **Warning** 
> This document presents an incomplete overview of the RLB experimental framework. The documentation was written quickly to provide transparency on the functioning of the framework, and might contain errors


The **R**obot **L**ab **B**ridge (**RLB**) framework is a generic robotics muti-agent interface based on ROS2 Foxy. 

It aims at providing a framework and tools for running and testing single or multi-agent control and/or task allocation algorithms, using a combination of real and simulated data sources. The toolkit provides all necessary packages for fulfiling most simple experimental cases, from pre-implemented goto control laws for higher level coordination experiments to lower-level control. It also provides visualization tools, a set of emulator for simulating robots/communications/etc.

Each package in the RLB toolkit was build to be independent of each other, with the exception of the RLB_utils (*more in the installation section*).

The documentation contains an **Overview** of the framework architecture, an **Installation** guide, and a number of **Tutorials** covering various aspects of the framework.

# Overview

## General RLB structure 
Each RLB run is functionally split into a few key structural elements, each responsible for different aspect of the run. Not all elements are necessary depending on the experiment type, and any element can be swapped for a custom one if necessary as long as the **key topics** are maintained. 

Notable elements include:
- **Agent instances**: Responsible of emitting agent's pose to */Agent/pose*
- A **source simulation**: Responsible of emitting goals to */goals_backlog*. Used with the rlb_controller package for experimenting with task allocation systems.
- **Controllers**: Responsible of controlling agents, such as performing gotos etc… Controllers emit *Twist* messages to the agent's */cmd_vel* 
- **Visualisers**: Responsible of agregating and displaying information from all other nodes
- **Synchronizers**: Allow of synchronizing states across multiple simulations (examples include the gazebo_syncroniser, allowing for synchronizing the pose of rlb agents with a gazebo simulation).

## Key topics
The RLB framework is build around a few key *Topics*, through which all packages access and retreive all necessary information. To ensure compatibility of all RLB packages, it is necessary to go through those topics when passing certain data. 

Those include:
- **/goals_backlog:** Used to pass Goal messages

- **/Turtle_#/pose:** Used to pass PoseStamped messages (standard ROS geometry msg), containing information about the position of a robot at a given time.

- **/Turtle_#/interrupt:** Used to pass an RLBInterrupt message. More details about the interrupt functionality is provided further down in the *rlb_controller* package description.

- **/team_comms:** Main topic used for intra-rlb communication. Contains information about the collision detection, goal selection etc…


### Recording topics
RLB topics' QOS differs from ROS's bag default one. As such a QOS override is necessary, and provided if necessary. It can be found in the `rlb_misc` folder.

To bag the RLB key topics:
```
ros2 bag record -o <new directory name> --qos-profile-overrides-path qos_override.yaml /Turtle_1/pose /Turtle_1/interrupt /goals_backlog
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

To publish a goal manually:
```
ros2 topic pub --once  /goals_backlog rlb_utils/msg/Goal "{robot_id: "Turtle_1", goal_sequence_id: "Test_goal", meta_action: "Set", priority: 1, sequence: [{x: 1, y: 1, z: 1}]}"ros2 topic pub --once  /goals_backlog rlb_utils/msg/Goal "{robot_id: "Turtle_1", goal_sequence_id: "Test_goal", meta_action: "Set", priority: 1, sequence: [{x: 1, y: 1, z: 1}]}"
```

- **TeamComm** messages: Generic message used internally by rlb to pass around relevant information to update various aspect of the framework. 
```
string robot_id
string type
string memo
```

- **RLBInterrupt** messages: Message used to control the behavior of an RLB controller. Acceptable values for `type` include *KILL*, *CLEAR_GOAL*,  *CLEAR_BACKLOG*, *STOP_GOAL_AND_CLEAR_BACKLOG*, *RESET*. 
```
builtin_interfaces/Time stamp
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

When running an experiment, it is first necessary to determine what test condition and experimental setup will be most optimal, and which packages will be necessary. This will help determining the scheme used and the sources of data.

*A number of example experimental setup are presented further down. For each case, the elements to be substituted is highlighted in green.*

## Real-world vs simulated
rlb supports both real-world and simulated runs. The only difference being in the source of the pose /sensors data retrieved and the execution of the *Twist* instructions. 

### Running an RLB simulation
To perform a fully simulated run, simply specify the desired configuration for the robot team, and run the *rlb_turtles_emulator* packages.

*Note: See "Package: RLB Turtle emulator" for more information on the package*

### Running an RLB real-world/hybrid experiment (Voliere specific)
To perform a run using real robots, a number of steps must be undertaken to prepare the room:
1) Power on all the optitrack cameras, and make sure the optitrack computer is booted up with motive running.
2) Connect to the optitrack networking computer through SSH and launch natnet lara (alias: natnet).
3) Insert a charge battery into every Turtlebot to be used, place all turtlebots in the testing area and power them on.
4) Check if all turtlebots are visible in motive and properly tracked (re-calibration of the optitrack cameras must be done on a regular basis, follow this [link](https://www.youtube.com/watch?v=cNZaFEghTBU) for more information of the calibration process).

*Note: The location of the robots should now be streamed to the corresponding topics*

5) Connect to every robot through SSH and launch a turtlebot natnet client on each of them (tb3_lara)

The room should now be ready to go to launch an experiment.

## Package: RLB viz
RLB viz is the standard visualization interface provided for the rlb framework. It allows for a very simplistic visualization of an ongoing experiment, and the state of the robots. It was primarily designed for real-world experiments and for monitoring turtlebots running in ONERA's Voliere.

To run:
```
ros2 run rlb_viz rlb_viz
```

Any modification of the **robot_parameters.py** script requires a re-launch of the controllers and visualizer nodes to be reflected

## Package: RLB Gazebo synchronizer
Used for synchronizing a model's pose in the gazebo turtlebot simulation with a pose obtained from real-world robots (or emulated ones).

To run:
```
ros2 run rlb_viz rlb_viz
```

*Note: Gazebo turtlebot3 simulation must be running to retrieve data, the synchronizer is only responsible of keeping the position of the turtle model in the simulation up to date.*

## Package: RLB Controller
Basic controller implementation. Provided to enable testing task allocation algorithms. The controller retreives goal messages, and performs a goto on all goals received in order of priority/FIFO (goals are not preemptive). The controller also contains some basic sensor-based collision detection and avoidance logic.

All properties of the controller can be adjusted in the **robot_parameters.py** script (every modification however requires a re-launch of the controllers and visualizer nodes to be reflected)

To run:
```
ros2 launch rlb_controller rlb_<# turtles>_launch.py
```

*Note: One controller must be launched per agent/turtle*

### Interrupt
Each controller posses an `/Interrupt` topic, which can be used to control its behavior mid-run. An `RLBInterrupt` message must be used, with the following structure:

```
builtin_interfaces/Time stamp
string type
string memo
```
Acceptable values for `type` are:
- **KILL**: Locks the controller until a **RESET** interrupt is received
- **CLEAR_GOAL**: Cancel the current goal
- **CLEAR_BACKLOG**: Clear the goal backlog
- **STOP_GOAL_AND_CLEAR_BACKLOG**: Cancel the current goal and clear the goal backlog
- **RESET**: Reset kill flag, resuming controller process

To manualy send an interrupt:
```
ros2 topic pub --once  /Turtle_1/interrupt rlb_utils/msg/RLBInterrupt "{stamp: {sec: 1.0, nanosec: 1.0}, type: "KILL", memo: "Batterie_vide"}"
```

## Package: RLB Turtle emulator
Used to emulate the physical behavior of the Turtlebot3 robot. The emulator only simulates movement dynamics, and does not simulate sensors. A single emulator node can emulate multiple turtlebots

To run:
```
ros2 run rlb_viz rlb_viz
```

*Note: While the emulator does not simulate any sensors, it can be made to return an empty scan as placeholder for compatibility's sake if desired*

## RLB experiment example: Custom controller

![cover](https://github.com/vguillet/rlb_documentation/blob/main/RLB%20stack%20-%20Controller%20experimental%20setup%20with%20virtual%20environment%20and%20sensors%20in%20gazebo.svg?raw=true)


1) The controller determines *Twist* instructions based on pose and sensors input. The instruction is then published on the */Turtle_1/cmd_vel* topic.
2) The instruction is retrieved by the agent node and executed
3) The resulting pose is retrieved by the Synchronizer node and applied to the simulated model in Gazebo.

---
Packages used:
- **Misc**: rlb_utils
- **Controller**: Custom controller (user provided)
- **Visualizer**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
---


## RLB experiment example: Simple Goto task allocation

![cover](https://github.com/vguillet/rlb_documentation/blob/main/RLB%20stack%20-%20Simple%20goto%20experimental%20setup.svg?raw=true)

1) The source sim is responsible for emitting *goal* messages. These will then be picked up by the agents controllers.
2) The agents receive each goal, determine whether it is addressed to the agent they respectively are in charge of, and if so add the goal to their goal backlog. In parallel, each controller will follow its defined goto protocol to start undertaking goals as soon as possible, and will emit*Twist* instructions to the target agent's */cmd_vel* topic. The "state" of each controller is relayed to the visualized through *TeamComm* messages.
3) The agents continuously publish their pose as as *PoseStamped* message to their respective pose topic */Turtle_…/pose*. It is then picked up by the source sim, controllers, and visualizer.

---
Packages used:
- **Misc**: rlb_utils
- **Source sim**: Custom task allocation algorithm (user provided)
- **Controller**: rlb_controller
- **Visualiser**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
---

## RLB experiment example: Goto task allocation with virtual environment and sensors

![cover](https://github.com/vguillet/rlb_documentation/blob/main/RLB%20stack%20-%20Goto%20experimental%20setup%20with%20virtual%20environment%20and%20sensors%20in%20gazebo.svg?raw=true)

The key difference between this experimental setup and the one described previously is the introduction of a synchronizer and a gazebo simulation. 
1) The synchronizer is responsible for synchronizing the pose of the robots with the ones in the gazebo simulation. 
2) The gazebo simulation then uses the provided virtual environment and sensor suite to provide sensor data, which can be used in the source sim.
>[!Note]
> It is possible to also use the gazebo virtual environment to simulate terrain elevation or roughness. All that is needed is to feed the relevant gazebo simulation data back into the controller to be accounted for. Note that the provided rlb controller does not account for that; a custom controller would as such be necessary.

---
Packages used:
- **Misc**: rlb_utils
- **Source sim**: Custom task allocation algorithm (user provided)
- **Controller**: rlb_controller
- **Visualiser**: rlb_viz
- **Agents**: Real robot obtained pose (optitrack) or rlb_turtles_emulator
- **Syncroniser**: rlb_gazebo_bridge
- **Simulation**: Turtlebot3 gazebo simulation
- **Virtual environment/sensor suite**:  (user provided)
---
