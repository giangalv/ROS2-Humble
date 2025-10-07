# ROS2-Humble

## 1.INSTALL TURTLESIM

Install te turtlesim package for ROS2-Humble:
```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

To check if the package is installed, run the following command, which should return a list of turtlesim’s executables:
```bash
ros2 pkg executables turtlesim
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node

## 2.START TURTLESIM
To start turtlesim, enter the following command in your terminal:
```bash
ros2 run turtlesim turtlesim_node
```
Under the command, you will see messages from the node. There you can see the default turtle’s name and the coordinates where it spawns.

The simulator window should appear, with a random turtle in the center.

## 3.USE TURTLESIM
Open a new terminal and source ROS 2 again.
Now you will run a new node to control the turtle in the first node:
```bash
ros2 run turtlesim turtle_teleop_key
```
At this point you should have three windows open: a terminal running **turtlesim_node**, a terminal running **turtle_teleop_key** and the **turtlesim window**.

Arrange these windows so that you can see the turtlesim window, but also have the terminal running turtle_teleop_key active so that you can control the turtle in turtlesim.

Use the arrow keys on your keyboard to control the turtle. It will move around the screen, using its attached “pen” to draw the path it followed so far.

You can see the nodes, and their associated topics, services, and actions, using the **list** subcommands of the respective commands:
```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
The goal of this tutorial is only to get a general overview of turtlesim.

## 4.INSTALL RQT
Open a new terminal to install rqt and its plugins:
```bash
sudo apt update
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

```

To run rqt:
```bash
rqt
```

rqt is **plugin-based**, so it can do many things. Some common pluggins are:
- **RQT_GRAPH** shows the ROS computation graph — nodes and topics — visually.
- **RQT_CONSOLE** display log messages (ROS_INFO, ROS_WARN, etc.) in real-time
- **RQT_PLOT** plot numerical data from topics in real-time.
- **RQT_TOPIC** monitor topic data, publish test messages, or echo messages.
- etc...

## 5.CLOSE TERMINALS
To stop the simulation you can enter **Ctrl+c** in the terminals.

-----------------------------------------------------------------------------------------------
