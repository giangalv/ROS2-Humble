# ROS2-Humble
# USING TURTLESIM, ROS2 and RQT
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
# NODES
## 1. Ros2 run
The command **ros2 run** launches an executable from a package.
```bash
ros2 run <package_name> <executable_name>
```
To run turtlesim, open a new terminal, and enter the following command:

```bash
ros2 run turtlesim turtlesim_node
```
The turtlesim window will open, as we saw.

Here, the package name is **turtlesim** and the executable name is **turtlesim_node**.

We still don’t know the node name, however. You can find node names by using **ros2 node list**.

## 2. Ros2 node list
**ros2 node list** will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command. The terminal will return the node name:

```bash
ros2 node list
```
/turtlesim

Open another new terminal and start the teleop node with the command:

```bash
ros2 run turtlesim turtle_teleop_key
```

Here, we are referring to the **turtlesim** package again, but this time we target the executable named **turtle_teleop_key**.

Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:

```bash
ros2 node list
```
/turtlesim
/teleop_turtle

## 3. Ros2 node info
Now that you know the names of your nodes, you can access more information about them with:

```bash
ros2 node info <node_name>
```
To examine your latest node, **my_turtle**, run the following command:
```bash
ros2 node info /turtlesim
```
ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.

### Node remapping
Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values.
Now, let’s reassign the name of our **/turtlesim** node. In a new terminal, run the following command:
```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```
Since you’re calling **ros2 run** on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran **ros2 node list**, and run it again, you will see three node names.

## 4.CLOSE TERMINALS
To stop the simulation you can enter **Ctrl+c** in the terminals.
-----------------------------------------------------------------------------------------------
# TOPICS

```bash
ros2 node list
```


```bash
ros2 node list
```


```bash
ros2 node list
```
