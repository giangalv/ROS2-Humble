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
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

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

* ros2 node list
* ros2 topic list
* ros2 service list
* ros2 action list

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
## 1. ROS2 RUN
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

## 2. ROS2 NODE LIST
**ros2 node list** will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command. The terminal will return the node name:

```bash
ros2 node list
```
```
/turtlesim
```
Open another new terminal and start the teleop node with the command:

```bash
ros2 run turtlesim turtle_teleop_key
```

Here, we are referring to the **turtlesim** package again, but this time we target the executable named **turtle_teleop_key**.

Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:

```bash
ros2 node list
```
```
/turtlesim
/teleop_turtle
```

## 3. ROS2 NODE INFO
Now that you know the names of your nodes, you can access more information about them with:

```bash
ros2 node info <node_name>
```
To examine your latest node, **my_turtle**, run the following command:
```bash
ros2 node info /turtlesim
```
ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.

## 4. Node Remapping
Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values.
Now, let’s reassign the name of our **/turtlesim** node. In a new terminal, run the following command:
```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```
Since you’re calling **ros2 run** on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran **ros2 node list**, and run it again, you will see three node names.

## 5.CLOSE TERMINALS
To stop the simulation you can enter **Ctrl+c** in the terminals.

-----------------------------------------------------------------------------------------------
# TOPICS
Open a new terminal and run:
```bash
ros2 run turtlesim turtlesim_node
```
Open another terminal and run:
```bash
ros2 run turtlesim turtle_teleop_key
```
## 1. RQT_GRAPH
We will use **rqt_graph** to visualize the changing nodes and topics, as well as the connections between them.
To run rqt_graph, open a new terminal and enter the command:
```bash
ros2 run rqt_graph rqt_graph
```
You can also open rqt_graph by opening rqt and selecting **Plugins > Introspection > Node Graph**.

The graph is depicting how the **/turtlesim** node and the **/teleop_turtle** node are communicating with each other over a topic. The **/teleop_turtle** node is publishing data (the keystrokes you enter to move the turtle around) to the **/turtle1/cmd_vel** topic, and the **/turtlesim** node is subscribed to that topic to receive the data.

rqt_graph is a graphical introspection tool. Now we’ll look at some command line tools for introspecting topics.

## 2. ROS2 TOPIC LIST
Running the **ros2 topic list** command in a new terminal will return a list of all the topics currently active in the system:
```bash
ros2 topic list
```
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**ros2 topic list -t** will return the same list of topics, this time with the topic type appended in brackets:
```bash
ros2 topic list -t
```
```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

These attributes, particularly the type, are how nodes know they’re talking about the same information as it moves over topics.

## 3. ROS2 TOPIC ECHO
To see the data being published on a topic, use:
```bash
ros2 topic echo <topic_name>
```
Since we know that **/teleop_turtle** publishes data to **/turtlesim** over the **/turtle1/cmd_vel** topic, let’s use **echo** to introspect that topic:
```bash
ros2 topic echo /turtle1/cmd_vel
```
At first, this command won’t return any data. That’s because it’s waiting for **/teleop_turtle** to publish something.

Return to the terminal where **turtle_teleop_key** is running and use the arrows to move the turtle around. Watch the terminal where your **echo** is running at the same time, and you’ll see position data being published for every movement you make.

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

## 4. ROS2 TOPIC INFO
Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.

Another way to look at this is running:
```bash
ros2 topic info /turtle1/cmd_vel
```
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

## 5. ROS2 INTERFACE SHOW
Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.

The topic types we saw earlier after running **ros2 topic list -t** let us know what message type is used on each topic. Recall that the **cmd_vel** topic has the type:
```
geometry_msgs/msg/Twist
```
This means that in the package **geometry_msgs** there is a **msg** called **Twist**.
Running:
```bash
ros2 interface show <msg_type>
```
Specifically, what structure of data the message expects:
```bash
ros2 interface show geometry_msgs/msg/Twist
```
Which will return:
This expresses velocity in free space broken into its linear and angular parts.
```
    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```        
This tells you that the **/turtlesim** node is expecting a message with two vectors, **linear** and **angular**, of three elements each.

## 6. ROS2 TOPIC PUB
Now that you have the message structure, you can publish data to a topic directly from the command line using:
```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```
The **'<args>'** argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.

To get the turtle moving, and keep it moving, you can use the following command. It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
With no command-line options, **ros2 topic pub** publishes the command in a steady stream at 1 Hz.

Optional arguments:
* To publish your command just once add the **--once** option.
* To wait for two matching subscriptions -w 2.

## 7. ROS2 TOPIC HZ
You can also view the rate at which data is published using:
```bash
ros2 topic hz /turtle1/pose
```
```
average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
```

It will return data on the rate at which the **/turtlesim** node is publishing data to the **pose** topic.

## 8. ROS2 TOPIC BW
The bandwidth used by a topic can be viewed using:
```bash
ros2 topic bw /turtle1/pose
```
```
Subscribed to [/turtle1/pose]
1.51 KB/s from 62 messages
    Message size mean: 0.02 KB min: 0.02 KB max: 0.02 KB
```
It returns the bandwidth utilization and number of messages being published to the /turtle1/pose topic.

## 9. ROS2 TOPIC FIND
To list a list of available topics of a given type use:
```bash
ros2 topic find <topic_type>
```
Using the **find** command outputs topics available when given the message type:
```bash
ros2 topic find geometry_msgs/msg/Twist
```
```
/turtle1/cmd_vel
```
## 10.CLOSE TERMINALS
To stop the simulation you can enter **Ctrl+c** in the terminals.

-----------------------------------------------------------------------------------------------
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```

```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
