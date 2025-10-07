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

# SERVICES
Open a new terminal and run:
```bash
ros2 run turtlesim turtlesim_node
```
Open another terminal and run:
```bash
ros2 run turtlesim turtle_teleop_key
```

## 1. ROS2 SERVICE LIST
Running the **ros2 service list** command in a new terminal will return a list of all the services currently active in the system:
```bash
ros2 service list
```
```
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
You will see that both nodes have the same six services with parameters in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of.

## 2. ROS2 SERVICE TYPE
Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.

To find out the type of a service, use the command:
```bash
ros2 service type <service_name>
```

Let’s take a look at turtlesim’s **/clear** service. In a new terminal, enter the command:
```bash
ros2 service type /clear
```
```
std_srvs/srv/Empty
```
The **Empty** type means the service call sends no data when making a request and receives no data when receiving a response.

To see the types of all the active services at the same time, you can append the --show-types option, abbreviated as -t, to the list command:
```bash
ros2 service list -t
```

## 3. ROS2 SERVICE FIND
If you want to find all the services of a specific type, you can use the command:
```bash
ros2 service find <type_name>
```
For example, you can find all the **Empty** typed services like this:
```bash
ros2 service find std_srvs/srv/Empty
```
```
/clear
/reset
```

## 4. ROS2 INTERFACE SHOW
You can call services from the command line, but first you need to know the structure of the input arguments.
```bash
ros2 interface show <type_name>
```

To see the request and response arguments of the **/spawn** service, run the command:
```bash
ros2 interface show turtlesim/srv/Spawn
```
```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```
x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional.

## 5. ROS2 SERVICE CALL
Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:
```bash
ros2 service call <service_name> <service_type> <arguments>
```
The **<arguments>** part is optional. For example, you know that **Empty** typed services don’t have any arguments:
```bash
ros2 service call /clear std_srvs/srv/Empty
```
This command will clear the turtlesim window of any lines your turtle has drawn.

Now let’s spawn a new turtle by calling /spawn and setting arguments. Input <arguments> in a service call from the command-line need to be in YAML syntax.
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
You will get this method-style view of what’s happening, and then the service response. Your turtlesim window will update with the newly spawned turtle.


# PARAMETERS
A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.
Open a new terminal and run:
```bash
ros2 run turtlesim turtlesim_node
```
Open another terminal and run:
```bash
ros2 run turtlesim turtle_teleop_key
```

## 1. ROS2 PARAM LIST
To see the parameters belonging to your nodes, open a new terminal and enter the command:
```bash
ros2 param list
```
Based on their names, it looks like **/turtlesim**’s parameters determine the background color of the turtlesim window using RGB color values.

## 2. ROS2 PARAM GET
To display the type and current value of a parameter, use the command:
```bash
ros2 param get <node_name> <parameter_name>
```

Let’s find out the current value of **/turtlesim**’s parameter **background_g**:
```bash
ros2 param get /turtlesim background_g
```
```
Integer value is: 86
```
Now you know **background_g** holds an integer value.
If you run the same command on background_r and background_b, you will get the values 69 and 255, respectively

## 3. ROS2 PARAM SET
To change a parameter’s value at runtime, use the command:
```bash
ros2 param set <node_name> <parameter_name> <value>
```
Let’s change /turtlesim’s background color:
```bash
ros2 param set /turtlesim background_r 150
```
```
Set parameter successful
```
The background of your turtlesim window should change colors

Setting parameters with the set command will only change them in your current session, not permanently.

## 4. ROS2 PARAM DUMP
You can view all of a node’s current parameter values by using the command:
```bash
ros2 param dump <node_name>
```

To save your current configuration of /turtlesim’s parameters into the file turtlesim.yaml, enter the command:
```bash
ros2 param dump /turtlesim > turtlesim.yaml
```
You will find a new file in the current working directory your shell is running in.

## 5. ROS2 PARAM LOAD
You can load parameters from a file to a currently running node using the command:
```bash
ros2 param load <node_name> <parameter_file>
```
To load the turtlesim.yaml file generated with ros2 param dump into /turtlesim node’s parameters, enter the command:
```bash
ros2 param load /turtlesim turtlesim.yaml
```

## 6. LOAD PARAMETERS FILE ON NODE STARTUP
To start the same node using your saved parameter values, use:
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```
This is the same command you always use to start turtlesim, with the added flags --ros-args and --params-file, followed by the file you want to load.

Stop your running turtlesim node, and try reloading it with your saved parameters, using:
```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```
The turtlesim window should appear as usual, but with the purple background you set earlier.

-----------------------------------------------------------------------------------------------



```bash
ros2 run rqt_graph rqt_graph
```
```bash
ros2 run rqt_graph rqt_graph
```
