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

# ACTIONS
Open a new terminal and run:
```bash
ros2 run turtlesim turtlesim_node
```
Open another terminal and run:
```bash
ros2 run turtlesim turtle_teleop_key
```

## 1. USE ACTIONS
When you launch the /teleop_turtle node, you will see the following message in your terminal:
```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```
Let’s focus on the second line, which corresponds to an action.
Notice that the letter keys **G|B|V|C|D|E|R|T** form a “box” around the F key on a US QWERTY keyboard.
Each key’s position around **F** corresponds to that orientation in turtlesim. For example, the **E** will rotate the turtle’s orientation to the upper left corner.

Pay attention to the terminal where the **/turtlesim** node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of the **/turtlesim** node. The goal is to rotate the turtle to face a particular direction. A message relaying the result of the goal should display once the turtle completes its rotation
```
[INFO] [turtlesim]: Rotation goal completed successfully
```

The **F** key will cancel a goal mid-execution.

Try pressing the **C** key, and then pressing the **F** key before the turtle can complete its rotation. In the terminal where the **/turtlesim** node is running, you will see the message:
```
[INFO] [turtlesim]: Rotation goal canceled
```
Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the /turtlesim node) can as well. When the server-side chooses to stop processing a goal, it is said to “abort” the goal.

Try hitting the D key, then the G key before the first rotation can complete. In the terminal where the /turtlesim node is running, you will see the message:
```
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```
This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.

## 2. ROS2 NODE INFO
To see the list of actions a node provides, **/turtlesim** in this case, open a new terminal and run the command:
```bash
ros2 node info /turtlesim
```
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
The command returns a list of /turtlesim’s subscribers, publishers, services, action servers and action clients.

Notice that the **/turtle1/rotate_absolute** action for **/turtlesim** is under **Action Servers**. This means **/turtlesim** responds to and provides feedback for the **/turtle1/rotate_absolute** action.

The **/teleop_turtle** node has the name **/turtle1/rotate_absolute** under **Action Clients** meaning that it sends goals for that action name. To see that, run:
```bash
ros2 node info /teleop_turtle
```
```
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

## 3. ROS2 ACTION LIST
To identify all the actions in the ROS graph, run the command:
```bash
ros2 action list
```
```
/turtle1/rotate_absolute
```
This is the only action in the ROS graph right now. It controls the turtle’s rotation, as you saw earlier. You also already know that there is one action client (part of **/teleop_turtle**) and one action server (part of **/turtlesim**) for this action from using the **ros2 node info <node_name>** command.

Actions have types, similar to topics and services. To find /turtle1/rotate_absolute’s type, run the command:
```bash
ros2 action list -t
```
```
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```
In brackets to the right of each action name (in this case only /turtle1/rotate_absolute) is the action type, turtlesim/action/RotateAbsolute.

## 4. ROS2 ACTION INFO
You can further introspect the /turtle1/rotate_absolute action with the command:
```bash
ros2 action info /turtle1/rotate_absolute
```
```
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
This tells us what we learned earlier from running **ros2 node info** on each node: The **/teleop_turtle** node has an action client and the **/turtlesim** node has an action server for the **/turtle1/rotate_absolute** action.

### 5. ROS2 INTERFACE SHOW
One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.

Recall that you identified **/turtle1/rotate_absolute**’s type when running the command **ros2 action list -t**. Enter the following command with the action type in your terminal:
```bash
ros2 interface show turtlesim/action/RotateAbsolute
```
Which will return:
```
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
The section of this message above the first **---** is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

## 6. ROS2 ACTION SEND_GOAL
Now let’s send an action goal from the command line with the following syntax:
```bash
ros2 action send_goal <action_name> <action_type> <values>
```
**<values>** need to be in YAML format.

Keep an eye on the turtlesim window, and enter the following command into your terminal:
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```
```
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```
You should see the turtle rotating.

All goals have a unique ID, shown in the return message. You can also see the result, a field with the name **delta**, which is the displacement to the starting position.

To see the feedback of this goal, add **--feedback** to the **ros2 action send_goal** command:
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```
```
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```
You will continue to receive feedback, the remaining radians, until the goal is complete.

# LAUNCHING NODES
In most of the introductory tutorials, you have been opening new terminals for every new node you run. As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.

Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Running a single launch file with the ros2 launch command will start up your entire system - all nodes and their configurations - at once.

## 1. RUNNING A LAUNCH FILE
Open a new terminal and run:
```bash
ros2 run turtlesim turtlesim_node
```
This command will run the following launch file:
```
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='turtlesim1', package='turtlesim',
            executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace='turtlesim2', package='turtlesim',
            executable='turtlesim_node', output='screen'),
    ])
```
This will run two turtlesim nodes.
The launch file above is written in Python, but you can also use XML and YAML to create launch files.

## 2. CONTROL THE TURTLESIM NODES
Now that these nodes are running, you can control them like any other ROS 2 nodes.

In a second terminal:
```bash
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

In a third terminal:
```bash
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
After running these commands, you should see the turtles drive in opposite directions.

# USING COLCON TO BUILD PACKAGES
## 1. INSTALL COLCON
```bash
sudo apt install python3-colcon-common-extensions
```

## 2. BASICS
A ROS workspace is a directory with a particular structure. Commonly there is a **src*** subdirectory. Inside that subdirectory is where the source code of ROS packages will be located. Typically the directory starts otherwise empty.

colcon performs out-of-source builds. By default it will create the following directories as peers of the **src** directory:
* The **build** directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.
* The **install** directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.
* The **log** directory contains various logging information about each colcon invocation.

NB: Compared to catkin there is no **devel** directory.

## 3. CREATE A WORKSPACE
First, create a directory (ros2_ws) to contain our workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
At this point the workspace contains a single empty directory **src**.

## 4. ADD SOME SOURCES
Let’s clone the examples repository into the src directory of the workspace:
```bash
git clone https://github.com/ros2/examples src/examples -b humble
```
Now the workspace should have the source code to the ROS 2 examples.

In the root of the workspace, run colcon build.
```bash
colcon build --symlink-install
```
After the build is finished, we should see the build, install, and log directories:
```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

## 5. RUN TESTS
To run tests for the packages we just built, run the following:
```bash
colcon test
```

## 6. SOURCE THE ENVIRONMENT
When colcon has completed building successfully, the output will be in the **install** directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the **install** directory to help set up the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.
```bash
source install/setup.bash
```

## 7. TRY A DEMO
With the environment sourced, we can run executables built by colcon. Let’s run a subscriber node from the examples:
```bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

In another terminal, let’s run a publisher node (don’t forget to source the setup script):
```bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```
You should see messages from the publisher and subscriber with numbers incrementing.

## 8. TIPS
* If you do not want to build a specific package, then place an empty file named **COLCON_IGNORE** in the directory and it will not be indexed.
* If you want to avoid configuring and building tests in CMake packages you can pass: **--cmake-args -DBUILD_TESTING=0**.
* If you want to run a single particular test from a package:
```bash
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```

# CREATING A WORKSPACE
A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

## 1. CREATE A NEW DIRECTORY
 The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name ros2_ws, for “development workspace”:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 2. CLONE A SAMPLE REPO
Ensure you’re still in the ros2_ws/src directory before you clone.

A repo can have multiple branches. You need to check out the one that targets your installed ROS 2 distro. When you clone this repo, add the -b argument followed by that branch.

In the ros2_ws/src directory, run the following command:
```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```
Now ros_tutorials is cloned in your workspace. The ros_tutorials repository contains the turtlesim package, which we’ll use in the rest of this tutorial. The other packages in this repository are not built because they contain a COLCON_IGNORE file.

## 3. RESOLVE DEPENDECIES
Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.

From the root of your workspace (ros2_ws), run the following command:
If you’re still in the src directory with the ros_tutorials clone, make sure to run cd .. to move back up to the workspace (ros2_ws).
```bash
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
#All required rosdeps installed successfully
```
Packages declare their dependencies in the package.xml file (you will learn more about packages in the next tutorial). This command walks through those declarations and installs the ones that are missing. You can learn more about rosdep in another tutorial (coming soon).

## 4. BUILD THE WORKSPACE WITH COLCON
From the root of your workspace (ros2_ws), you can now build your packages using the command:
```bash
colcon build
```
Once the build is finished, enter the command in the workspace root (~/ros2_ws). You will see that colcon has created new directories:
```bash
ls
build  install  log  src
```
The **install** directory is where your workspace’s setup files are, which you can use to source your overlay.

## 5. SOURCE THE OVERLAY
Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace.
In the new terminal, source your main ROS 2 environment.
```bash
source /opt/ros/humble/setup.bash
```
Go into the root of your workspace:

```bash
cd ~/ros2_ws
```

In the root, source your overlay:
```bash
source install/local_setup.bash
```

Now you can run the turtlesim package from the overlay:
```bash
ros2 run turtlesim turtlesim_node
```

# CREATING A PACKAGE
A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

You can create a package using either CMake or Python, which are officially supported, though other build types do exist.

## 1. WHAT MAKES UP ROS 2 PACKAGE?
ROS 2 Python and CMake packages each have their own minimum required contents:

### 1.a CMAKE
* CMakeLists.txt file that describes how to build the code within the package
* include/<package_name> directory containing the public headers for the package
* package.xml file containing meta information about the package
* src directory containing the source code for the package
  
The simplest possible package may have a file structure that look like:
```
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```

### 1.b PYTHON
- package.xml file containing meta information about the package
- resource/<package_name> marker file for the package
- setup.cfg is required when a package has executables, so ros2 run can find them
- setup.py containing instructions for how to install the package
- <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

The simplest possible package may have a file structure that looks like:
```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

## 2. PACKAGES IN A WORKSPACE
A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.). You cannot have nested packages.

Best practice is to have a src folder within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

A trivial workspace might look like:
```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

## 3. CREATE A PACKAGE
Make sure you are in the src folder before running the package creation command.
```bash
cd ~/ros2_ws/src
```

The command syntax for creating a new package in ROS 2 is:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```
For this tutorial, you will use the optional argument --node-name which creates a simple Hello World type executable in the package.

Enter the following command in your terminal:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
```
You will now have a new folder within your workspace’s src directory called my_package.

## 4. BUILD A PACKAGE







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
