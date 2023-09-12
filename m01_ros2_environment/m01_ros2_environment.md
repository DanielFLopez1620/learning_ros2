# ROS2 Environment

After you have made the installation of ROS2, you will need to know how ROS2 works and is organized. In this module, we will cover the basics to get started with ROS2 on your PC.

**Note:** Take into account that this repository is designed over Humble distro. Remember, a distro is a ROS "version" linked to a Ubuntu release.

# Environment and workspaces

A workspace is where you develop using ROS2, it can be the underlay (the core ROS2 workspace) or overlay (Local workspaces for development). The benefit of having different workspaces, is that you can have a unique combination of *packages* (think of them as a bunch of executables, libraries, configurations... to work with robots/simulations/environments) that can be independent or even work allowing multiple distros.

## Set up of the environment:

When you want to install packages, you can run:

    sudo apt install ros-<distro>-<package> # In our case, the distro is humble

To access to the ROS2 commands and to get access to run packages installed on the machine, you will need to run on a new terminal:

    source /opt/ros/<distro></distro>/setup.bash

If you do not like to run commands periodically, you can add it to your ./bashrc:

    echo "source /opt/ros/<distro>/setup.bash" >> ./bashrc

## Environment variables:

ROS2 have several variables configurated that are need for running its process, so they must be set up properly. To check them you can run:

    printenv | grep -i ROS

Some of this variable are the **ROS_VERSION** (which indicates that you are using ROS2), **ROS_PYTHON_VERSION** (related with the usage of python (3)) and **ROS_DISTRO** (if 'humble' is not here, you should have to change the distro to follow the following modules).

Another important variable is the **ROS_LOCALHOST_ONLY** which is useful when you want to limit the communications only for you, so they aren't visible to other computers or people. It is boolean, and you can also configure it in the ./bashrc.

# The turtle and the way:

One of the simplest implementations, and you may ask... Why a turtle? The answer is also simple, it is because **Turtlesim** is a great tool to learn the basics of ROS.

Before we start, make sure the turtle is with you and is installed, you use:

    sudo apt-get install ros-humble-turtlesim

## Packages and nodes:

A **node** is the basic unit of ROS2, it aims to accomplish a task that can be related with communitacation mainly, multiple nodes can communicate by using **topics**, **services** or **actions**, we will deepen more on them soon. 

A **packages** is a collection of *nodes* and the content related to them like libraries, parameters, configurations, among others. With the last command we have installed the package of the turtlesim (a 2D turtle that will have a random look based on the logos of all ROS distros). 

To check the information of a package or packages, the command is **ros2 pkg**, for example, if you want to check the executables of a packages you can use:

    ros2 pkg executables turtlesim

The output in terminal, should mention some options like *draw_square*, *mimic*, *turtle_teleop_key* and *turtlesim_node*... Yeah, they are executables, but how do I run them? For that you will need another command, in this case **ros2 run**:

    ros2 run turtlesim turtlesim_node

The previous command, should have opened a new window, like this one:

TODO: Add image of turtlesim

Say hello to your turtle friend, but if you think he is not a good friend (because it does nothing), you can run another executable:

    ros2 run turtlesim turtle_teleop_key # Run it on a new terminal

With this executable you can play with your friend, by using the key arrows. This is one example of communication, and we can make more with this little turtle, just wait until we get to know more about *nodes*, *topics*, *services* and *actions*.

# Visualization with RQT:

There is a tool that let you see how the communication between nodes works, it is called **rqt**. As you may recall, first, we need to install it:

    sudo apt-get install ~nros-humble-rqt*

The '*' at the end of the command means that we want to install all the rqt plugins or related packages. To test it (make sure you have some nodes running, for example, the turtlesim), you can run:

    rqt

The display will look like this: 

TODO: Add image of rqt

To watch something, you need to go to....

Another interesting feature, is that this allows you to publish/subscribe  (*topics*) and request/call (*services*) to nodes, for instance, with turtlesim we can spawn more turtles, in the service plugin we select the **/spawn** service, we add the info we want and then call the service. Now we have two turtles.

Before we move on, make sure you close (Ctrl+C) all the nodes.

# A node, a world:

TODO: Add image of nodes, topics, services and actions

Nodes are the basic unit and they have the ability to communicate with other nodes, for executing a node, remember that you will need:

    ros2 run <package_name> <executable_name>

But here we do not know the node of the name, just the executable that invokes it, for that you can run:

    ros2 node list

If you run this, maybe there won't be output. So try to run it after having nodes like the executables in *turtlesim* and check the results.

TODO: Add image of ros2 node list

There would be cases where you want to change the name of the node, for that we use **remapping**:

    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=new_turtle

This will generate another turtlesim window, independent from the preovios one. Here you can check the info of the node, for that you can use:

    ros2 node info /new_turtle

After the command you should be able to wath the subscribers, publishers, clients and servers of the custom name *turtlesim* you invoke.

# Exchange messages with topics:

The main basic communication for ROS2, it is based in four parts:

TODO: Add image of topics.

- **Message:** The information you want to share between nodes. It can be a data type or a custom type.
- **Topic:** The way messages is moved between nodes, it allows the communication of a message like a channel, so it allows one-to-one, one-to-many or many-to-many communication.
- **Publisher:** The one submitting the information, it can be a sensor, an indicatation...
- **Subscriber:** The one recieving information, so it can process it and determinate what to do.

For the next, make sure you are running the *turtlesim_node* and the *turtle_teleop_key*, also you will need *rqt*:

    ros2 run turtlesim turtlesim_node # Terminal 1
    ros2 run turtlesim turtle_teleop_key # Terminal 2
    rqt_graph # Terminal 3

In the *rqt_graph* window (with the options: Plugin > Introspection > Node graph) you can be able to watch the node communication. And this is the reason you can control the turtle by using the keyboard, because the topic you are using is **/turtle1/cmd_vel**. But this isn't the only topic here, to check more topics just run in a new terminal:

    ros2 topic list

It will enumerate the topics that are currently available, so if you need more details you can type:

    ros2 topic list -t

This flag add the option to get the message type, so you can use it to create a communication. But you can also, check them with *rqt* by unchecking the box *Hide:*.

Now, let's take a look to other important commands of *ros2 topic*:

- **bw**: It allows to see the bandwidth used
- **delay**: Adds timestamp and display the delay
- **echo**: Like the one in shell, it displays the ouput of the topic
- **find**:  List the available topic refered to the type specified.
- **hz**: Get the publish rate frequency.
- **info**: Need help to understand a topic? This is the key.
- **pub**: Publish a single message from terminal.
- **type**: A good complement of info, to get the topic type.

In some cases, you will need extra help for understanding messages and topics, for example, do you know what is cmd_vel? Well, you can check that answer by knowing the type and then search the definition:

    ros2 interface show geometry_msgs/msg/Twist

To close, this subsection, let's publish a velocity command to the turtle, for this you can use:

    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"