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

![turtlesim_node](/m01_ros2_environment/resources/the_turtlesim.png)

Say hello to your turtle friend, but if you think he is not a good friend (because it does nothing), you can run another executable:

    ros2 run turtlesim turtle_teleop_key # Run it on a new terminal

![turtlesim_teleop](/m01_ros2_environment/resources/turtle_teleop.png)

With this executable you can play with your friend, by using the key arrows. This is one example of communication, and we can make more with this little turtle, just wait until we get to know more about *nodes*, *topics*, *services* and *actions*.

# Visualization with RQT:

There is a tool that let you see how the communication between nodes works, it is called **rqt**. As you may recall, first, we need to install it:

    sudo apt-get install ~nros-humble-rqt*

The '*' at the end of the command means that we want to install all the rqt plugins or related packages. To test it (make sure you have some nodes running, for example, the turtlesim_node and the turtlesim_teleop), you can run:

    rqt

The display will look like this: 

![rqt](/m01_ros2_environment/resources/plain_rqt.png)

To watch something, you need to go to the plugins and select the one of your interest, for example, topics which are being used by the turtlesim nodes, here you can explore the publisher, check message types or monitore the topics.

![rqt_graph](/m01_ros2_environment/resources/rqt_graph.png)

Another interesting feature, is that this allows you to publish/subscribe  (*topics*) and request/call (*services*) to nodes, for instance, with turtlesim we can spawn more turtles, in the service plugin we select the **/spawn** service, we add the info we want and then call the service. Now we have two turtles.

If you feel lost, do not worry, we will cover more on this later in this module and its implementation in the next modules.

Before we move on, make sure you close (Ctrl+C) all the nodes.

# A node, a world:

![spoiler-communication](/m01_ros2_environment/resources/node-topic-service-action.png)

Nodes are the basic unit and they have the ability to communicate with other nodes, for executing a node, remember that you will need:

    ros2 run <package_name> <executable_name>

But here we do not know the node of the name, just the executable that invokes it, for that you can run:

    ros2 node list

If you run this, maybe there won't be output. So try to run it after having nodes like the executables in *turtlesim* and check the results.

![ros2_node_list](/m01_ros2_environment/resources/ros2_node_list.png)

There would be cases where you want to change the name of the node, for that we use **remapping**:

    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=new_turtle

This will generate another turtlesim window, independent from the preovios one. Here you can check the info of the node, for that you can use:

    ros2 node info /new_turtle

After the command you should be able to wath the subscribers, publishers, clients and servers of the custom name *turtlesim* you invoke.

# Exchange messages with topics:

The main basic communication for ROS2, it is based in four parts:

![Topics](/m01_ros2_environment/resources/topics.png)

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

# Services, another perspective:

It is a different type of communication based on the roles of the client and the server. Think of it like a shop, someone comes to buy something (he is the **client**) to your shop (so you are the **server**), they are *requesting* you a product and your *response* is the product they are asking for or (in case you don't have it) an indication that you cannot give it to him.

Let's play with our turtle friends to get to know more about services, Make sure you run the *turtlesim* node and the *turtle_teleop_key* node. In this section, we will use the commands of **ros2 service**, first let's list all services available.

    ros2 service list

![Services](/m01_ros2_environment/resources/service.png)

Now, you need to understand them, so you can take advantage of them, the services have two parts the **request** (the info you provide to the server) and the **response** (the result of processing the info provided). Also, they have different types, and as you can guess, you can use:

    ros2 service type <service_name>
    ros2 service list -t # Is another option to get the type

Another option is to search a service by its type, for that the command is:

    ros2 service find <type_service_name>

And if you need more details on the type, you can run the command:

    ros2 interface show <type_service_name>

Here it should display a text divided by "---" which is the indicator/separator between request and response. Finally, you can also play with services in the terminal by calling the corresponding server with the appropiate information, for example, if you want to create another turtle you can use:

    ros2 service call /spawn  turtlesim/srv/Spawn "{x:2, y:5, theta:0.5, name:'Dan'}"

![spawn_turtle](/m01_ros2_environment/resources/new_turtle.png)

# Parameters for node configurations:

Not everything needs to be configuration, but you will need a way to define some behaviors with the nodes, for that you have the parameters. Once again, let's review this with our turtle friends.

After you have run the turtle nodes, let's check the list of params of this nodes:

    ros2 param list

![ros2_param_list](/m01_ros2_environment/resources/ros2_param_list.png)

Know, as in programming a certain languages, you can **get** or **set** this parameters, but you need to know before hand how they work and which values do they use.

    ros2 param get <node_name> <param_name>
    ros2 param set <node_name> <param_name> <new_value>

The more visible params with *turtlesim* are the colors of the background, as you may know, colors can work with different formats, in case of this background we use RGB (Red, Green, Blue) with a range of 0-255 (integer) for each one. So, try to play with the colors of the turtlesim to get one of your favorite colors, for example:

    ros2 param set /turtlesim background_g 255

But, this is not all, you can save the params and use them later, this process is called **dump** and **load**, the format supported for storing and loading parameters are the **.yaml** files. After you modify some parameters (and make sure everything is still working), you an try:

    ros2 param dump /turtlesim > my_params.yaml
    # Close all and re-run the nodes
    ros2 param load my_params.yaml

Another possibility is to load the params in the moment you run the node, for that, you can use:

    ros2 run turtlesim turtlesim_node --ros-args --params-file my_params.yaml

# When something goes wrong...

There would be case when you will need some help, you can find resources online searching on Google (or the search engine of your preference), in the forum of [Robotics Exchange]() and in the documentation. Also, pay attetion to the errors and warning the console is displaying to you when you compile and test.

However, you should try **ros2doctor** at some point, for those cases when the ROS2 setup is not working as expected, as it check the platform, version, network, env and system, then it informs you about warnings and reason for the issues. For running you just use:

    ros2 doctor

If everthing seems to be right, you should see a:

    All <n> checks passed

If you received a warn, it could have the form of

    <path>: <line> UserWaring: <message>

For getting a report, you can use:

    ros2 doctor --report

# Using data with ROS2

There would be times when you need to make tests, to make sure everything works as expected, you can try these with the topics of your nodes... you may need to test the same data to make sure it works, in those cases, using the node directly multiple times can be problematic. 

That is why we have **ros2 bag**, which is a command line tool, it is used to record, save and reproduce data. As always, let's use our favorite turtle companion:

    ros2 run turtlesim turtlesim_node
    ros2 run turtlesim turtle_teleop_key

Then, create a directory for your **bag files**:

    mkdir <your_bag_dirs>
    cd <your_bag_dirs>

Select a topic to save, and start recording, note that you can stop recording by using Ctrl + C.

    ros2 bag record /turtle1/cmd_vel

In the case you want to record every topic, you just need to pass the **-a** flag. Now, to make sure you recorded the data, assign a file, you can use

    ros2 bag record -o turtle_info /turtle1/cmd_vel /turtle1/pose # Ctrl+ C when you are done.

If you want to check the information of a bag file, you can use:

    ros2 bag info turtle_info

Now, if you need to make some test with the same info, you just use the recorded data, you can play it with:

    ros2 bag play turtle_info


# Resources 

- ROS2 Cli Tutorials: [Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
    
