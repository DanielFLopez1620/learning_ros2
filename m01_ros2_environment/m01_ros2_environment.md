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





