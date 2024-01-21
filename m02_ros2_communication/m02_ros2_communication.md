# ROS2 Communications

In this module, you will learn more about packages and its usage for communication with the tools you have already watched in the previous module (**topics**, **services** and **actions**). This implementations can be made with the ROS Client Libraries for Python (**rclpy**) and for C++ (**rclcpp**). You will also learn about compilation of the packages, the usage of **colcon** and the basics of package configuration with **Python** and **CMake**.

## Colcon for your workspace:

Colcon is the tool oriented for compilation of your packages, which is an iteration of the catkin tools from ROS (1). If you made the complete installation, you must have access to *colcon*, but you can check it or install it (if missing) with:

    sudo apt install python3-colcon-common-extensions

The key with the usage of colcon is that you need to follow the conventions for the directories:

- **build:** Where temporary files are stored, the packages are represented with subdirectories. This folder appears after the first run of the colcon configuration.

- **install:** The directory where the packages are installed to, by default in seperate subdirectories.

- **log:** A directory you will need to see when sonmething goes wrong, it contains the logging information about colcon calls.

- **src:** The space of the package, where you develop packages for future executables, configurations...

If this is your first time, you will need to build everything in your ROS2 space, so you need to use the option *colcon build --symlink-install* which allows all the installed files to be changed. If you want to build a single package or a group of them, you may prefer the option *colcon build*.

As always, do not forget to source the **setup.bash** file, otherwise you won't be able to run the executables of any packages.

    source /opt/ros/humble/setup.bash

After this introduction to colcon, we need a *workspace*, the place where your robotics development and magic will come to life. So, the basic for creating a package is that you need a folder (with the name of your preference), and a nested *src* directory.

    mkdir -p ~/<your_workspace>/src

In the *src* directory you can clone packages from Github, Gitlab or you can aslo create your own packages. This packages would need some **dependencies** to be able to run/execute something, in these cases, you can run:

    cd ~/<your_workspace>
    rosdep install -i --from-path src --rosdistro humble -y

Each time you add/create a package you will need to build them and adds the executables to the list, for this you run:

    cd ~/<your_workspace>
    colcon build
    source install/local_setup.bash

## Packages are the key:

Once you have set up your workspace, you could have asked... it seems boring... and yes... you do not have something to execute there, or even something saved there. But what should be in the workspace? Well, to be more precise, what should be in the *src* directory of our workspace, the answer here is: **Packages**.

A package is a collection of directories and files that defines an organizatio unit for your ROS2 code. It also allows you to share it with others. The packages can work with two different aments, which are *CMake* or *Python*, and this implies different package structures.

On one hand, the case of **CMake**, the minimum required content refers to:

- **CMakeLists.txt**: Which describes how the code will be build (mostly related with C/C++).
- **include/<package_name>**: Is the directory that contains the headers of the package.
- **package.xml**: Meta description of the package.
- **src/**: Directory for source code.

On the other hand, we have **Python**, and the minimum structure refers to:

- **package.xml**: Meta description of the package.
- **resource/<package_name>**: Marker files for the package
- **setup.cfg**: Required for setting up executables.
- **setup.py**: File with instructions to install the package.
- **<package_name>/**: Containts the __init__.py for finding your package.

The packages in your workspace doesn't need to be of the same ament, and you can have as many as your computer and memory allows it.

Now, let's create one package with one simple node that says "hello world", before running the commands, make sure you are located on the workspace on your terminal.

- **CMake:**
    
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package --license Apache-2.0

- **Python:**

    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package --license Apache-2.0

So, the basic structure of the command is:

    ros2 pkg create --build-type ament_<type> --license <LICENSE> <package_name>

And the aditional argument *--node_name* results in the creation of a simple Hello World node.

After the creation of the packages, you will need to build and compile, as you should remember, we use *colcon*:

    cd ~/<your_workspace>
    colcon build

If you want to compile and build a single package, you can add the flag *--package-select*:

    colcon build --packages-select <package_name>

Also, you will need to set up the files, so you will need to run:

    source ~/<your_workspace>/install/local_setup.bash

Another option is to add it to your .bashrc, so every time that you open a new terminal, it has the setup for execution of nodes:

    echo "source ~/<your_workspace>/install/local_setup.bash" >> ~/.bashrc

If you creating the "Hello World" node, you should be able to run it after the building and set up with:

    ros2 run <package_name> <node_name>

NOTE: Do not forget to check your **package.xml** files and update them with your info and the proper info of the package. This will also apply for the **setup.cfg** files with Python Ament.

## Communication in ROS2

You have already read about topics, service and actions, but you may still have a question rounding around your head... Why use to use them and why in ROS?

Without communication, the info will be static and only present to one node, this is not functional in the case of computers and robots, because they are composed of different parts and will also need info that they do not have (for example, present in Internet). Here, ROS present a standard communication (with different types) that allows your robot to connect with a computer, other robots or event itself, having multiple nodes communicating is better that having just one program that does everything and if it fails all is stopped.

In the case of the topics, they imply an unidirectional communication (pub and sub), for example, we have one node that process the velocity and another node that controls a motor, so the first one publish the desired speed and the second one subscribes to it, then processes it and make the motors work to achieve the goa.

In the case of the services, as a bidirectional communication based on req/resp, it can be useful in the case of robotics arm where you specify a pose you need to reach and send a request, a server process and calculates the cinematic of your robot and respond to you with valid position that you will need to set.

And finally, for the case of actions, as they work to achieve a goal while keeping a feedback, it can be also done for robotics arm in cases where you have an array of position to reach, and you get feedback of the pose calculated until it reaches the goal and sends it to you.

But... how can this be implemented? For this we will use client libraries for Python and CPP, so keep reading to learn more.

## Python and ROS2 Communication:

As you may suppose, we will need a package with **amment_python** set up, we will be working with the package [m02_ros2_with_py](/m02_ros2_communication/m02_ros2_with_py/) and you can make the modifications you want to learn more, or even duplicate files and then edit them to practice, but if you want to create your own package, you can always do it, remember that you can use:

    ros2 pkg create --build-type ament_python --license Apache-2.0 <your_python_package_name>

All the codes that aims to use Python muss import the **rclpy** (ROS Client Library for Python) to be compatible with ROS, with that say, let's move to our first examples.

### Publisher and subscriber with Python:

We will create a simple publisher and subscriber for integers of 64 bits, keep in mind that most of the implementation in ROS2 are OOP (Object Oriented Programming), if you are not familiarized with the topic, check the resources at the bottom.

NOTE: Almost of the basic types you used in all-days programming (stiring, ints, floats...) can be used in ROS for communication, by using the official implementation of them in the **std_msgs** or **standard message**, more details can be found on the official site of documentation for [std_msgs](https://docs.ros2.org/galactic/api/std_msgs/index-msg.html)s
https://docs.ros2.org/galactic/api/std_msgs/index-msg.html
The publisher in our case will be the [int64_pub.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/int64_pub.py) file, you can check the comments to understand it better, but the idea for implementing it follows the next items:

1. Import **rclpy** and the node libraries.
2. Import the message type of your interest (in this case is **Int64** from **std_msgs**).
3. Create a class that inherates from **Node**
4. In the constructor, give a name to the node, create an instance of publisher objectwith the type, topic name and queue. Also, you may need a timer.
5. Create a method of the class as a timer callback that will publish when the time indicates it.
6. In the main, instance an object of your node and make a **spin** (a loop in ROS to still publishing when done the previous one).
7. Do not forget to call the destroy and the shutdown functions.

And for the case of the subscriber, in our case the [int64_sub.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/int64_sub.py), the steps are the following:

1. Import **rclpy** and the node libraries.
2. Import the message type of your interest (in this case is **Int64** from **std_msgs**). Must be the same of the publisher.
3. Create a class that inheritates from **Node**
4. In the constructor, give the name to the node, declare an instance of object with a type, topicname, callback function (used for reading and processing the info received) and the queue, do not forget that this should be compatible with the publisher.
5. Define a method that will act as your subscriber callback function to print/process/save the info received.
6. In the main, instance an object of your subscriber node and make a spin of it.
7. Finally, add the destroyer method and shutdown function.

This are the codes to implement this types of communication, but they aren't ready to be executed, first you will need to make sure that your packages has the related dependencies. As we mentined before, the python codes will need **rclpy**, and also they are using msg types that comes with the **std_msgs** package, for that we will need to modify the *[package.xml](/m02_ros2_communication/m02_ros2_with_py/package.xml)* file of our package and add the next tags:

    <exec_depend>rclpy<exec_depend>
    <exec_depend>std_msgs<exec_depend>

**NOTE**: In our package.xml file, the <exec_depend> tag was replaced with <depend> which is also valid. Also, if you add a package (in this case, turtlesim) and it already has the dependency of std_msgs, you do not need to define it again in the tag. 

Then to make sure, you can execute the code, you will need to configure the **entry points** of your *[setup.py](/m02_ros2_communication/m02_ros2_with_py/setup.py)* file, the corresponding structure that is "<exec_name> = <package_name>.<node_name>:main", in the present case would be:

    entry_points={
            'console_scripts': [
                    'pub_int64 = m02_ros2_with_py.int64_pub:main',
                    'sub_int64 = m02_ros2_with_py.int64_sub:main',
            ],
    },

Finally, before executing, make sure the the *[setup.cfg](/m02_ros2_communication/m02_ros2_with_py/setup.cfg)* file is propertly populated like this:

    [develop]
    script_dir=$base/lib/<package_name>
    [install]
    install_scripts=$base/lib/<package_name>

To execute them, you will need to build them and configure your terminal, the commands are:
    
    cd ~/<your_workspace>
    colcon build --packages-select m02_ros2_with_py
    source install/local_setup.bash

Then, in two terminal, you have two run one of the next to commands:

    ros2 run m02_ros2_with_py pub_int64
    ros2 run m02_ros2_with_py sub_int64

After running this, you should be able to see that in one terminal, the pub is sending information, and the sub is receiving information, like the following case:

![py_pub_sub_int64](/m02_ros2_communication/resources/rclpy_int64_pub_sub.png)
****
If you want to practice and learn more, you can try to change the type of message, for example, send a float instaed of an integer, or go on and discover more **std_msgs**.

### Servers and client with Pyhton:

The basic for using services is that there is a request and a response, the first one is sent by the client and received by the server, which should process it and send a response.

As we already have our package, we will only add the sources code and the related dependencies to run the executables.  In the first place we will use a example interface provided in the ROS2 examples, then we will create a custom **srv** file.

For using the example interfaces (and this apply when using messages or services present in other packages), you will need to add them in the dependencies of your package, most specific in hte *package.xml*, in this case, you need to add:

    <depend>example_interfaces</depend>

The definition that we will focus is *AddTwoInts* which structure is:

    int64 a
    int64 b
    ---
    int64 sum

Remember that the characters **---** indicate the separation of the request and the response of the service. with this said, no it is time to explore the creation of the nodes.

On one hand, we have the service server node ([add_two_nums_srv.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/add_two_nums_srv.py)), the steps for creation are:

1. Add the imports of ROS related with the **rclpy** library
2. Add the imports related with the service file, in this case, we will use the *example_interfaces* for *AddTwoInts*.
3. Create a class for the service that inheritates from the Node class.
4. Define a callback function that will manage the request, in this case, the sum of two numbers.
5. Define the construct for initialize the node nam and instance a service object, which must include the *srv* description and channel, and the callback.
6. For the main implementation, initialize **rclpy**, instance a server object and use a spin to allow the server for making more than one connection.

On the other hand, you need a client to make use of the server service, for this check the code [add_two_nums_cli.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/add_two_nums_cli.py) while considering the steps that are listed as follow: 

1. Add the imports of ROS related with the **rclpy** library.
2. Import the related service file, should be consistent with the ones in the server.
3. Create a class that inheritastes from the node.
4. Define a constructor that initialize the node, instance a client object and implement a timeout that wait for the service to be available.
5. Define a request method that will pass the arguments of the request, call the server and get a response as a return.
6. In the main implementation initialize **rclpy**, instance a client object, receive or store values for the request, call the request method with the values considered and use the returned value for future processes.

NOTE: If you want you receive parameters from the user with the terminal, you will need an implementation of the *sys* library to manage the inputs in your code, this is shown in the code of the client. But it is not the only way to recieve values for the request.

After you have written/checked your codes, you should remember to add the entrypoints so you can execute them after building your packages, in this case, add the next info to your [setup.py](/m02_ros2_communication/m02_ros2_with_py/setup.py) file:







## C++ and ROS2 Communication:



# Troubleshooting:

- If you get a warn equal or related to: "SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn (...)". It means that the package 'setuptools' isn't in the proper version for ros2, you can resolve (according to [ros.answer](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)) with the next command (only ROS2 Humble):

    pip install setuptools==58.2.0