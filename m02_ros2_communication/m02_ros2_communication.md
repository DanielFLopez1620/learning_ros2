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

All the codes that aims to use Python muss import the **rclpy** (ROS Client Library for Python) to be compatible with ROS, but before we move to some examples and explain the communication, you have to understand the basics of a **amen_python** package with the **setup.py**, **setup.cfg** and the **package.xml** files.

### *Ament_python* basics:

For making a *package* you need some files, so *colcon* can understand it is a package. We will explore some of the basics for each file and the commands/keywords needed for each one.

- **setup.py** : It is the file where you specify your package name, the basic set up (for versions, data files, exclusions and install requirements), the package info of the maintainers and devs, the license and the entry points (where you add what will be your executables). Here you can find a template, and there you can see that it is pretty clear what is the idea of each item: 

    from setuptools import find_packages, setup

    package_name = '<name_of_package>'

    setup(
        name=package_name,
        version='<version>',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='<your_username>',
        maintainer_email='<your_email>',
        description='<a_clear_and_illustrative_description>',
        license='<license_chose>',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                '<executble_name> = <package_name>.<node_file>:<main>',
            ],
        },
    )

- **setup.cfg:** Used to indicate where are the scripts that will be used as entry points, and where will be installed the libraries. This is a part related with the 'colcon' build process after it ahs added the packages to the build dir, and there locates the executables and libraries.

- **package.xml:** It is known as the package manifiest, here you add the most important info of the package. It is formed by XML tags, so let's explore some of them:

    - <package fomrmat=#3></package>: Tags that contains all of the info of the package, format 3 is relativ to ROS2, at least during Foxy and Humble version.

    - <name></name> Contains the package name.

    - <version></version> Stores the version of the package. Usually with three numbers separeated by points.

    - <description></description> Used to add a short, but illustrative explanation of what you package's purpose is.

    - <maintainer email=""></maintainer> Here you add your name (or ROS User or Github/Gitlab user) and your email, to indicate that you are the one in charge of the package.

    - <license></license> One of the most important related with a license (official name) that indicates limitations that other people have when using your codes.

    - <depend><depend> Add a general dependency (execution, build or other) to your package, the dependencies are other packages that contains msg, srv, interfaces or nodes needed for your packages.

    - <runtime_depend></runtime_depend> Dependency needed only for runtime operations.

    - <test_depend></test_depend> Dependency needed for test about code quality, format or functionlity.

    - <export></export> For exporting info or data of the package.

    - <build_type></build_type> Export needed for specifying the ament_type, in this case, it will be *ament_python*.

After checking this, let's move to our first way to communicate using *Python* and *rclpy*.

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
If you want to practice and learn more, you can try to change the type of message, for example, send a float instead of an integer, or go on and discover more **std_msgs**.


### Servers and client with Pyhton:

The basic for using services is that there is a request and a response, the first one is sent by the client and received by the server, which should process it and send a response. As we already have our package, we will only add the sources code and the related dependencies to run the executables.

Always keep in mind that for using message or services, we need to have an already defined implementation, which can be in another package that we can add to the depedencies of our package. For implementing your own custom messages/services you will need to use packages with ament_cmake and there write the raw implementation for **msg**, **srv** and **act**, this will be covered more ahead in this module when we got to talk about CPP, rclcpp and ament_cmake. For now, we will use two interfaces, one in the package called **example_interfaces** and another one in the **std_srvs**. If **std_msgs** work for you and you made the desktop installation, you may probably have both, but if you do not, you can try installing it with apt, for example:

    sudo apt install ros-humble-example-interfaces

For using custom interfaces that are present in other packages (and this apply when using messages or services present in other packages), you will need to add them in the dependencies of your package, most specific in hte *package.xml*, in this case, you need to add:

    <depend>example_interfaces</depend>

The definition that we will focus is *AddTwoInts* which structure is:

    int64 a
    int64 b
    ---
    int64 sum

Remember that the characters **---** indicate the separation of the request and the response of the service. with this said, no it is time to explore the creation of the nodes.
 
In the ROS2 tutorials, there is a simple adder implementation, here we will try the same but with some updates and improvements.

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

    'add_nums_cli = m02_ros2_with_py.add_two_nums_cli:main', 
    'add_nums_srv = m02_ros2_with_py.add_two_nums_srv:main', 

Then, you will need to build it, source and finally you can execute them, the commands are listed here

    # Terminal 1
    cd ~/<your_workspace>
    colcon build --packages-select m02_ros2_with_py
    source install/local_setup.bash
    ros2 run m02_ros2_with_py add_nums_srv

    # Terminal 2 
    cd ~/<your_workspace>
    source install/local_setup.bash
    ros2 run m02_ros2_with_py add_nums_cli 16 20

![add_two_nums_py](/m02_ros2_communication/resources/rclpy_add_two_nums.png)

The arguments passed in the client are mandatory to achieve the sum, if the command is send without them you will get a warn and the node will close itself. In this example, the client will end process after it displays the message, but you can invoke it as many times as you want with all the numbers you can imagine.

Now, let's try to use an official service provided by the packages, in this case, we can explore the **std_srvs** (standard services), if you want to know more about them, you can run:

    ros2 interface list | grep std_srvs

You will obtain a list that contains the *Empty*, *SetBool* and *Trigger* services, if you want to see the service description you can run:

    ros2 interface show std_srvs/srv/SetBool

Which will show you this:

    bool data # e.g. for hardware enabling / disabling
    ---
    bool success   # indicate successful run of triggered service
    string message # informational, e.g. for error message

You can check the implementation for the server in the [set_bool_srv.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/set_bool_srv.py) file, and the client in the [set_bool_cli.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/set_bool_cli.py), keep in mind, that this isn't a serious implementation as it uses random states and it is only a short example. Do not forget to add the *entrypoints* in the [setup.py](/m02_ros2_communication/m02_ros2_with_py/setup.py) file:

    'set_bool_srv = m02_ros2_with_py.set_bool_srv:main',
    'set_bool_cli = m02_ros2_with_py.set_bool_cli:main',

After building, you should be able to execute them with the next commands:

    ros2 run m02_ros2_with_py set_bool_srv
    ros2 run m02_ros2_with_py set_bool_cli 1

![set_bool_py](/m02_ros2_communication/resources/rclpy_set_bool.png)

### Using parameters in Python

Parameters can be considered node configurations you can change even when the node is running, you can use them to change the name of an object, indicate a new objective or make a call according to the value of the node. You can make this implementation with a *rclpy* node. Our example is the code [saying_hi.py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/saying_hi.py), where we publish on the terminal a simple message related with a param called **your_name** that is used for the node to change the name to the one who it is saying hello.

After you have added the entrypoints, and compile the package, you can run it with the next command:

    ros2 run m02_ros2_with_py saying_hi             # Terminal 1
    ros2 param set /saying_your_name your_name dan  # Termianl 2

The idea for implementing a parameter with rclpy is:

1. Import ROS2 headers as *rclpy*, *rclpy_node* and *Parameter_Descriptor*.
2. Create a class that inheritates from Node.
3. In the constructor initialize (with the parent constructor) the name of the node, then define a parameter descritor, the declaration of the parameter and for testing you can use a timer linked to a callback.
4. Create a method taht will act as a callback, where you update the value and set it back to the original after specified time.
5. In the main, initialize rclpy, instance a node, spin the node and do noto forget the destruction and shutdown.

### Resume on important **rclpy** commands:

Now, let's move our attention to some important commands and instructions for developing nodes with **rclpy**:

- **```rclpy.init(args=<args>)```** : Command to initialize the rclpy, <args> are related with the *argc* and *argv* equivalent from C and C++.

- **```rclpy.spin(<instance>)```** : Implementation for making iterations of the <instance> specified, in most cases will be the node object.

- **```rclpy.shutdown()```** : Last command of the program, closes rclpy dependencies.

Also, let's mention some important commands when making your implementation of publishers and subscribers with **rclpy**:

- **```<node_self>.publisher_ = <node_self>.create_publisher(<type>, <topic>, <queue>)```** : Instance a publisher with name 'publisher_', with a <type> that must have been imported, <topic> a string that will be the name of the channel and the <queue> in order to specify the max size of it.

- **```<node_self>.publisher_.publish(<msg>)```** : Using the publisher called 'publisher_' send (publish) a given <msg> which must be consistent with the definition of the message.

- **```<node_self>.subscription = <node_self>.create_subscription(<type>, <topic>, <callback>, <queue>)```** : Instance a subscriber with the name 'subcription' with a <type> that must have been imported, a <topic> which is a string that provides the name of the channel, and the respective <queue>. It also has a <callback> that will be called when something is received on the <topic> channel.

Another important aspect is related with the servers and clients, for your implementation you should know about:

- **```<node_self>.server = <node_self>.create_service(<type>, <srv>, <callback>)```** : Instance a service server with the name 'server' that has a specific service <type> with its own request/response type, that will use the <srv> channel provided and links a <callback> function that will act when a request is received.

**NOTE**: Do not forget that for callbacks functions in services you need to add a request and a response, for taking advantage of the params.

- **```<node_self>.cli = <node_self>.create_client(<type>, <srv>)```** : Instance a service client whith the name 'cli' that has a specific <type> (imported previously) and a <srv> channel.

- **```<node_self>.future = <node_self>.cli.call_async(<req>)```** : Make a call with a given request <req> using asynchronous communication, the result will be assigned to the 'future' variable.

- **```rclpy.spin_until_futur_complete(<node_self>, <node_self>.future)```** : Iterate and wait until response is received in the case of a 'future' refereing to service clients.

Some general commands and interface, are related with logging, timers and other aspects from **rclpy** are also mention as follow:

- **```super()__init__(<name>)```** : When we have a class that inherits from node, you need to pass to the parent interface a name to initialize the node.

- **```<node_self>.get_logger().info(<str>)```** : Log from the node, in this case, an info report consisting on the <str>, it can be also used for *warn*, *error*.

- **```<node_self>.timer = <node_self>.create_timer(<period>, <callback>)```** : Instance a timer with name 'timer', that has a <period> in seconds and links a <callback> function that will be called when the period is achieved.

- **```<param_desc> = ParameterDescriptor(descritipon = "<str>")```** : Add a parameter descriptiron that can be used to explain the declaration of a ROS2 Parameter.

- **```<node_self>.declare_paremeter(<name>, <defaul_value>, <param_desc>)```** : Declare a ROS2 paramaeter with a given <name> and a <default_value>, you can also add a parameter descriptor <param_desc>.

If you want to learn more about **rclpy**, you can check the official **[API of rclpy](https://docs.ros2.org/foxy/api/rclpy/api.html)**, where you can find more commands and a better explanation of the arguments that can be used for each function.


## C++ and ROS2 Communication:

Now we change of language, then we will need a new package, bu tthis time we will use **amment_cmake** set up, we will be working with the package [m02_ros2_with_cpp](/m02_ros2_communication/m02_ros2_with_cpp/) and you can make the modifications you want to learn more, or even duplicate files and then edit them to practice, but if you want to create your own package from the beginning, you can use:

    ros2 pkg create --build-type ament_cmake --license Apache-2.0 <your_python_package_name>

All the codes that aims to use C++ muss include the **rclcpp** (ROS Client Library for C++) to be compatible with ROS, now, let's explore the same cases studied with Python "traslated" to C++.

Keep in mind that when using **ament_cmake**, you will have two folders by default the *include* directory where you must add you libraries and header files, and the *src* where you add the source code (for future executables). So, let's start by watching the package configuration.

### *ament_cmake* basics:

You have already checked on **ament_python**, and now we have to focus on **ament_cmake**, yes (not ament_cpp, it is because of the **cmake and make** build tools). Here you share the *package.xml*, but you now have a *CMakeList.txt* file that can be considered an equivalent of the setup.py to set the executables and the dependencies.

- **package.xml** : The main chance is related with the <export></export> tags, where you no longer need the <build_type>ament_python</build_type> but instaed the **<build_type>ament_cmake</build_type>**.

- **CMakeLists.txt** : Cmake isn't a build tool but rather a support to use Make (build tool) in a better way, it serves to indicate the depedencies and how to compile (as we are in C++ and need a compiler) the source code to obtain the executables, so let's mention some important commands:

    - **cmake_minim_required(VERSION <ver>)** : Indicates the version needed.

    - **project(<name>)** : Add the name of your package (project)

    - **find_package(<package> REQUIRED)** : Search for package that is considered to be a dependency.

    - **add_executable(<exec_name> <path>)** : Set an executable with a specified name by using the .cpp file in the provided path.

    - **ament_target_depencies(<exec_name> <libraries>)** : Link the executable with the corresponding dependencies.

    - **intall(TARGETS <exex_name> DESTINATION lib/${PROJECT_NAME})** : Add the executables to the project library to be used then as nodes.

    - **ament_package()** : Final tag that reminds of the ament type used.

With that said, we can explore the communication methods from another point of view, in this case, *rclcpp*, so keep going...

### Publisher and subscriber with C++:

We will replicate the integer of 64 bits case from the previous examples, but in this case using *rcplcpp*, it will also have an OOP implementation. Also, as said in the Python Module, you can implement any basic data type, you can also create your custom types by generating structures of the common data types, we will cover this later.

On one hand, our publisher will be [int64_pub](/m02_ros2_communication/m02_ros2_with_cpp/src/int64_pub.cpp) file, do not forget to check it and consider the comments presented there. The idea for implementing a publisher is:

1. Include needed libraries, you will need *chrono* (for time tracking), *functional* (related with hashes), *memory* (for dynamic memory), *string* (if you want to use strings) and you may need other standard libraries according to your needs like *cstdlib* or *iostream*.
2. Include ROS2 Libraries, the main one the proper *rclcpp.hpp* header, but also you will need the interface you are goint to comunicate (*int64.hpp* in this case).
3. Configure the proper namespace for chrono
4. Create a publisher class that inheritates from Node.
5. In the public interface, add the constructor that initialize the node by providing a name, initilaize your variables, instance a publisher with the proper type and channel you need, and create a timer that link a callback.
6. In the private interface, add the timer callback to publish the desired integer and log the info.
7. In the private interface declare the private publisher and timer as shared pointers of their specific types.
8. Create the main, initialize rclcpp, spin the instance of the node and finally add a shutdown.

On the other hand, our subscriber will be the [int64_sub.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/int64_sub.cpp), let's me tell you the steps:

1. Include needed libraries, in this case we will need memory (for dynamic memory) and you can use any other standard library needed to process the data received.
2. Include ROS2 Libraries, the main one the proper *rclcpp.hpp* header, but also you will need the interface you are goint to comunicate (*int64.hpp* in this case).
3. Configure a namespace related to the place holders.
4. Create a publisher class that inheritates from Node.
5. In the public interface, add the constructor that initialize the node by providing a name, initilaize your variables, instance a subscriber that links a callback for managing incoming data.
6. In the private interface add the subscriber callback that will process the data received.
7. In the private interface, declare the private subscriber as a shared pointer related to the type of the msg of interest.
8. Create the main, initialize rclcpp, spin the instance of the subscriber class and finally add a shutdown.

As you may know, you need to compile the codes and create the executables in order to run cpp codes. For this, we will take advantage of *Make* and *CMake* to build our package, but first... we need to configure our manifiest, the [package.xml](/m02_ros2_communication/m02_ros2_with_cpp/package.xml) file, with:

    <exec_depend>rclpy<exec_depend>
    <exec_depend>std_msgs<exec_depend>

**NOTE:** If you check the *package.xml* file, you should have noticed we hace changed the <exec_depend> with <depend>, and also, as we have imported *turtlesim*, we ignore the *std_msgs* inclussion as we have already had it in the *turtlesim* package.

Then, we can use the [CMakeLists.txt](/m02_ros2_communication/m02_ros2_with_cpp/CMakeLists.txt) file to add the building configuration, for this case, it will be:

    # Add package dependencies
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # Add executables
    add_executable(pub_int64 src/int64_pub.cpp)
    add_executable(sub_int64 src/int64_sub.cpp)

    # Link dependencies
    ament_target_dependencies(pub_int64 rclcpp std_msgs)
    ament_target_dependencies(sub_int64 rclcpp std_msgs)

    # Obtain/install executables
    intall(TARGETS
        pub_int64
        sub_int64
        DESTINATION lib/${PROJEC_NAME}
    )

The previous allow us to make the compilation, link dependencies and locate the executables so after colcon build, we know where they are and call them to run its content.

Before you run, make sure you source your workspace, and then, you can run:

    ros2 run m02_ros2_with_cpp pub_int64 # Terminal 1
    ros2 run m02_ros2_with_cpp sub_int64 # Terminal 2

![int64_pub_sub_cpp](/m02_ros2_communication/resources/rclcpp_int64_pub_sub.png)

Try to practice creating your own pub/sub with a type of your interest in the *std_msgs* library, and check what can you make.

### Servers and clients with C++:

We will make an implementation of the adding srv that we check previously with Python, the idea is the same, connect a server and a client by using a service which have a request and a response component. If you haven't install the *example_interfaces*, do it with the command:

    sudo apt install ros-humble-example-interfaces

For custom interfaces (made by someone in the community or by you), you need to add the package as a dependency, firstly in your *package.xml*, as we made before in Python:

    <depend>example_interfaces</depend>

The definition we are gonna use is the same, related with *AddTwoInts*, which is presented here:

    # Request
    int64 a
    int64 b
    ---
    # Response
    int64 sum

One curious fact, is that it is called due to the objetive (to add nums), but it is a generic definition, that just sends to numbers and ask back for just one. So, you can use this interface to substract, multiply and divide (but returning a integer), keep in mind that a good practice is to follow the convention of the name, but prevent to use specific names to general interfaces, for example, this interface could be called *OperateTwoNums* which could be implemented in different services that make a different operation. Keep this like an idea on how to improve your projects.

But continuing with what call us here, is the implementation of the adder, it is the same of the ROS2 tutorials, but with some additional explanations in code.

On the first hand, we need to stablish a server node that is related with our [add_two_nums_srv.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/add_two_nums_srv.cpp), the steps to consider are below:

1. Include standard headers, in this case our focus will be *memory* (for managing dynamic memory).
2. Include ROS hearders, do not forget about *rclcpp* header and the one with the header of your *srv*, in this case *add_two_ints.hpp*.
3. Declare a callback that will be linked when a request has been received, it must hve to params as shared pointers, the request and the response with the proper types of the service, then include the processing of the data received.
4. Create a main and initialize rclcpp and instance a object of the node class.
5. In the main, instance a service with an unique name and link the *srv* definition included, taking advantage of the node, create a log of status ready of the server and spin so you allow the server to stay in function continuously.

On the second hand, we create the client node, check the [add_two_nums_cli.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/add_two_nums_cli.cpp) file for more info. The steps to cconsider are:

1. Include standard headers, in this case, it is needed to use *chrono* (for time tracking), *memory* (dynamic memory usage) and libraries need for your proper implementation, for example, *cstdlib* (General purpose functions).
2. Include ROS hearders, do not forget about *rclcpp* header and the one with the header of your *srv*, in this case *add_two_ints.hpp*. Which must be consistent with the server.
3. Add a namespace for using of chrono.
4. Create a main and initiliaze rclcpp.
5. In the main read the arguments passed with the call and check the number is valid and consistent.
6. In the main instance a node with a unique name, then instance a client with a name and add the corresponding *srv* imported, to make it consistent with the definition of the server.
7. In the main, stablish the *request* (by considering the args passed) and send it.
8. Wait for response and determinate an action in case the response was a success, or in case no response were given.
9. Finally, shutdown the node.

To run this example, make sure you add the corresponding files to the [CMakeList.txt](/m02_ros2_communication/m02_ros2_with_cpp/CMakeLists.txt) to create the executables:

    # Adding Two Nums Server
    add_executable(add_nums_srv src/add_two_nums_srv.cpp)
    ament_target_dependencies(add_nums_srv rclcpp example_interfaces)

    # Adding Two Nums Client
    add_executable(add_nums_cli src/add_two_nums_cli.cpp)
    ament_target_dependencies(add_nums_cli rclcpp example_interfaces)

    # Obtain/install executables
    intall(TARGETS
        add_nums_cli
        add_nums_srv
        DESTINATION lib/${PROJEC_NAME}
    )

After you have used colcon to build the packages, and sourced the setup file, you can run:

    ros2 run  m02_ros2_with_cpp add_nums_srv # Terminal 1
    ros2 run  m02_ros2_with_cpp add_nums_cli 16 20 # Terminal 2

![add_ints_cpp](/m02_ros2_communication/resources/rclcpp_add_ints.png)

You have now explored, in both Python and C++, the communication needed for your robots with ROS2, but... what if you need a custom msg or service? Well, you can do it, let's explore it in the next header.

### Using custom interfaces with *ament_cmake*:

No matter it is the case where you need to define a custom *msg* for a specific work, or you just want to test your own *srv* implementations, you will need to have those descriptions in a separated packages.

For this, you will need a package with *ament_cmake* in order to configure the interface, export it or even use it in the same package. We will start by creating to important directories, **msg** and **srv** dirs:

    cd <path_to_your_package>
    mkdir srv msg

After that, you can create your own interface for msg or srv, in this case, we created both, the [RarePoint.msg](/m02_ros2_communication/m02_ros2_with_cpp/msg/RarePoint.msg) and the [Answer.srv](/m02_ros2_communication/m02_ros2_with_cpp/srv/Answer.srv) interfaces, that you can check there. Remember to follow the structure of the message (using valid types) and the service (request/response structure).

The next step is to make available, the structure, for that, on the [CMakeLists.txt] file, make sure you add:

    set(msg_files
        "msg/RarePoint.msg"
    )

    set(srv_files
        "srv/Answer.srv"
    )

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES std_msgs
    )

If you want to use the same definition on the package, you will need to configure the type support target in the CMakeLists.txt:

    rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} rosidl_typesupport_cpp)
    
    target_link_libraries(<executable> ${cpp_typesupport_target})

For testing this interface, I added two source codes, one publisher and one server, they are the [rand_xy_pub.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/rand_xy_pub.cpp) file and the [exam_srv.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/exam_srv.cpp) file, after you have added the executables, and link the custom interface, let's try them:

    ros2 run m02_ros2_with_cpp rand_xy_pub # Terminal 1
    ros2 topic echo /rare_point # Terminal 2

![rand_xy_cpp](/m02_ros2_communication/resources/rclcpp_rand_xy.png)

    ros2 run m02_ros2_with_cpp exam_srv
    ros2 service call /exam_channel m02_ros2_with_cpp/srv/Answer "{option: 3}"

![exam_cpp](/m02_ros2_communication/resources/rclcpp_exam.png)

Now that you have discovered how to create your own interfaces, try to create the codes of the csubscriber for the *rand_xy_publisher* node, and a client for the *exam_srv*, you can even try it with Python. Just remember to add the corresponding dependencies.

### Parameters with cpp:

Let's also mention *ROS2 Parameters* which are used for configurations or set up processes inside the nodes. This parameters can be set on the terminal, but as you will discover soon, you can also use *launch* files.

For this purpose, we will use the code [saying_hi.cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/saying_hi.cpp), which allow us to configure a simple parameter, and then it go back to the original.

Let's mention the idea of the code:

1. Include needed standard libraries, in this case *chrono* (time management), *functional* (related with funtion implmentation) and other needed for you main process, for example, *string*.
2. Include ROS2 related headers, do not forget about *rclcpp*.
3. Creata a class that inheritates from Node.
4. In the public interface, create the constructor, initialize the name of the node, declare a parameter and create a timer that will be linked to a callback.
5. In the public interface, define a method which will act as a callback of the timer.
6. In the private interface, do not forget to add the timer definition.
7. Create a main, initialize rclcpp, spin the instance of the object that will be used to manage the param and add a final shutdown.

You do not need extra tags or configs in CMake to make this node work, you only have to add the executable, link the dependencies and install the target as you have done before in the *CMakeLists.txt* of the package. You can execute it here with:

    ros2 run m02_ros2_with_cpp saying_hi               # Terminal 1
    ros2 param list                                    # Terminal 2
    ros2 param set /saying_your_name your_name dan     # Terminal 2

![params_cpp](/m02_ros2_communication/resources/rclcpp_param_c.png)

### Resume of important *rclcpp* commands:

Now, let's mention some important commands that you should keep in mind, as the order we previously managed, we will start with publishers and subscribers:

- **```rclcpp::Publisher< <msg_type> >::SharedPtr <publisher>;```** : Base declaration of a publisher.

- **```<publisher> = <node_obj>->create_publisher< <msg_type> >(<topic>, <queue>);```** : Instance a publisher with a specified <msg_type> that will use the topic called <topic> and have a defined <queue>.

- **```<publisher>.publish(<msg_content>);```** : Publish a valid message (according publisher definition) and sends the <msg_content>.

- **```rclcpp::Subscription< <msg_type> >::SharedPtr <subscriber>;```** : Base declaration for a subscriber.

- **```<subscriber> = <node_obj>->create_subscription< <msg_type> >(<topic>, <queue>, <callback>);```** : Instance a subscriber with the corresponding <msg_type>, related with the publisher. It should consider the same <topic> and <queue>.

We also, need to talk about important commands when implementing our own service clients and serves, here we have:

- **```rclcpp::Service< <srv_type> >::SharedPtr <server>;```** : Base declaration of a service server.

- **```<server> = <node_obj>->create_service< <srv_type> >(<service>, <callback>);```** : Instance a service server with a specified <srv_type> that contains the request and response structure, set the configuration to use the given <service> channel and link a <callback> for incoming responses.

- **```rclcpp::Client< <srv_type> >::SharedPtr <client>;```** : Base declaration of a service client.

- **```<client> = <node_obj>->create_client< <srv_type> >(<service>);```** : Instance a service client with a specified <srv_type>, compatible with the one of the server, and pass the <service> channel that will be used.

- **```auto <request> = std::make_shared< <srv_type>::Request>();```** : Declare a request that will be used by a client.

Finally, do not forget about general commands when using *rclcpp*:

- **```class <NameClass> : public rclcpp::Node```** : Create a class that inheritates from Node class.

- **```Node (<str>)```** : Initialize Node object with a name <str>, it can be used when inheritate is used.

- **```rclcpp::init(<argc>, <argv>);```** : Initialize rclpp with the corresponding args from the terminal call.

- **```rclcpp::spin(<instance>);```** : Spin can be considered as an alike loop implementation for a given <instance>.

- **```rclcpp::shutdown();```** : Do not forget to add it at the end of your main.

- **```RCLCPP_INFO(<node_obj>->get_logger(), <str>);```** : Log <str> to display status, info or related, its structure is similiar to the ones with warns and errors.

- **```rclcpp:TimerBase::SharedPtr <timer>;```** : Base declaration of a timer.

- **```<timer> = <node_obj>->create_wall_timer(<milliseconds>, <bind_callback>)```** : Instance a timer that will have a time of given <milliseconds> and everytime is completed, will call the <bind_callback> function.

# Playing with Turtlesim

You have played with the terminal, now it is time to play with a 2D robot. As you may remember, **turtlesim** has a collection of topics, services and actions you can interact with, then it is good for learning, testing and experimenting.

    ros2 run turtlesim turtlesim_node

Remember, you can list the topics, services and parameters, like follow:

    ros2 topic list
    ros2 service list
    ros2 param list

In case you can use a topic, you can get info of it with:

    # Command
    ros2 topic info /turtle1/cmd_vel

    # Output
    Type: geometry_msgs/msg/Twist
    Publisher count: 0
    Subscription count: 1

And it provides the type (message type), the publisher and subscriber count. If you want to interact with the topic, you have to understand if it is a subscriber (so you need to publish in order to interact) or it is a publisher (then you need a subscriber to tget the proper info), after this, you can check the interface with:

    # Command
    ros2 interface show geometry/msg/Twist

    # Output
    Twist
    # This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z

So, you will need to import the type of message, in the proper way for each language:

- **Python:** from geometry_msgs.msg import Twist
- **C++** #include "geometry_msgs/msg/twist.hpp"

Here I have developed some examples you can check, for both **rcply** and **rclcpp**, they are listed below:

- **simple_turtle_mov:**  It is the implementation of a simple publisher that uses the */turtle1/cmd_vel* topic, then the turtle will move linearly and angularly. You can find the implementation for [py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/simple_turtle_mov.py) and [cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/simple_turtle_mov.cpp).
- 
- **turtle_challenge:** It provides a class to interact with the services of the *turtlesim* that include clear, spawn, kill, set pen and the teleports. It is provided a simple demostration, you can find the implementation for [py](/m02_ros2_communication/m02_ros2_with_py/m02_ros2_with_py/turtle_challenge.py) and [cpp](/m02_ros2_communication/m02_ros2_with_cpp/src/turtle_challenge.cpp).

If you want to interact with the, down below, I list the respective commands, do not forget to execute **turtlesim_node** and build the packages (and source) before using them:

    ros2 run m02_ros2_with_py simple_turtle_mov
    ros2 run m02_ros2_with_py turtle_challenge
    ros2 run m02_ros2_with_cpp simple_turtle_mov
    ros2 run m02_ros2_with_cpp turtle_challenge

# Launches

You may notice that the number of nodes tend to increase, and if you need one terminal for each one... what will happen when you need to run 100 nodes?

Well, there is a solution, it is called **ros2 launch**, and here we will discover how to use them. It is not limited to running a set of nodes, but you can also specify where to run them, how to configure their parameters and then offer a option to monitoring them. They can be made with Python, XMAL and YAML, for more info on the types you can check this [link](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)

Here we will implement them with Python. You can check the documentation and API for more info on Python, or if you want to explore XML and YAML.

AS a convention, for using launches, in your packages, you will need to create a *launch* dir:

    cd <path_to_your_package>
    mkdir launch

You can use *launch* files in a packages with **ament_python** or **ament_cmake**, but you have to properly configure the package, let's watch the cases:

- **ament_python:** In the [setup.py](/m02_ros2_communication/m02_ros2_with_py/setup.py) file, you need to include the *.launch.py* files, the configuration need is:

        from setuptools import setup
        
        import os
        
        from glob import glob
        
        ...

            data_files=[
                ...
                (os.path.join('share', package_name), glob('launch/*.launch.py'))
            ],
        ...

- **ament_cmake:** In the [CMakeLists.txt](/m02_ros2_communication/m02_ros2_with_cpp/CMakeLists.txt) file, you need to add the directory launch:

        ...
        install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )
        ...

Now, it is time to see a launch file, we will use our friends from **turtlesim**, and we will use the nodes previously made for the turtles, the examples files are called [turtlesim_simple_mov.launch.py | Python Package](/m02_ros2_communication/m02_ros2_with_py/launch/turtlesim_simple_mov.launch.py) and [turtlesim_simple_mov.launch.py | CMake Package](/m02_ros2_communication/m02_ros2_with_cpp/launch/turtlesim_simple_mov.launch.py) and explore the content:

- **```from launch import LaunchDescription```** : From the launch module (non-ROS-specific launch framework) import the tools to generate a descriptio.
- **```from launch_ros.actions import Node```** : Import node object description for launch execute.
- **```def generate_launch_description(): return LaunchDescription([ ])```**: Space where the launch description is going to be added.
- **```Node( package='<package_name>', namespace='<namespace>', executable='<exec>' name='<node_name>', remappings=[<remaps>]),```** : Add the configuration to execute a node by passing the name of the package, the name of the executable and the name of the node.Also you can optionally add a namespace and remap of topics.

The previous info presented is the basic one for launching multiple nodes, but you can make even more...
- ### Adding launch configurations and arguments: 
  This will make easier to configure and set up your nodes, while generating common default implementations of your params, and providing an easy way to change them.

  - **```from launch.actions import DeclareLaunchArgument, ```** : Add commands related to arguments configuration and set up.
  - **```<config_var> = LaunchConfiguration(<config_name>)```** : Add a launch configuration, it is recommended that both ```<config_var>``` and ```<config_name>``` are the same. They are passed and can be used with the package, namespace, exeutable, name or arguments params from a node.
  - **```<config_arg> = DeclareLaunchArgument( <config_name>, default_value='<default_value>')```** : Create a launch argument that uses a launch configuration. They can be used above the launch or from the terminal.
  - 

- ### Using substitutions: 
  When you have to configure parameters of multiple nodes, you may miss using macros or variables for this process, but in launches you can use substitutions, which makes a launch more flexible. The commands needed are present in the **launch.substitutions**, **launch.launch_description_sources** and **launch_ros.actions** library, keep in mind the next ones.

  - **```from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression```** : Import the commands relate to add paths to the files, replace text, configure a launch and add python expressions to substitute.
  - **```from launch.launch_description_sources import PythonLaunchDescriptionSource```** : Import the command to use others launch descriptions in the current launch.
  - **```PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare(<package_name>, <dir>, <launch_file>)])])```** : For using the ```<launch_file>```present in the package called ```<package_name>``` under the directory ```<dir>``` in the current launch, the substitution comes handy as with PathJoin it help to find the package an pass the path to the launch.
  - **```'<launch_argument>' : TextSubstitution(text=str(<var/value>))```** : Use text substitutions to make sure you pass the arguments in a way it can be used by the launch configuration.
        
- ### Executing commands inside the launch file:
  You can use the terminal (bash) for launching or adding additional info/commands to your launch in order to generate a proper implementation, or to call commands from the *ros2cli*.
  **```from launch.actions import ExecuteProcess, TimerAction```** : Add commands related with process execution and timers.
  - **```from launch.conditions import IfCondition```** : Add if statement for launches.
  - **```<process_var> = ExecuteProcess(cmd=[[<command_description>]], shell=True)```** : Add a shell command to run, it can be a ros2cli command. And inside the ```command_description``` you can use launch configs.
  - **```<condition_process_var> = ExecuteProcess(condition=IfCondition(PythonExpression([<condition>])), cmd=<command>)```** : Add a condition to execute the process, the ```<command>``` must have the structure previously defined.

- ### Using event handlers:
  When you have multiple process, they may have problems on the way, launches provide options to check if the process started, was completed succesfully, or if it has error, it is exiting or shutting down. Then, you can implement a function, action or behavior in order to robust your launch files. Here some important commands to keep in mind.
  - **```from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown```** : Add commands related to handle events on different parts of the processes (start, end, I/O...)
  - **```from launch.events import Shutdown```** : Add the event when the launch is asked to shutdown (usually when a kill command is executed)
  - **```from launch.actions import EmitEvent, LogInfo, RegisterEventHandler```** : Add a commands for actions that are related with events and display info.
  - **```LogInfo(msg='<content>')```** : Log a message or text, can be linked with actions.
  - **```RegisterEventHandler( <event_to_handle> (<process>))```** : Mark an event to be handled, then link a corresponding action in the ```<event_to_handle>``` which can be on start, on I/O interrupt, on completion, on exit or on shutdown.
  - **```OnProcessStart( target_action=<action>, on_start=[ <process> ]```** : In case that the given ```<action>``` begins, launch the ```<process>``` considered.
  - **```OnProcessIO( target_action=<action>, on_stdout=[ <process> ]```** : In case that the ```<action>``` ask for an input/output process, launch the ```<process>``` considered.
  - **```OnProcessComplete( target_action=<action>, on_completion=[ <process> ]```** : In case that the ```<action>``` is completed (launched with success but still running), launch the ```<process>``` considered.
  - **```OnProcessExit( target_action=<action>, on_exit=[ <process> ]```** : In case that the ```<action>``` exits (or ends), launch the ```<process>``` considered.
  - **```OnShutdown( target_action=<action>, on_shutdown=[ <process> ]```** : In case that the ```<action>``` the launch is asked for shutdown, it executes a final process.
- ### Using config files:
  You can use .yaml file (usually located in the *config* directory) to provide configurations of parameters for you nodes, you must be careful as they depend on the namespace and the name of the topic, their structure is:
  
        <namespace(optional)>/<node_name>:
            ros__parameters:
                <param_name>: <value>

  The usage in the launch file is very simple, you need to provide the path to the config file, and then in the configs of a node, pass the object.

        config = os.path.join(
            get_package_share_directory('<package_name>'),
            '<dir>',
            '<name>.yaml'
            )
        
        return LaunchDescription([
            Node(
                package='<package>',
                executable='<exec>',
                namespace='<namespace(optional)>',
                name='<node_name>',
                parameters=[config]
            )
        ])

If you want to check on the usage of the last commands, you can explore the next launch files:

- **[turtlesim_background.launch.py](/m02_ros2_communication/m02_ros2_with_py/launch/turtlesim_background.launch.py):** Oriented to use actions and substitutions to play with the background, while also executing commands from turtlesim. You can test it with:

        ros2 launch m02_ros2_with_py turtlesim_background.launch.py
        ros2 launch m02_ros2_with_cpp turtlesim_background.launch.py

- **[turtlesim_spawn.launch.py](/m02_ros2_communication/m02_ros2_with_cpp/launch/turtlesim_spawn.launch.py):** Oriented to check events while spawing a turtle in the turtlesim world. You can execute it with:

        ros2 launch m02_ros2_with_py turtlesim_spawn.launch.py
        ros2 launch m02_ros2_with_cpp turtlesim_spawn.launch.py

- **[turtlesim_with_yaml.launch.py](/m02_ros2_communication/m02_ros2_with_cpp/launch/turtlesim_with_yaml.launch.py):** Oriented to use a parameter file for chaning the background of the turtlesim.

        ros2 launch m02_ros2_with_py turtlesim_with_yaml.launch.py
        ros2 launch m02_ros2_with_cpp turtlesim_with_yaml.launch.py


# Troubleshooting:

- If you aren't able to autocomplete (a package), make sure you have succesfully build (using colcon build and the corresponding flags), and also, make sure you have added and sourced the *local_setup.bash* or the *setup.bash* file.

        cd <your_ws>
        colcon build
        source install/local_setup.bash

- If you get a warn equal or related to: "SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn (...)". It means that the package 'setuptools' isn't in the proper version for ros2, you can resolve (according to [ros.answer](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)) with the next command (only ROS2 Humble):

        pip install setuptools==58.2.

- If a yaml file seems to not be loading the parameters, you can check for the next options:
  - ```[WARNING] [launch_ros.actions.node]: Parameter file path is not a file: ...``` If you receive this warn, you should check the *share* path and watch for the real location of the yaml file, for example, if it is in the *config* dir or the directory you specified in the launch.
  - Pay attentation to the namespaces, if you are sure the name of the node and the execution is done properly, maybe you mistype something in the namespace section or you have to use a namespace as the topic, service (...) was launched inside one.
    
# Resources

- OOP Python: [RealPython](https://realpython.com/python3-object-oriented-programming/), [W3School](https://www.w3schools.com/python/python_classes.asp). 

- OOP C++: [W3Schools](https://www.w3schools.com/cpp/cpp_oop.asp), [Geeks for Geeks](https://www.w3schools.com/cpp/cpp_oop.asp)

- ROS2 Client Libraries Tutorials: [Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

- ROS2 rclpy API: [Foxy](https://docs.ros2.org/foxy/api/rclpy/)

- ROS2 rclcpp API: [Foxy](https://docs.ros2.org/foxy/api/rclcpp/)

- ROS2 Launch Tutorials: [Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

