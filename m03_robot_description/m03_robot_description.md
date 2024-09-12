# Robot description:

Here we are going to explore the aspects needed to generate a robot description that can be used for visualization and simulation. We will cover topics related with **tf2**, **urdf**, **sdf**, and other related.

# RVIZ2

TODO: Add info of visualization tool of ROS2 before entering topic.

# TF2

A transform is the basics of understating the system of a robot, it gives you the option to calculate the relations of the different parts of your robot. It can be a mobile robot, like turtlebot3, or a industrial robot, like Universal Robots Manipulators. 

You can understand a transform like an origin with its own axis and rotations. Let's illustrate this, you have two mobile robots exploring a room, we will call them 'mob1' and 'mob2', during the exploration 'mob1' found something interesting and want to tell 'mob2' to come, but... how does he tell the position of the objective?

![robot_exploring](/m03_robot_description/resources/robot_exploring.png)

You can say the position relative to a common origin, but it can get messy if the position of 'mob1' is too complex. Another option is to make the origin at 'mob1', but it get difficult to pass the position and consider 'mob2'.

![random_origin](/m03_robot_description/resources/random_origin.png)

![robot_origin](/m03_robot_description/resources/bot_origin.png)

The final option is related with using transforms... what if we create two origins and consider the transform, so we keep track of the info since the origin to the robot (and even further).

![tf_system](/m03_robot_description/resources/tf_system.png)

A transform is a consideration of the steps needed to go from one origin to another (frame to frame) that consider linear and angular movements, the linear components are expressed by the x, y and z axis, while the angular components are considered as [quaternions from AllAboutCircuits](https://www.allaboutcircuits.com/technical-articles/dont-get-lost-in-deep-space-understanding-quaternions/).

This can also be applied to defined a robotic arm and its joints, and also for explorations. Our focus in TF will search with the **turtlesim**, so then we can move on to describe robots with the context gaining and by using additional technologies.

Some general dependencies you will need are related with the proper *tf2* packages, you can run the command:

```bash
    sudo apt-get install ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim
```

Now, let's move to the practice part, we will begin with Python, this practice is heavily based on the code provided by the package **turtle-tf2-py**, you can check the original info by installing the next package (do not forget to source the package):

```bash
    sudo apt-get install ros2-humble-turtle-tf2-py
```

The practice with turtlesim will make on making frames and tfs with both Python and C++ on ROS2, the objective program will be to create a turtle chaser in different cases.

## TF2 with C++:

Let's begin by creating the package we will use:

```bash
ros2 pkg create --build-type ament_cmake --license BSD-Clause3 --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- m03_tf2_with_cpp
```

The dependencies required for using tf2 are:

- **goemetry_msgs:** For geometric interfaces, like poses, twists and related.
- **rclcpp:** ROS2 Client Library for C++.
- **tf2_ros:** Transform library for ROS
- **tf2:** Trasnform library
- **turtlesim:** Oriented to playing with 2D turtles

Once, we have set up things, we will proceed to explain how to use the transform with C++, then the codes used are based on the [TF2 Tutorial Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html), so if you need more information do not doubt to check them.

### Using a static TF broadcaster:

You can understand a transform broadcaster as a continious publisher of a transforms
that relates to components of a robot, two objects in a world or the world with an object.

Here the focus is to talk about a static tf broadcaster, which, can be used to describe the relationship between a robot base and sensors (LIDARs, cameras, IMUs...) or non moving parts (like sensors supports, chassis, protections...), do not forget this as we will use related terms when designing our robots.

In C++, to create a static transform broadcaster you will need the following libraries:

- **geometry_msgs/msg/transform_stamped.hpp** : To use transforms messages that has attached a stamp in the header for time considerations.
- **tf2/Linearath/Quaternion.h** : Rotations in ROS works with Quaternions rather than RPY system, then we import the quaternion to make the conversions. If you do not know about Quaternions, after this module you will find information about them.
- **tf2_ros/static_transform_broadcaster** : Header that has the information for using the definiton of the static transform broadcaster.

The code for this definition is [static_broadcaster.cpp](/m03_robot_description/m03_tf2_with_cpp/src/static_broadcaster.cpp), which is commented so go aand check it out, but we would like to make some additional highlights of commands presented in the code:

- **```std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broad;```** Instance of a static tf brodcaster by considering a
shared pointer.

- **```tf_static_broad_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);```** : Declaration of a static broadcaster by using a shared pointer and consider the node class itself.

- **```geometry_msgs::msg::TransformStamped transf```** : Instance of a stamped transform, its attributes are *header* with *stamp* (time of creation or considered time for the TF) and *frame_id* (name of the parent frame), *child_frame_id* (which is the name of the child frame), *transform* which has *translation* (considered in 3 components in meters whihc are x, y and z) and *rotation* (considerred as a quaternion in terms of x, y, z and w) 

- **```<quaternion>.setRPY( <r>, <p>, <y>);```** : If you use RPY system, you can create the coordinates in quaternions by using the *setRPY* method.

What will the code do? Well, it will create a custom transform between two origins, where the first one is the **world** (the parent) and the second one with a user provided name. Why the parent is **world**, becasue it is the base origin for all the applications of ROS and simulation, it is the [0,0,0] coordinate and that is the reason we add in the code we compare the name of the transform to check avoid repetitive parent names.

```C++
if (strcmp(argv[1], "world") == 0)
{
    RCLCPP_INFO(logger, "Cannot name child as parent, with name 'world'");
    return 1;
}
```

Then, do not forget to modify the **CMakeLists.txt**:

```CMake
add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(
  static_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
...

install(TARGETS
  static_broadcaster
  ...
  DESTINATION lib/${PROJECT_NAME}  
)
```

After you have build the package, you can run this code with:

```bash
# ros2 run m03_tf2_with_cpp static_broadcaster child_name x y z roll pitch yaw
ros2 run m03_tf2_with_cpp static_broadcaster new_tf 1.0 2.0 0.0 0.0 0.0 0.0
```

You may not notice anything at first,  but if you list the topics, you may notice the **/tf_static** topic, and if you subscribe or make an echo you should notice something like this:


```bash
ros2 topic list
ros2 topic echo /tf_static
```

TODO: Add image of echo tf_static

Also, you can do something interesting if you do not like just watching raw info, you can use **RVIZ2** to check the tfs, for that you can run:

rviz2 -d m03_tf2_with_cpp/rviz/static_tf_view.rviz

The option added with *-d* is to link a file to configure a path to obtain a config file for RVIZ and get the visualization of the panels already set up for a specific situation. The result of the transforms is:

![tf_static_rviz2](/m03_robot_description/resources/rviz2_tf_static.png)

In the image, you can watch the two origins **world** and **new_tf** and the calculation of the tf which is represented by the yellow arrow that connects both. Also, do not forget that for origins, we have that XYZ come in the order of RGB, which means, red axis is X axis, green is Y axis and blue is Z axis. 

### Using a dynamic TF broadcaster:

You may also have the situation where the TFs move, for example, in the case of rotation of parts attached to motors or articulations of a robot. For this cases, you need a continious broadcasting to know details about the robot position and states, then let's check the implementation in C++.

For now, and going on, we will use our favorite turtle friends for the tutorial, and do not worry, every turtle was treated in a peaceful and nice way.

For using the turtles with TFs we need to broadcast according a origin,  and you guess right, according to the **world** frame, then for each turtle must be a broadcater, but this doesn't mean that if we have 9 turtles we need 9 different source files for the nodes, as we will take advantage of the parameters.

The node we will be using comes from the source file called [turtle_broadcaster.cpp](/m03_robot_description/m03_tf2_with_cpp/src/turtle_broadcaster.cpp). At first, we need to change some libraries and include new libraries:

- **tf2_ros/transform_braodcaster.h:** For a broadcast of dynamic transforms, it will replace the static transform broadcaster library we presented previously.

- **turtlesim/msg/pose.hpp:** As we will be playing with turtles in turtlesim, we need to subscribe to the */pose* topic of the turtles to broadcast the correspoding transforms, and we will need this message from the turlesim package.

Now, you can check the full code on the specified file, but some additional highlight (different from the ones mentioned in the static broadcaster) are:

- **```using namespace std::placeholders;```** : Included to manage the argument placement in a shorter way for the callbacks, for example, to simply use _1 instaed of std::placeholders::_1
- **```turtle_name = this->declare_parameter<std::string>("turtlename", "turtle");```** : As we mentioned before, in order to make the broadcaster usable with different turtles, we can take advantages of parameters, here we declare a std::string parameter with a default "turtle" value.
- **```rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subs;```** : Instance of a suscription that uses pose messages of the turtlesim declared by using a shared pointer.
- **```subs_ = this->create_subscription<turtlesim::msg::Pose>(topic_name, 10, std::bind(TurtlePoseBroad::handle_turtle_pose, this, _1));```** Create the subscription for the given topic that in the code is created by the combination of the parameter and the /pose addition, and link the callback to create the transform based on the posed received.
- **```t_stamp.child_fram_id = turtlename_.c_str();```** For the stamped transform the child frame name, the string parameter is used to keep unique names.
- **```t_stamp.transform.translation.x = msg->x```** As the transforms is based on the x,y position of the turtle, therefore dynamic, for the traslational arguments we use the received x/y positions.
- **```q.setRPY(0,0, msg->theta);```** As we are in a plane, the angular movement comes from the Z axis, in the pose, it is the corresponding to the theta position, which is reinterpreted and converted to quaternion here.

After this, do not forget to edit the **CMakelist.txt** file:

```CMake

add_executable(turtle_broadcaster src/turtle_broadcaster.cpp)
ament_target_dependencies(
  turtle_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)
...

install(TARGETS
  turtle_broadcaster
  ...
  DESTINATION lib/${PROJECT_NAME}  
)

```

You can run this node (after building with colcon and sourcing) with:

```bash
ros2 run turtlesim turtlesim_node # Terminal 1
ros2 run m03_tf2_with_cpp turtle_broadcaster --ros-args --param turtlename:=turtle1 # Terminal 2
```

Again, at once you should only watch the turtlesim world, and nothing more, but if you use RVIZ to watch the TFs, you should be able to see it, you can run:

```bash
rviz2 -d m03_tf2_with_cpp/rviz/dynamic_tf_view.rviz
```

![turtle1_broadcast_dyn](/m03_robot_description/resources/rviz2_turtle1_broad_1.png)

And if you start playing with the turtle teleop, you should see in RVIZ the origins and tf moving along with the turtle.

![turtle1_broadcast_dyn2](/m03_robot_description/resources/rviz2_turtle1_broad_2.png)

Now, as we will have to run the broadcaster and specify the parameter for each turtle we create, we can use a *launch.py* file to make it easier, like this one:

```Python
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_node'
        ),

        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='original_broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])
```

### Using a TF listener:

A listener is needed when you want to follow up the movements of transforms and origins, in this case, we will use it to make sure a turtle follows another one by connecting the listener to the cmd_vel topic.

This example is located in the file [turtle_listener.cpp](/m03_robot_description/m03_tf2_with_cpp/src/turtle_listener.cpp), and as always, you can check the commented code, but do not forget some additional considerations presented below:

- **```#include "tf2/exceptions.h"```** : Used to consider proper exceptions that may happen if no relation between the transforms is found or other problems related with tf2.

- **```include "tf2_ros/transform_listener.h```** : To use a transform listener in C++

- **```include "turtlesim/srv/spawn.hpp```** We will need to spawn a turtle to follow another one, so we have to import the corresponding service.

- **```using namespace std::chrono_literals;```** To use user-defined suffixes related with time from the chrono library.

- **```target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");```** : As we will follow one turtle, we must know its name to select the correct transform, and we use it like a parameter to allow changes to follow different turtles with different nodes.

- **```rclcpp::Client<turtlesim::srv::Spawn>SharedPtr spawner_{nullptr};```** : Instance of a service client that uses the Spawn service from turtlesim.

- **```spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");```** : Definition of a client that will be used to spawn one single turtle which future mission would be to follow another one, it is defined by using a shared pointer.

- **```rclcpp::TimerBase::SharedPtr timer_{nullptr};```** : Instance of a timer by using a shared pointer.

- **```timer_ = this->create_wall_timer(1s, std::bind(&TurtleTimeTravelListener::on_timer, this))```** : Definition of a timer that will be used to make the turtle follow another turtle, by calling the on_timer member function to do that task.

- **```try {...} catch(const tf2::TransformException & ex) {...}```** : As we will handle look up in the transforms we should check for exception in the transform is not found or the calculation cannot be done, for that purpose, you can use the ```tf2::TransformException```.

- **```t_stamp = tf_buf_->lookupTransform(toFrameRel, fromFrameRel, now, 250ms);```** : The lookup will search for a transform that relates the to specified frames, in this case, in the present time with a timeout. We use the transform buffer to save the calculated transform if everything goes right.

- **```msg.angular.z = scaleRotationRate * atan2(t_stamp.transform.traslation.y, t_stamp.transform.traslation.x);```** : We will publish an angular movement to point directly to the position of the turtle we are following, for that we use the relation of x and y in arctangent squared.

- **```msg.linear.x = scaleForwardSpeed * sqrt(pow(t_stamp.transform.translation.x,2) + pow(t_stamp.transform.traslation.y,2));```** : For the linear movement we will consider the pythagorean theorem and launch the movement underscaled.

- **```auto result = spwaner->async_send_request(request, resp_rec_callbac)```** : If the turtle follower hasn't been created it will end up with this command that considers a ```request``` to spawn a turtle, and a lambda function ```resp_rec_callback``` to check if the response was recieved.

After this, do not forget to include the source file in the **CMakeLists.txt** and compile it: 

```CMake
add_executable(turtle_listener src/turtle_listener.cpp)
ament_target_dependencies(
    turtle_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)

install(TARGETS
  ...
  turtle_listener
  DESTINATION lib/${PROJECT_NAME}  
)
```

With that said, it is moment to proceed to the execution, for this case, we will use a launch file, as we need to broadcast the position (in tf terms) of two turtles, and listen to the first one we created the launch file called [tf2_demo.launch.py](/m03_robot_description/m03_tf2_with_cpp/launch/tf2_demo.launch.py), which content is:

```Python
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    """
    Script oriented to luanch the turtlesim node, and create the broadcaster
    and listener for turtle 1 and the "follower" turtle which is going to be 
    spawned during the launch execution.
    """
    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_node'
        ),

        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='original_broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),

        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'follower'}
            ]
        ),

        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),    
    ])
```

Make sure you have build the package, and you can run:

```bash
ros2 launch m03_tf2_with_cpp tf2_demo.launch.py
```

TODO: Add image of pending launch

### Adding a Frame:

### Using timeouts:

### Using time traivel:

## TF2 with Python:

Let's start creating a new package for Python:

```bash
    ros2 pkg create --build-type ament_python m03_tf_with_py
```

The dependencies you will need for this exercise are:

- **geometry_msgs:** For geometric interfaces, in this case pose and transforms.
- **python3-numpy:** For matrix usage and operations.
- **rclpy:** Do not forget about what we learnt in the previous module.
- **tf2_ros_py:** Tf2 package for Python.
- **turtlesim:** Our turtle friends will guide us.
- **launch:** For making multiple node runs and configure params/args.
- **launch_ros:** Package related with the .launch.py files.

Our focus will be guiding the journey of the **tf2** in Python, then, we will only explain certain parts of the code as we play with the nodes that interact with the **turtlesim**.

### Using a static tf broadcaster: 

As we mentioned before, transforms express the relation between different parts of a robot, when we use static transforms we are will describe the relation between the base of the robot and a fixed sensor on the robot, so we can interpretate the data of the sensor in terms of the robot position. 

For creating a transform in Python, first we need to import the dependencies that are **TransformStamped** from **geometry_msgs** and the **StaticTransformBroadcaster**. Some key commands are:

- **```<static_broad> = StaticTransformBroadcaster(<node>```)** : Intance a static transform broadcaster.
- **```<tf> = TransformStamped()```** : Intance a tranform stamped, which contains transform info in terms of traslation, rotation, but also the stamped info related with the parent, child frames and the time.
- **```<tf>.transform.traslation.x = <value>```** : Update the traslation x value in a transform.
- **```<tf>.transform.rotation.w = <value>```** : Udpate the rotation w value in a trnasform (remember, use quaternion components.)
- **```<tf>.header_stamp = <node>.get_colck().now().to_msg()```** : Add time stamp with the node interface.
- **```<tf>.header_frame_id = <parent_frame>```** : Link to frame (as a parent.)
- **```<tf>.child_frame_id = <child_frame>```** : Specify the name of the child frame.
- **```<static_broad>.sendTransform(<tf>)```** : Publish (broadcast) transform created.

In this case, we will use the code [static_broadcaster.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/static_broadcaster.py) to generate a static frame that create a pseudo-random transform (using random in Python), and check the corresponding transforms, it will need an argument with the name of the child frame, so do not forget to check the code and compile, you can run it with:

```bash
    ros2 run m03_tf2_with_py static_broad my_turtle
```

For checking if it is working, on another terminal you can run: 

```bash
    ros2 topic echo /tf_static
```

![static_py_broad](/m03_robot_description/resources/static_py_broad.png)

### Using a tf broadcaster:

Well, a static frame doesn't seems to be interesting, as it doesn't move, but what if we broadcast the transform of the turtle (respective to the world), then we can track its movements, and we can achieve that with a pretty similar focus as the static broadcaster.

In this case, we will use the code [turtle_broadcaster.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/turtle_broadcaster.py), where we are going to subscribe to the **Pose** of the **turtle1** from *turtlesim* (if you do not remember subscriptions, go and check the [Module 2](/m02_ros2_communication/m02_ros2_communication.md) info). Some key concepts of the code are mentioned here:

- **```<param> = <node>.declare_parameter(<name>)```** : Declare a ROS2 parameter that can be accessed for different nodes.
- 
- **```<param>.get_parameter_value()```** : Get the value of a ROS2 parameter available in the network or ambient.
- 
- **```<subscription> = <node>.create_subscription(<msg_type>, <topic>, <callback>)```** : Create a subscriber that receives msgs of type <msg_type> broadcasted in <topic> and links a <callback> for managing it back. In this case the message type is **Pose** from **turtlesim.msg**, and topic is **turtle1/pose** and you can check the callback on the code.

- **```<tf>.transformation.traslation.x = <msg>.x```** : Remember that you can use the messages received to update this value, in this case for updating linear position of the transform.

- Remember that orientations need to be provided as a quaternion, then as the turtlesim has rpy system, you will need to create a converter for making this process with your different tf.

Now, as what follows is a group demostration as we move on, we will start creating a launch for managing the nodes, in this first part we are going to use the *turtlesim* node and the recent node we created, do not forget to add the proper entrypoint in the [setup.py](/m03_robot_description/m03_tf2_with_py/setup.py):

```Python
    'turtle_broad = m03_tf2_with_py.turtle_broadcaster:main',
```

Our launch file will be [tf2_demo.launch.py](/m03_robot_description/m03_tf2_with_py/launch/tf2_demo.launch.py), and the first two nodes we mentioned were goint to be the next ones:

```Python
    from launch_ros.actions import Node
    from launch.substitutions import LaunchConfiguration

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            Node(
                package='m03_tf2_with_py',
                executable='turtle_broad',
                name='broadcaster1',
                parameters=[
                    {'turtlename': 'turtle1'}
                ]
            ),    
        ])
```


After you have save changes, and compile with *colcon* (and also source the directories), you can run the example with:

```bash
    ros2 launch m03_tf2_with_py tf2_demo.launch.py
```

Then you can use the turtle teleoperation node, and check the info of the transforms

```bash
    ros2 run turtlesim turtle_teleop_key  # Terminal 1
    ros2 topic echo /tf  # Terminal 2
```

You can obtain the results below:

![turtle_py_broad](/m03_robot_description/resources/turtle_py_broad.png)

### Using a tf listener:

If you remember from the previous module, when you have broadcaster (or a publisher), you will need someone listening (or subscribing) to that info to make usage of it in a node. That's what we are going to do here, as we will now listen to the transforms of the turtle.

The code for this section is [turtle_listener.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/turtle_listener.py), for this case, we will need to import **TransformListener** (from *tf2_ros.transform_listener*)instaed of TransformBroadcaster, and also, you will need a import related **Buffer** (from *tf2_ros.buffer*). Some key commands for this node are:

- **```<node>.<tf_buffer> = Buffer()```** : Instance a transfor buffer for storing tf info received.

- **```<node>.<tf_listener> = TrnasformListener(<tf_buffer>, <node>)```** : Instance a transform listener that will storage the info received in a <tf_buffer>.

- **```<tf> = <node>.<tf_buffer>.lookup_transform(<target_frame>, <source_frame>, <time>)```** : Receive the info of a transform in the buffer defined previosly, the info consideres a <target_frame> and a <source_frame> at given a <time>.

- **```TransformException```** : Exception that can be handled when using tfs, usually related with no relationship found of target and source frame.

For more orientations, check the comments present in the code and also the documentation. Once you end the node, remember to add the proper entrypoint at the **setup.py** file.

```Python
    'turtle_listen = m03_tf2_with_py.turtle_listener:main',
```

And let's modify the launch, at the previous definition of the launch in this .md file, you can add the next: 

```Python
    DeclareLaunchArgument(
        'target_frame', default_value='turtle1',
        description='Target frame name.'
    ),
    Node(
        package='m03_tf2_with_py'   ,
        executable='turtle_broad',
        name='broadcaster2',
        parameters=[
            {'turtlename': 'turtle2'}
        ]
    ),
    Node(
        package='m03_tf2_with_py',
        executable='turtle_listen',
        name='listener',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
        ]
    ),
```

After you have built it, you can run it with the command below, then you you can use turtle teleop to move the turtle, and you will see that a second turtle is chasing your orginal turtle.

```bash
    ros2 launch m03_tf2_with_py tf2_demo.launch.py
```

![static_py_list](/m03_robot_description/resources/turtle_py_listener.png)

### Adding a frame

Sometimes you will need addition frames to make possible some functions of the program, and they can be fixed or dynamic (as the broadcaster cases that were presented before). As you add more frames, will add complexity to the transformation tree, so you will need to consider proper implementations of your frames. If you remember, at the end of the module, we presented you a form to check the tranform tree with:

```bash
    ros2 run tf2_tools view_frames
```

If you do it, while running the last launch we made, you can discover that the transforms are:

```bash
    ros2 launch m03_tf2_with_py tf2_demo.launch.py
```

You can check the results of the tf2 tree in the file: [frames_tf2_demo.pdf](/m03_robot_description/frames_tf2_demo.pdf).

Technically, implementing a new frame is implementing a new broadcaster, and it can be static or dynamic, some key commands to keep in mind are:

- **```<node>.<tf_broadcaster> = TrnasformBroadcast(<node>)```** : You will need to implement again a broadcaster that you are going to use at some point to use the **sendTransform( **< tf >** )** function.

- **```<tf>.header.frame_id = <parent_name>```** : Here you should add a proper parent frame to indicate a valid relation in the tf tree.

- **```<tf>.child_frame_id = <child_name>```** : This is going to be your new frame with the name <child_name>

It is simple,  and the implementation was proposed on the code [lettuce_frame.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/lettuce_frame.py) as an analogy of having the turtle following some food attached to his body (think of it like having a pig following a carrot in Minecraft). Do not forget to add the entrypoint to the **setup.py** file:

```Python
    'lettuce_frame = m03_tf2_with_py.lettuce_frame:main',
```

And now, we will have another launch, where we are going to call our previous file, but also, add the static frame we just mentioned. The file is [lettuce_fix_fram.launch.py](/m03_robot_description/m03_tf2_with_py/launch/lettuce_fix_frame.launch.py) and the content is:

```Python
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    from launch_ros.actions import Node


    def generate_launch_description():
        demo_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('m03_tf2_with_py'), 'launch'),
                '/tf2_demo.launch.py']),
            )
        
        return LaunchDescription([
            demo_nodes,
            Node(
                package='m03_tf2_with_py',
                executable='lettuce_frame',
                name='fixed_broadcaster',
            ),
        ])

```

Do not forget to build and source, so you can run the next commands:

```bash
    ros2 launch m03_tf2_with_py lettuce_fix_frame.launch.py
    ros2 run tf2_tools view_frames
```

You can check the results of the tf2 tree in the file: [frames_fix_lettuce.pdf](/m03_robot_description/frames_fix_lettuce.pdf).


You can also add a dynamic frame, that for examples, move randomly, or follows a custom frame trajectory. The implementation is almost the same, but you will need some variable (like time) that add a dynamic change to the system, in our case, we will use the broadcast in the [lettuce_stick_frame](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/lettuce_stick_frame.py) code, that follows the analogy of a lettuce on a stick and a row, making random moves. Now, let's add the entrypoint, compile and run:

```Python
    'lettuce_stick_frame = m03_tf2_with_py.lettuce_stick_frame:main',
```

But, before running, let's create another launch called [lettuce_dyn_frame.launch.py](/m03_robot_description/m03_tf2_with_py/launch/lettuce_dyn_frame.launch.py) so the two cases (dynamic and static) are separete, the content is: 

```Python
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    from launch_ros.actions import Node


    def generate_launch_description():
        demo_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('m03_tf2_with_py'), 'launch'),
                '/tf2_demo.launch.py']),
            launch_arguments={'target_frame': 'lettuce'}.items(),
            )

        return LaunchDescription([
            demo_nodes,
            Node(
                package='m03_tf2_with_py',
                executable='lettuce_stick_frame',
                name='dynamic_broadcaster',
            ),
        ])
```

After this, we are ready to test it:

```bash
    ros2 launch m03_tf2_with_py lettuce_dyn_frame.launch
    ros2 run tf2_tools view_frames
```

You can check the results of the tf2 tree in the file: [frames_dyn_lettuce.pdf](/m03_robot_description/frames_dyn_lettuce.pdf).

## TF2 with C++:

TODO: Add cpp codes for static and dynamic broadcaster, listener and frame of tf2.
TODO: Add explanation of those codes.

# URDF: Unified Robot Description.

URDF is the tool that relates with transforms in order to describe our robot_model that can be used for visualization (status of motors, sensors info, position in the world, and more) and simulation (for those cases when you do not have a robot or want to test it with different environments).

## Using URDF for robot modeling:

It is an useful tool for visual representation of a robot, and for adding collisions, kinematic and dynamic descriptions. The *.urdf* must be located in the *urdf* directory and its files are composed of a series of tags based on XML, let's examine the most commonly used:

* **link:** It represents a specific part of a robot, for example, the upper arm. Inside this tag you can define the shape (cylinder, box, shpere, or mesh), size (according to the shape params), material (color and texture), collision, physcial properties or even use meshes to define the part.

```XML
        <link name="<name_link>">
            <visual>
                <!-- Visual shape / mesh -->
            <visual>
            <collision>
                <!-- Collision shape -->
            </collision>
            <inertial> ......... </inertial>
            <material ...... />
        </link>
```

ㅤㅤㅤㅤ![link_and_joint](/m03_robot_description/resources/joint_and_link.png)

* **visual/collision:** For representing the geometry of the visual and the collision of a link, it relates closely with the next commands, it is recommended to only use one figure or mesh per link, down below you can check the four possible figures you can use, but keep in mind the comment mentioned before, and the collision doesnt have to be the same as the visual, but it must be congruent with your robot model and task.

```XML
        <visual>
            <cylinder leng ="package://my_package/meshes/patr.dae"/>
        </visual>
        <collision>
            <cylinder length="0.6" radius="0.2"/>
            <box size="0.6 0.1 0.2"/>
            <sphere radius="0.2"/>
            <mesh filename="package://my_package/meshes/patr.dae"/>
        </collision>
```

* **joint:** An implementation of a connection, that can be fixed (static or attached to other part), continuous (like a wheel), prismatic (like a piston), floating, planar and revolute (like a non-continious servo). Inside this tags, you can define the parent/child relationship, physical limits, dynamic effects, origin (rpy for rotation or xyz for cartesian position), among others.

```XML
        <joint name="<joint_name>" type = ">joint_category>">
            <parent link = "parent_name" />
            <child link = "child_name" />

            <origin ....... />
            <!-- axis depending on the type --/>
            <!-- limit effort depending on the type -->

            <calibration ...... />
            <dynamics damping .... />
        </joint>
```

ㅤㅤㅤㅤ![related_links](/m03_robot_description/resources/parent_and_child.png)

* **origin:** For represeting the relative origin of the link and the joints, it has a linear part related with *x*, *y* and *z*, and also a rotational part related with *roll*, *pitch* and *yaw*. 

```XML
        <origin xyz="0 0 0" rpy="0 0 0"/>
```

* **robot:** Encapsulation of a group of joints and links that conform a robot, inside a robot tag you cannot implment multiple joints or links with the same name.

```XML
        <robot name = "<my_robot>">
            <link> .... </link>
            <link> .... </link>
            <joint> .... </joint>
        </robot>
```

* **gazebo:** For simulation in gazebo, you must specify additional params for plugins, materials and other.

```XML
        <gazebo reference="<link>">
            <material> Gazebo/White </material>
        </gazebo>
```

## Visualization of a URDF file:

For using URDF in a package, you should include *tf2*, *geometry_msgs*, *urdf*, *rviz* (for visualization), *xacro* (for macros that will be covered in the next section).

Maybe, in your ROS installation, the *xacro* and *urdf* package are missing, you can install them with:

```bash
    sudo apt-get install ros-humble-urdf*
    sudo apt-get install ros-humble-xacro
    sudo apt-get install liburdfdom-tools 
```

Also, when creating a package for a *robot_description*, you should add some new dirs:

* **/launch** For files that will be used to run multiple nodes, configure params like worlds and models, and taking advantage of rviz configurations.

* **/urdf** The source folder for XML descriptions and implementation of robots for visualization, simulation and configuration.

* **/meshes** Here is the place to add the 3D CAD models to use with the URDF descriptions.

Now, let's see some models, in this module you can find descriptions for an R2-D2 version for ROS, you can launch the configuration with the next command:

```bash
    ros2 launch m03_using_urdf display_urdf.launch.py
```

I encourage you to watch the code, explore the tags, modify and experiment for better understanding the usage of URDF, the file is [](/m03_robot_description/m03_using_urdf/urdf/r2d2_model.urdf). Also, you can explore them in a different way, that include checking the urdf and watching a graphic of them, this can be achieved using the next commands:

```bash
    check_urdf <your_urdf>.urdf
    urdf_to_graphiz <your_urdf>.urdf  #Generates a .pdf and a .gv files
    evince <your_urdf>.pdf
```

Now, lets explain the launch we are going to use for visualization, it must (at least contain) the following python structure:

1. The imports from **launch** in this case the **LaunchDescriptions** (as it is the base for any *launch.py* description), **actions.DeclareLaunchArguments** (to use arguments for nodes in the launch), **actions.IncludeLaunchDescription** (to include other launch files), **substitutions.LaunchConfiguration** (related to the configuration of an argument), **substituions.PathJoinSubstitution** (for adding file with a relative path to the same package or other packages) and **substitutions.FindPackageShare** (for including other packages paths), as show as follow:

```Python
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare
```

2. Now generate the function to creat the description, and instance the launch description.

```Python
    def generate_launch_description():
        ld = LaunchDescription()
```

3. We will start the lanch description, by considering the patch to the package, the model (*urdf*) and the rviz2 config.

```Python
        urdf_tutorial_path = FindPackageShare('m03_using_urdf')
        default_model_path = PathJoinSubstitution(['urdf', 'r2d2_model.urdf'])
        default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'robot_model.rviz'])
```

4. Then, we will declare the arguments, in this case, related with the gui and config file for rviz2, and link them with a proper action in the launch description.

```Python
        gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui')
        ld.add_action(gui_arg)
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file')
        ld.add_action(rviz_arg)
```

5. After that, we will use an argument to inlcude the urdf description by also considering the path defined previously

```bash
        ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Path to robot urdf file relative to m03_using_urd package'))
```

6. Finally, we include an action that links a launch present in other package, called **urdf_launch**, and we pass the arguments we defined through this steps:

```bash
        ld.add_action(IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'm03_using_urdf',
                'urdf_package_path': LaunchConfiguration('model'),
                'rviz_config': LaunchConfiguration('rvizconfig'),
                'jsp_gui': LaunchConfiguration('gui')}.items()
        ))

        return ld
```

You can check an explore the original launch, present in the **urdf_launch**'s launch directory, you can do it by using:

```bash
    ros2 pkg prefix urdf-launch
    cd <path_provided>
    code . # If you have VS Code, and chekc the launch
```

## Adding properties to the URDF Model

For obtaining a better description, you should include physical properties to the model, these includes collision, mass, inertia and others:

* **collision:** Where you define contacts and limits for the model, wheter it follows the visual or not.

```XML
        <collision>
            <geometry> ... </geometry>
            <origin ... />
        </collision>
```

* **inertial:** It includes params related to the movement and reaction to forces based on the inertia and the mass.

```XML
        <inertial>
            <mass value="<kg>">
            <!-- Matrix of inertia according the figure -->
             <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        <inertial/>
```

* **transmision:** Used to describe the relationship between actuator and joint, then it should include a `<joint>` and a `<actuator>` tags, for example:

```XML
        <transmission name="simple_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="foo_joint">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="foo_motor">
          <mechanicalReduction>50</mechanicalReduction>
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </actuator>
        </transmission>
```

* **limit:** Used for the boundaries of a joint, according to its type, it contains the effort, the lower and upper limit and the velocity.

```XML
        <limit effort="<max_effort>" lower="<min_value>" upper="<max_value>" velocity="<vel>" />
```

* **safety_controller:** Another joint option, that relates with k_position (relation between pos and vel limits), k_velocity (relation between effort and velocity limits), and soft_poser/upper_limits.

```XML
        <safety_controller k_position="<k_p>" k_velocity="<k_v>" soft_lower_limit="<lower_value>" soft_upper_limit="<upper_value>"/> 
```


## Xacro to improve your URDF descriptions:

Againt the lack of reusability, simplicity and programmability of URDF, you can use macros with Xacro to make it more user-friendly. Now, you will be able to use variables, constants, math, conditional statments, among others.

The root for working with xacro is:

```XML
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="<your_robot>"> 
```

### Using properties:

It is used for the declaration of constants, for creating one you use:

```XML
    <xacro:property name="<cte_name"> value="<value>" />
```

And for invoke them, you will need to use an expression, for example, `${<cte_name>}`.

### Including equations:

For making math relations between the constants and the model, or improving the definition of mobile parts, for these you can use math operators inside the expresion `${}`. Let's see some examples:

```XML
    <xacro:property name="<relation>" value="<cte_name>*3" />
    <sphere radius="${cte_name}-2" />
```

Also,you can include some functions and constants from the python math modele, like `radians(<degree>)`

### Using conditional:

Another useful tool, here you can compare properties or evaluate expresions, the basic usage relates with:

```XML
    <xacro:if value="<expression>">
        <!-- XML code or URDF description>
    </xacro:if>

    <xacro:if value="${expression}"/>
```

### Using macros:

Which is oriented to reduce the amount of code, to make it more simply and reusable, first you will need to define the macro, then you just need to invoke it with the correct params, for example.

```XML
    <xacro:macro name="<macro_name>" params="<param1> <param2> ...">
        <joint name="joint_${<param1>}>
            <!-- joint info--->
        </joint>
    </xacro:macro>

    <xacro:<macro_name> <param_1>="<my_param1>" ... />
```

Another interesting feature, is to include/import other xacro files, for this you can use:

```XML
    <xacro:include filename="$(find <package>)/path_to_urdf/<file>.xacro />
```

## From xacro to URDF:

When needed, you can convert your xacro file, into a URDF description, you just need to run the command:

```bash
    ros2 run xacro xacro -o r2d2.urdf r2d2_model.urdf.xacro 
```

Which can be used as a *robot_description* too, in fact, you can create another launch for running your model with xacro directly, instead fo converting it manually, the *launch.py* structure is presented in the file [display_xacro.launch.py](/m03_robot_description/m03_using_urdf/launch/display_xacro.launch.py), focus your attention on the usage of **Command**, for executing the xacro traslation:

```Python
    pkg_share = FindPackageShare(package='m03_using_urdf').find('m03_using_urdf')

    default_urdf_model_path = os.path.join(pkg_share, 'urdf/r2d2_model.urdf.xacro')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])
```

## Experimentation:

Now, you know the basics for modeling your robot with URDF, Xacro and tf. I encourage you to explore the URDF and Xacro files in the */urdf* directory. There you will find an R2D2 in URDF and Xacro, a robotic arm and even a car. Enjoy and play for yourself, you can view the models with the launches created for it, as:

```bash
    ros2 launch m03_using_urdf display_xacro.launch.py
```

ㅤㅤㅤㅤ![R2_D2_Xacro](/m03_robot_description/resources/r2d2_dan_version.png)

You can now try to create your own URDF, you can imagine a new robot or create one model of a robot you already have. But... what if the geometry of the robot is complex? The answer is simple, you can create the parts in CAD software like Solidworks, FreeCad, Blender, or others, and then add it to the mesh folder of your package, and import it as it was mentioned before, then start playing with the origins and transforms of your URDF.

But there is one additional option, you can use tools present in some CADs software to create your robot and export it in URDF. Most of these tools aren't available for ROS2, as they worked for ROS1, but you can obtain the *.urdf* file and the *mesh* dir, and then create your own launch and configure your package to consider the files present in the dir mentioned. Some options are:

- **[PTCE Creo to URDF:](https://github.com/icub-tech-iit/cad-libraries/wiki/Prepare-PTC-Creo-Mechanism-for-URDF):** Tool that works for Matlab <= R2017b and Creo, that allows URDF.
 
- **[SolidWorks URDF Exporter](http://wiki.ros.org/sw_urdf_exporter):** Exporter that works for solidworks assemblies and give you the option to set joints, origins and links in a easy way. Designed for Solidworks 2021, but still works for some recent versions. It exports directly with a ROS1 package, that can be easily converted to ROS2.

- **[Fusion2urdf](https://github.com/syuntoku14/fusion2urdf):** Unmantained for the last four years, but still works for Fusion 360 to export URDF of your models.

- **[OnShape to robot](https://onshape-to-robot.readthedocs.io/en/latest/)** : The more recent one, need full access to your OnShpae account by linking your API, it can generate a SDF or URDF file with the meshes.

- **[Phobos | Blender](https://github.com/dfki-ric/phobos):** The first free and open option to build your model that can be used in ROS, ROSCK, MARS or Gazebo, as it allows URDF, SDF and SMURF.

- **[CROSS | FreeCAD](https://github.com/galou/freecad.cross):** The most recent one, presented in ROSCon2023, and the second free and open option. It is still n development, but you can try it to generate your URDF.

My personal experience for creating URDFs is using Solidworks or FreeCad to create the meshes, then using the **solid2urdf** (Solidworks exporter to URDF), I create the ROS1 package that I will modify to make it work on ROS2 (only if I have checked that the URDF seems ok, the tfs and joints are correct and the meshes load correctly).


## Resources:
* [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
* [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
* [URDF/XML | ROS1 Guide](http://wiki.ros.org/urdf/XML)
* [Xacro | ROS1 Guide](http://wiki.ros.org/xacro)