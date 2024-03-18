# Robot description:

Here we are going to explore the aspects needed to generate a robot description that can be used for visualization and simulation. We will cover topics related with **tf2**, **urdf**, **sdf**, and other related.

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

    sudo apt-get install ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim

Now, let's move to the practice part, we will begin with Python, this practice is heavily based on the code provided by the package **turtle-tf2-py**, you can check the original info by installing the next package (do not forget to source the package):

    sudo apt-get install ros2-humble-turtle-tf2-py

The practice with turtlesim will make on making frames and tfs with both Python and C++ on ROS2, the objective program will be to create a turtle chaser in different cases.

## TF2 with Python:

Let's start creating a new package for Python:

    ros2 pkg create --build-type ament_python m03_tf_with_py

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

    ros2 run m03_tf2_with_py static_broad my_turtle

For checking if it is working, on another terminal you can run: 

    ros2 topic echo /tf_static

![static_py_broad](/m03_robot_description/resources/static_py_broad.png)

## Using a tf broadcaster:

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

    'turtle_broad = m03_tf2_with_py.turtle_broadcaster:main',

Our launch file will be [tf2_demo.launch.py](/m03_robot_description/m03_tf2_with_py/launch/tf2_demo.launch.py), and the first two nodes we mentioned were goint to be the next ones:

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


After you have save changes, and compile with *colcon* (and also source the directories), you can run the example with:

    ros2 launch m03_tf2_with_py tf2_demo.launch.py

Then you can use the turtle teleoperation node, and check the info of the transforms

    ros2 run turtlesim turtle_teleop_key  # Terminal 1
    ros2 topic echo /tf  # Terminal 2

You can obtain the results below:

![turtle_py_broad](/m03_robot_description/resources/turtle_py_broad.png)

## Using a tf listener:

If you remember from the previous module, when you have broadcaster (or a publisher), you will need someone listening (or subscribing) to that info to make usage of it in a node. That's what we are going to do here, as we will now listen to the transforms of the turtle.

The code for this section is [turtle_listener.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/turtle_listener.py), for this case, we will need to import **TransformListener** (from *tf2_ros.transform_listener*)instaed of TransformBroadcaster, and also, you will need a import related **Buffer** (from *tf2_ros.buffer*). Some key commands for this node are:

- **```<node>.<tf_buffer> = Buffer()```** : Instance a transfor buffer for storing tf info received.

- **```<node>.<tf_listener> = TrnasformListener(<tf_buffer>, <node>)```** : Instance a transform listener that will storage the info received in a <tf_buffer>.

- **```<tf> = <node>.<tf_buffer>.lookup_transform(<target_frame>, <source_frame>, <time>)```** : Receive the info of a transform in the buffer defined previosly, the info consideres a <target_frame> and a <source_frame> at given a <time>.

- **```TransformException```** : Exception that can be handled when using tfs, usually related with no relationship found of target and source frame.

For more orientations, check the comments present in the code and also the documentation. Once you end the node, remember to add the proper entrypoint at the **setup.py** file.

    'turtle_listen = m03_tf2_with_py.turtle_listener:main',

And let's modify the launch, at the previous definition of the launch in this .md file, you can add the next: 

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

After you have built it, you can run it with the command below, then you you can use turtle teleop to move the turtle, and you will see that a second turtle is chasing your orginal turtle.

    ros2 launch m03_tf2_with_py tf2_demo.launch.py

![static_py_list](/m03_robot_description/resources/turtle_py_listener.png)

## Adding a frame

Sometimes you will need addition frames to make possible some functions of the program, and they can be fixed or dynamic (as the broadcaster cases that were presented before). As you add more frames, will add complexity to the transformation tree, so you will need to consider proper implementations of your frames. If you remember, at the end of the module, we presented you a form to check the tranform tree with:

    TODO: Command for transform tree

If you do it, while running the last launch we made, you can discover that the transforms are:

    TODO: Add image of tf2_demo.launch.py tf tree

Technically, implementing a new frame is implementing a new broadcaster, and it can be static or dynamic, some key commands to keep in mind are:

- **```<node>.<tf_broadcaster> = TrnasformBroadcast(<node>)```** : You will need to implement again a broadcaster that you are going to use at some point to use the **sendTransform( **< tf >** )** function.

- **```<tf>.header.frame_id = <parent_name>```** : Here you should add a proper parent frame to indicate a valid relation in the tf tree.

- **```<tf>.child_frame_id = <child_name>```** : This is going to be your new frame with the name <child_name>

It is simple,  and the implementation was proposed on the code [lettuce_frame.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/lettuce_frame.py) as an analogy of having the turtle following some food attached to his body (think of it like having a pig following a carrot in Minecraft). Do not forget to add the entrypoint to the **setup.py** file:

    'lettuce_frame = m03_tf2_with_py.lettuce_frame:main',

And now, we will have another launch, where we are going to call our previous file, but also, add the static frame we just mentioned. The file is [lettuce_fix_fram.launch.py](/m03_robot_description/m03_tf2_with_py/launch/lettuce_fix_frame.launch.py) and the content is:

    
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

Do not forget to build and source, so you can run the next commands:

    ros2 launch m03_tf2_with_py lettuce_fix_frame.launch.py
    TODO: Add tf_tree command

TODO: Add image turtle and lettuce fixed launch


You can also add a dynamic frame, that for examples, move randomly, or follows a custom frame trajectory. The implementation is almost the same, but you will need some variable (like time) that add a dynamic change to the system, in our case, we will use the broadcast in the [lettuce_stick_frame](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/lettuce_stick_frame.py) code, that follows the analogy of a lettuce on a stick and a row, making random moves. Now, let's add the entrypoint, compile and run:

    'lettuce_stick_frame = m03_tf2_with_py.lettuce_stick_frame:main',

But, before running, let's create another launch called [lettuce_dyn_frame.launch.py](/m03_robot_description/m03_tf2_with_py/launch/lettuce_dyn_frame.launch.py) so the two cases (dynamic and static) are separete, the content is: 


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

After this, we are ready to test it:

    ros2 launch m03_tf2_with_py lettuce_dyn_frame.launch
    TODO: Add tf tree command

TODO: Add image of the dynamic frame