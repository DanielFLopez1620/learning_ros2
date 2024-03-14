# Robot description:

Here we are going to explore the aspects needed to generate a robot description that can be used for visualization and simulation. We will cover topics related with **tf2**, **urdf**, **sdf**, and other related.

# TF2

A transform is the basics of understating the system of a robot, it gives you the option to calculate the relations of the different parts of your robot. It can be a mobile robot, like turtlebot3, or a industrial robot, like Universal Robots Manipulators. 

You can understand a transform like an origin with its own axis and rotations. Let's illustrate this, you have two mobile robots exploring a room, we will call them 'mob1' and 'mob2', during the exploration 'mob1' found something interesting and want to tell 'mob2' to come, but... how does he tell the position of the objective?

TODO: Add image of two mobile robots exploring.

You can say the position relateve a common origin, but it can get messy if the position of 'mob1' is too complex. Another option is to make the origin at 'mob1', but it get difficult to pass the position and consider 'mob2'.

TODO: Add image of origin and robot origin

The final option is related with using transforms... what if we create two origins and consider the transform, so we keep track of the info since the origin to the robot (and even further).

TODO: Add image of tf consideration

A transform is a consideration of the steps needed to go from one origin to another (frame to frame) that consider linear and angular movements, the linear components are expressed by the x, y and z axis, while the angular components are considered as [quaternions]().

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

### Using a static broadcaster: 

As we mentioned before, transforms express the relation between different parts of a robot, when we use static transforms we are will describe the relation between the base of the robot and a fixed sensor on the robot, so we can interpretate the data of the sensor in terms of the robot position. 

For creating a transform in Python, first we need to import the dependencies that are **TransformStamped** from **geometry_msgs** and the **StaticTransformBroadcaster**. Some key commands are:

- **<static_broad> = StaticTransformBroadcaster(<node>)** : Intance a static transform broadcaster.
- **<tf> = TransformStamped()** : Intance a tranform stamped, which contains transform info in terms of traslation, rotation, but also the stamped info related with the parent, child frames and the time.
- **<tf>.transform.traslation.x = <value>** : Update the traslation x value in a transform.
- **<tf>.transform.rotation.w = <value>** : Udpate the rotation w value in a trnasform (remember, use quaternion components.)
- **<tf>.header_stamp = <node>.get_colck().now().to_msg()** : Add time stamp with the node interface.
- **<tf>.header_frame_id = <parent_frame>** : Link to frame (as a parent.)
- **<tf>.child_frame_id = <child_frame>** : Specify the name of the child frame.
- **<static_broad>.sendTransform(<tf>)** : Publish (broadcast) transform created.

In this case, we will use the code [static_broadcaster.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/static_broadcaster.py) to generate a static frame that create a pseudo-random transform (using random in Python), and check the corresponding transforms, it will need an argument with the name of the child frame, so do not forget to check the code and compile, you can run it with:

    ros2 run m03_tf2_with_py static_broad my_turtle

For checking if it is working, on another terminal you can run: 

    ros2 topic echo /tf_static

TODO: Add image of the transform node running.

## Using a broadcaster:

Well, a static frame doesn't seems to be interesting, as it doesn't move, but what if we broadcast the transform of the turtle (respective to the world), then we can track its movements, and we can achieve that with a pretty similar focus as the static broadcaster.

In this case, we will use the code [turtle_broadcaster.py](/m03_robot_description/m03_tf2_with_py/m03_tf2_with_py/turtle_broadcaster.py), where we are going to subscribe to the **Pose** of the **turtle1** from *turtlesim* (if you do not remember subscriptions, go and check the [Module 2](/m02_ros2_communication/m02_ros2_communication.md) info). Some key concepts of the code are mentioned here:

- **<param> = <node>.declare_parameter(<name>)** : Declare a ROS2 parameter that can be accessed for different nodes.
- 
- **<param>.get_parameter_value()** : Get the value of a ROS2 parameter available in the network or ambient.
- 
- **<subscription> = <node>.create_subscription(<msg_type>, <topic>, <callback>)** : Create a subscriber that receives msgs of type <msg_type> broadcasted in <topic> and links a <callback> for managing it back. In this case the message type is **Pose** from **turtlesim.msg**, and topic is **turtle1/pose** and you can check the callback on the code.

- **<tf>.transformation.traslation.x = <msg>.x** : Remember that you can use the messages received to update this value, in this case for updating linear position of the transform.

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

