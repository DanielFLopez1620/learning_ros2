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

    ros2 run tf2_tools view_frames

If you do it, while running the last launch we made, you can discover that the transforms are:

    ros2 launch m03_tf2_with_py tf2_demo.launch.py

You can check the results of the tf2 tree in the file: [frames_tf2_demo.pdf](/m03_robot_description/frames_tf2_demo.pdf).

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
    ros2 run tf2_tools view_frames

You can check the results of the tf2 tree in the file: [frames_fix_lettuce.pdf](/m03_robot_description/frames_fix_lettuce.pdf).


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
    ros2 run tf2_tools view_frames

You can check the results of the tf2 tree in the file: [frames_dyn_lettuce.pdf](/m03_robot_description/frames_dyn_lettuce.pdf).

# URDF: Unified Robot Description.

URDF is the tool that relates with transforms in order to describe our robot_model that can be used for visualization (status of motors, sensors info, position in the world, and more) and simulation (for those cases when you do not have a robot or want to test it with different environments).

## Using URDF for robot modeling:

It is an useful tool for visual representation of a robot, and for adding collisions, kinematic and dynamic descriptions. The *.urdf* must be located in the *urdf* directory and its files are composed of a series of tags based on XML, let's examine the most commonly used:

* **link:** It represents a specific part of a robot, for example, the upper arm. Inside this tag you can define the shape (cylinder, box, shpere, or mesh), size (according to the shape params), material (color and texture), collision, physcial properties or even use meshes to define the part.

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

ㅤㅤㅤㅤ![link_and_joint](/m03_robot_description/resources/joint_and_link.png)

* **visual/collision:** For representing the geometry of the visual and the collision of a link, it relates closely with the next commands, it is recommended to only use one figure or mesh per link, down below you can check the four possible figures you can use, but keep in mind the comment mentioned before, and the collision doesnt have to be the same as the visual, but it must be congruent with your robot model and task.

        <visual>
            <cylinder length="0.6" radius="0.2"/>
            <box size="0.6 0.1 0.2"/>
            <sphere radius="0.2"/>
            <mesh filename="package://my_package/meshes/patr.dae"/>
        </visual>
        <collision>
            <cylinder length="0.6" radius="0.2"/>
            <box size="0.6 0.1 0.2"/>
            <sphere radius="0.2"/>
            <mesh filename="package://my_package/meshes/patr.dae"/>
        </collision>

* **joint:** An implementation of a connection, that can be fixed (static or attached to other part), continuous (like a wheel), prismatic (like a piston), floating, planar and revolute (like a non-continious servo). Inside this tags, you can define the parent/child relationship, physical limits, dynamic effects, origin (rpy for rotation or xyz for cartesian position), among others.

        <joint name="<joint_name>" type = ">joint_category>">
            <parent link = "parent_name" />
            <child link = "child_name" />

            <origin ....... />
            <!-- axis depending on the type --/>
            <!-- limit effort depending on the type -->

            <calibration ...... />
            <dynamics damping .... />
        </joint>

ㅤㅤㅤㅤ![related_links](/m03_robot_description/resources/parent_and_child.png)

* **origin:** For represeting the relative origin of the link and the joints, it has a linear part related with *x*, *y* and *z*, and also a rotational part related with *roll*, *pitch* and *yaw*. 

        <origin xyz="0 0 0" rpy="0 0 0"/>

* **robot:** Encapsulation of a group of joints and links that conform a robot, inside a robot tag you cannot implment multiple joints or links with the same name.

        <robot name = "<my_robot>">
            <link> .... </link>
            <link> .... </link>
            <joint> .... </joint>
        </robot>

* **gazebo:** For simulation in gazebo, you must specify additional params for plugins, materials and other.

        <gazebo reference="<link>">
            <material> Gazebo/White </material>
        </gazebo>

## Visualization of a URDF file:

For using URDF in a package, you should include *tf2*, *geometry_msgs*, *urdf*, *rviz* (for visualization), *xacro* (for macros that will be covered in the next section).

Maybe, in your ROS installation, the *xacro* and *urdf* package are missing, you can install them with:

    sudo apt-get install ros-humble-urdf*
    sudo apt-get install ros-humble-xacro
    sudo apt-get install liburdfdom-tools 

Also, when creating a package for a *robot_description*, you should add some new dirs:

* **/launch** For files that will be used to run multiple nodes, configure params like worlds and models, and taking advantage of rviz configurations.

* **/urdf** The source folder for XML descriptions and implementation of robots for visualization, simulation and configuration.

* **/meshes** Here is the place to add the 3D CAD models to use with the URDF descriptions.

Now, let's see some models, in this module you can find descriptions for an R2-D2 version for ROS, you can launch the configuration with the next command:

    ros2 launch m03_using_urdf display_urdf.launch.py

I encourage you to watch the code, explore the tags, modify and experiment for better understanding the usage of URDF, the file is [](/m03_robot_description/m03_using_urdf/urdf/r2d2_model.urdf). Also, you can explore them in a different way, that include checking the urdf and watching a graphic of them, this can be achieved using the next commands:

    check_urdf <your_urdf>.urdf
    urdf_to_graphiz <your_urdf>.urdf  #Generates a .pdf and a .gv files
    evince <your_urdf>.pdf

Now, lets explain the launch we are going to use for visualization, it must (at least contain) the following python structure:

1. The imports from **launch** in this case the **LaunchDescriptions** (as it is the base for any *launch.py* description), **actions.DeclareLaunchArguments** (to use arguments for nodes in the launch), **actions.IncludeLaunchDescription** (to include other launch files), **substitutions.LaunchConfiguration** (related to the configuration of an argument), **substituions.PathJoinSubstitution** (for adding file with a relative path to the same package or other packages) and **substitutions.FindPackageShare** (for including other packages paths), as show as follow:

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

2. Now generate the function to creat the description, and instance the launch description.

    def generate_launch_description():
        ld = LaunchDescription()

3. We will start the lanch description, by considering the patch to the package, the model (*urdf*) and the rviz2 config.

        urdf_tutorial_path = FindPackageShare('m03_using_urdf')
        default_model_path = PathJoinSubstitution(['urdf', 'r2d2_model.urdf'])
        default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'robot_model.rviz'])

4. Then, we will declare the arguments, in this case, related with the gui and config file for rviz2, and link them with a proper action in the launch description.

        gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui')
        ld.add_action(gui_arg)
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file')
        ld.add_action(rviz_arg)

5. After that, we will use an argument to inlcude the urdf description by also considering the path defined previously

        ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Path to robot urdf file relative to m03_using_urd package'))

6. Finally, we include an action that links a launch present in other package, called **urdf_launch**, and we pass the arguments we defined through this steps:

        ld.add_action(IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'm03_using_urdf',
                'urdf_package_path': LaunchConfiguration('model'),
                'rviz_config': LaunchConfiguration('rvizconfig'),
                'jsp_gui': LaunchConfiguration('gui')}.items()
        ))

        return ld

You can check an explore the original launch, present in the **urdf_launch**'s launch directory, you can do it by using:

    ros2 pkg prefix urdf-launch
    cd <path_provided>
    code . # If you have VS Code, and chekc the launch

## Adding properties to the URDF Model

For obtaining a better description, you should include physical properties to the model, these includes collision, mass, inertia and others:

* **collision:** Where you define contacts and limits for the model, wheter it follows the visual or not.

        <collision>
            <geometry> ... </geometry>
            <origin ... />
        </collision>

* **inertial:** It includes params related to the movement and reaction to forces based on the inertia and the mass.

        <inertial>
            <mass value="<kg>">
            <!-- Matrix of inertia according the figure -->
             <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        <inertial/>

* **transmision:** Used to describe the relationship between actuator and joint, then it should include a `<joint>` and a `<actuator>` tags, for example:

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

* **limit:** Used for the boundaries of a joint, according to its type, it contains the effort, the lower and upper limit and the velocity.

        <limit effort="<max_effort>" lower="<min_value>" upper="<max_value>" velocity="<vel>" />
    
* **safety_controller:** Another joint option, that relates with k_position (relation between pos and vel limits), k_velocity (relation between effort and velocity limits), and soft_poser/upper_limits.

        <safety_controller k_position="<k_p>" k_velocity="<k_v>" soft_lower_limit="<lower_value>" soft_upper_limit="<upper_value>"/> 



## Xacro to improve your URDF descriptions:

Againt the lack of reusability, simplicity and programmability of URDF, you can use macros with Xacro to make it more user-friendly. Now, you will be able to use variables, constants, math, conditional statments, among others.

The root for working with xacro is:

    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="<your_robot>"> 

### Using properties:

It is used for the declaration of constants, for creating one you use:

    <xacro:property name="<cte_name"> value="<value>" />

And for invoke them, you will need to use an expression, for example, `${<cte_name>}`.

### Including equations:

For making math relations between the constants and the model, or improving the definition of mobile parts, for these you can use math operators inside the expresion `${}`. Let's see some examples:

    <xacro:property name="<relation>" value="<cte_name>*3" />

    <sphere radius="${cte_name}-2" />

Also,you can include some functions and constants from the python math modele, like `radians(<degree>)`

### Using conditional:

Another useful tool, here you can compare properties or evaluate expresions, the basic usage relates with:

    <xacro:if value="<expression>">
        <!-- XML code or URDF description>
    </xacro:if>

    <xacro:if value="${expression}"/>

### Using macros:

Which is oriented to reduce the amount of code, to make it more simply and reusable, first you will need to define the macro, then you just need to invoke it with the correct params, for example.

    <xacro:macro name="<macro_name>" params="<param1> <param2> ...">
        <joint name="joint_${<param1>}>
            <!-- joint info--->
        </joint>
    </xacro:macro>

    <xacro:<macro_name> <param_1>="<my_param1>" ... />

Another interesting feature, is to include/import other xacro files, for this you can use:

    <xacro:include filename="$(find <package>)/path_to_urdf/<file>.xacro />

## From xacro to URDF:

When needed, you can convert your xacro file, into a URDF description, you just need to run the command:

    ros2 run xacro xacro -o r2d2.urdf r2d2_model.urdf.xacro 

Which can be used as a *robot_description* too, in fact, you can create another launch for running your model with xacro directly, instead fo converting it manually, the *launch.py* structure is presented in the file [display_xacro.launch.py](/m03_robot_description/m03_using_urdf/launch/display_xacro.launch.py), focus your attention on the usage of **Command**, for executing the xacro traslation:

    pkg_share = FindPackageShare(package='m03_using_urdf').find('m03_using_urdf')

    default_urdf_model_path = os.path.join(pkg_share, 'urdf/r2d2_model.urdf.xacro')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])

## Experimentation:

Now, you know the basics for modeling your robot with URDF, Xacro and tf. I encourage you to explore the URDF and Xacro files in the */urdf* directory. There you will find an R2D2 in URDF and Xacro, a robotic arm and even a car. Enjoy and play for yourself, you can view the models with the launches created for it, as:

    ros2 launch m03_using_urdf display_xacro.launch.py

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