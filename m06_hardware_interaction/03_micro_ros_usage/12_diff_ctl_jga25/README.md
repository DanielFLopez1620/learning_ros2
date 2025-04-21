# Differential controller base with µ-ROS

## Purpose

A demo on using two JGA25-371 DC Motors in order to subscribe to a twist topic in order to move. It considers the usage of the L298N driver and the lecture of the two encoder channel of both motors.

This is aimed to mount a code capable for future integrations with ros2_control.

NOTE: The motors are connected in a different way, as OUT1 (+) and OUT2 (-) are for the right motor, and OUT3 (+) and OUT4(-) are for the left motor. Check the **hardware** file on the **lib** directory for more about connections.

A brief note on them is shown below:

- **Left:**
  - **Encoder A:** GPIO32
  - **Encoder B:** GPIO33
  - **Motor Forward (IN3):** GPIO22
  - **Motor Backward (IN4):** GPIO21
  - **Motor Enable (ENB):** GPIO17
- **Rght:**
  - **Encoder A:** GPIO34
  - **Encoder B:** GPIO35
  - **Motor Forward (IN3):** GPIO19
  - **Motor Backward (IN4):** GPIO18
  - **Motor Enable (ENB):** GPIO16


## Guide step by step

1. Make sure your PlatformIO installation is ready, do not forget to follow the [official instructions](https://github.com/micro-ROS/micro_ros_platformio) on Github. Also, do not forget to set up your **Micro-ROS** setup as present in this [micros-ros-tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

2. Open the project [01_hello_micro_ros](/m06_hardware_interaction/03_micro_ros_usage/12_diff_ctl_jga25/) by using PlatformIO VS Code extension.

3. Connect the ESP32 to your computer and give the proper permissions, for example:

    ~~~bash
    sudo chmod 666 /dev/ttyUSB0 # Check which interface you are using as in your case it may not be USB0
    ~~~

4. Upload the code, do not forget to press the **Boot** button and check that the process is done correctly.

5. Go to your **µ-ros** workspace.

    ~~~bash
    cd ~/microros_ws
    ~~~

6. Run the local set up, do not forget to follow the steps from the guide for FreeRTOS systems mentioned in step 1 to access the next steps.

    ~~~bash
    source instal/local_setup.bash
    ~~~

7. Then, create the µ-ros agent

    ~~~bash
    ros2 run micro_ros_setup create_agent-ws.sh
    ~~~

8. After that, build the agent. You may see some warnings.

    ~~~bash
    ros2 run micro_ros_setup build_agent.sh
    ~~~

9. Finally, run the agent.

    ~~~bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 # Change the dev according your set up
    ~~~

10. If the microcontroller do not seem to connect, reboot it or disconnect/reconnect it.

11. Check the communication:

    ~~~bash
    ros2 topic list
    # Check for tht existence of /diff_ctl_left_enc and /diff_ctl_right_enc
    ~~~

12. On separated terminals, subscribe with **ros2 topic echo**:

    ~~~bash
    # Terminal 1
    ros2 topic echo /diff_ctl_left_enc

    # Terminal 2
    ros2 topic echo /diff_ctl_right_enc
    ~~~

13. Start moving the motors manually and check the increment/decrement of both, based on the front (positive) and back (negative) directions. If they
are inverted check connections or review ports on code.

14. Then, proceed to r publish to the controller, by using a muti array topic as shown below:

    ~~~bash
    ros2 topic pub /diff_ctl_motor_cmd std_msgs/msg/Int64MultiArray "layout:
    dim: []
    data_offset: 0
    data: [0,0]" --once
    ~~~

15. Send different values and test moving one or two motors, it accept any integer value. However, for considering a case of full range, it should be [-7500,  7500].

## Additional resources

- [ESP32 with DC Motor and L298N Motor Driver – Control Speed and Direction | Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/)

- [talker_c | riot-ros2 @ Github](https://github.com/astralien3000/riot-ros2/blob/3d0779b920996f4e701830b8248573cd0e23204d/examples/talker_c/main.c#L32)
