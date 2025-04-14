# Motor DC RPM Publisher with µ-ROS

## Purpose

A demo on using the JGA25-371 DC Motor with encoder of two channel (A-B) to determinate the speed of the motor by considering the RAISING interruptions with A and the parallel case of B.

## Guide step by step

1. Make sure your PlatformIO installation is ready, do not forget to follow the [official instructions](https://github.com/micro-ROS/micro_ros_platformio) on Github. Also, do not forget to set up your **Micro-ROS** setup as present in this [micros-ros-tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

2. Open the project [01_hello_micro_ros](/m06_hardware_interaction/03_micro_ros_usage/07_motor_dc_rpm_pub/) by using PlatformIO VS Code extension.

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
    # Check for tht existence of /micro_ros_platformio_rpm_publisher
    ros2 topic echo /micro_ros_platformio_rpm_publisher
    ~~~

12. Now, proceed to move the motor manually or by connecting it to a battery, and watch the velocity (RPM) updates

## Additional resources

- [Tutorial on Encoders with Arduino (Hall Efect encoder with DC Motor) | Aleksandar Haber PHD](https://www.youtube.com/watch?v=1PJOzrXAlcg)

- [ESP32 with DC Motor and L298N Motor Driver – Control Speed and Direction | Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/)

- [talker_c | riot-ros2 @ Github](https://github.com/astralien3000/riot-ros2/blob/3d0779b920996f4e701830b8248573cd0e23204d/examples/talker_c/main.c#L32)
