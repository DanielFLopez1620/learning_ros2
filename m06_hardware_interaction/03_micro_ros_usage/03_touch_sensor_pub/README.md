# Usage of touch sensor with µ-ROS

## Purpose

Simple demo for learning about **µ-ROS** the usage of a digital capacitive sensor with a ESP32.

## Guide step by step

1. Make sure your PlatformIO installation is ready, do not forget to follow the [official instructions](https://github.com/micro-ROS/micro_ros_platformio) on Github. Also, do not forget to set up your **Micro-ROS** setup as present in this [micros-ros-tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

2. Open the project [01_hello_micro_ros](/m06_hardware_interaction/03_micro_ros_usage/01_hello_micro_ros/) by using PlatformIO VS Code extension.

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
    # Check for tht existence of /micro_ros_platformio_touch_pub
    ros2 topic echo /micro_ros_platformio_touch_pub
    ~~~

12. Now interact with the sensor, and check the subscription (echo). It will display **true** when you touch the button.

## Additional resources

- [micro-ros_publisher example | micro-ROS @ Github](https://github.com/micro-ROS/micro_ros_platformio/tree/main/examples/micro-ros_publisher)

- [Touch Sensor with ESP32 | ESP32IO](https://esp32io.com/tutorials/esp32-touch-sensor-led)
