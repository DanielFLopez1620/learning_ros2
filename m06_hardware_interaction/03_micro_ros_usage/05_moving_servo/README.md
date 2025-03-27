# Using servo motor MG996r with µ-ros

## Purpose

Simple demo for learning about topics, publisher and subscribers with **µ-micro** by using the servo MG996r.

For this case, we are using the ESP32 W-ROOM 30 pins, and the pinout is listed below:

- **Servo Voltage +** (Red) --> **Esp32 Vin**
- **Servo Signal** (Yellow/Orange) --> **Esp32 GPIO 13**
- **Servo Voltage -** (Black) --> **Esp32 Ground**

For a graphical guide consider the resources listed at the end of the file. 

**NOTE:** Servos and motors may need to drag additional current, and depending on the system it is better to implement and external source to power up the motors.

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

# Check for tht existence of /micro_ros_platformio_servo_control to move the servo
ros2 topic pub /micro_ros_platformio_servo_control std_msgs/msg/Float32 "data: 180.0" --once

# Check for the positon on /micro_ros_platformio_servo_feedback to review the process in another terminal
ros2 topic echo /micro_ros_platformio_servo_feedback
~~~



## Additional resources:

- [Example micro-ros_publisher | micro-ROS @ Github](https://github.com/micro-ROS/micro_ros_platformio/tree/main/examples/micro-ros_publisher)
- [ESP32 Servo Library | madhephaestus @ Github](https://github.com/madhephaestus/ESP32Servo)
- [ESP32 with MG996r | ESP32io](https://esp32io.com/tutorials/esp32-mg996r)