// ////////////////////// DEPENDENCIES AND LIBRARIES //////////////////////////
// ---------------------- Required Arduino Libraries --------------------------
#include <Arduino.h>
#include <stdio.h>

// ---------------------- Platformio Libraries --------------------------------
#include <micro_ros_platformio.h>

// ---------------------- ROS Client Library for C Libs -----------------------
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ------------------------ Required messages ---------------------------------
#include <geometry_msgs/msg/twist.h>

// //////////////////////// GLOBAL DEFINITIONS ////////////////////////////////
// --------------------------- Definitions ------------------------------------
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for ESP32 ramework with serial transport.
#endif

// ------------------------- ROS 2 related definitions ------------------------

// Define subscriber
rcl_subscription_t subscriber;

// Define message
geometry_msgs__msg__Twist cmd_msg;

// Define executor
rclc_executor_t executor;

// Definte supporter
rclc_support_t support;

// Define memory allocator
rcl_allocator_t allocator;

// Define node
rcl_node_t node;

// Define timer
rcl_timer_t timer;

// Motor Left
int motor1Pin1 = 19; 
int motor1Pin2 = 18; 
int enable1Pin = 16; 

// Motor Right
int motor2Pin1 = 21;
int motor2Pin2 = 22;
int enable2Pin = 17;

// Motor params
int dutyCycle1 = 200;
int dutyCycle2 = 200;

// Define a ROS 2 Checker
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Define a soft ROS 2 Checker
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ///////////////////////////// FUNTION DEFINTIONS ///////////////////////////

void error_loop();
void velCallback(const void *msgin);

// //////////////////////////// SINGLE SET UP FUNCTION ///////////////////////
void setup() 
{
	// Configure Serial
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(2000);

	// Initialize allocator
	allocator = rcl_get_default_allocator();

	// Create init options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// Create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_sub_cmd_vel", "",
		&support));

	
	// Create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"micro_ros_platformio_cmd_vel"));

	RCCHECK(rclc_executor_init(
		&executor,
		&support.context, 
		1, 
		&allocator));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&subscriber,
		&cmd_msg,
		&velCallback,
		ON_NEW_DATA));

    // Sets motor pins as outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    // Set the PWM to consider in the enable pins
    analogWrite(enable1Pin, 120);
    analogWrite(enable2Pin, 120);
}

void loop() 
{
	// Delay required to avoid over-heating ESP32
	delay(100);

	// Spin
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// /////////////////////// FUNCTION DEFINITIONS /////////////////////////   

/**
 * Loop to handle errors
 */
void error_loop() 
{
	while(1) 
	{
		delay(100);
	}
}

/**
 * Function called when a new velocity command is received, then analize the
 * 2D components to generate the proper movement. It doesn't consider velicity
 * just directions.
 */
void velCallback(const void *msgin)
{
    if(cmd_msg.linear.x == 0)
    {
        enable1Pin = 100;
        enable2Pin = 100;
        if(cmd_msg.angular.z < 0)
        {
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor1Pin2, LOW); 
            digitalWrite(motor2Pin1, HIGH);
            digitalWrite(motor2Pin2, LOW); 
        }
        else if (cmd_msg.angular.z > 0)
        {
            digitalWrite(motor1Pin1, LOW);
            digitalWrite(motor1Pin2, HIGH); 
            digitalWrite(motor2Pin1, LOW);
            digitalWrite(motor2Pin2, HIGH);
        }
        else
        {
            digitalWrite(motor1Pin1, LOW);
            digitalWrite(motor1Pin2, LOW); 
            digitalWrite(motor2Pin1, LOW);
            digitalWrite(motor2Pin2, LOW); 
        }
    }
    else
    {
        enable1Pin = 120;
        enable2Pin = 120;
        if(cmd_msg.angular.z < 0)
        {
            enable2Pin -= 20;
        }
        else if (cmd_msg.angular.z > 0)
        {
            enable1Pin -= 20;
        }
        
        if(cmd_msg.linear.x > 0)
        {
            digitalWrite(motor1Pin1, LOW);
            digitalWrite(motor1Pin2, HIGH); 
            digitalWrite(motor2Pin1, HIGH);
            digitalWrite(motor2Pin2, LOW); 
        }
        else
        {
            digitalWrite(motor1Pin1, HIGH);
            digitalWrite(motor1Pin2, LOW); 
            digitalWrite(motor2Pin1, LOW);
            digitalWrite(motor2Pin2, HIGH); 
        }
    }
}