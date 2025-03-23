// ////////////////////// DEPENDENCIES AND LIBRARIES //////////////////////////
// ---------------------- Required Arduino Libraries --------------------------
#include <Arduino.h>

// ---------------------- Platformio Libraries --------------------------------
#include <micro_ros_platformio.h>

// ---------------------- ROS Client Library for C Libs -----------------------
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ------------------------ Required messages ---------------------------------
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// //////////////////////// GLOBAL DEFINITIONS ////////////////////////////////
// --------------------------- Definitions ------------------------------------
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for ESP32 ramework with serial transport.
#endif

// ------------------------- ROS 2 related definitions ------------------------
// Define publisher
rcl_publisher_t publisher;

// Define subscriber
rcl_subscription_t subscriber;

// Define message
std_msgs__msg__Int32 num_msg;
std_msgs__msg__Float32 dec_msg;

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

// Define a ROS 2 Checker
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Define a soft ROS 2 Checker
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ///////////////////////////// FUNTION DEFINTIONS ///////////////////////////
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
void subCallback(const void *msgin)
{
    dec_msg.data = (float) num_msg.data / 2;
	RCSOFTCHECK(rcl_publish(&publisher, &dec_msg, NULL));
}
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
	RCCHECK(rclc_node_init_default(&node, "micro_ros_pub_sub_std_node", "",
		&support));

	// Create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"micro_ros_platformio_node_float_pub"));
	
	// Create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"micro_ros_platformio_node_int_sub"));

	RCCHECK(rclc_executor_init(
		&executor,
		&support.context, 
		1, 
		&allocator));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&subscriber,
		&num_msg,
		&subCallback,
		ON_NEW_DATA));
}

void loop() 
{
	// Delay required to avoid over-heating ESP32
	delay(100);

	// Spin
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
