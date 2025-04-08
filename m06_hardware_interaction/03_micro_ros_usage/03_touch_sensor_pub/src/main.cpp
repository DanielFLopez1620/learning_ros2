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
#include <std_msgs/msg/bool.h>

// //////////////////////// GLOBAL DEFINITIONS ////////////////////////////////
// --------------------------- Definitions ------------------------------------
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for ESP32 framework with serial transport.
#endif

#define TOUCH_PIN 33

// ------------------------- ROS 2 related definitions ------------------------
// Define publisher
rcl_publisher_t publisher;

// Define message
std_msgs__msg__Bool msg;

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

// Global counter
int counter = 0;

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

/**
 * Function that will be linked to the timer in order to publish
 * the message data.
 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) 
	{
		// Check state button when timer is called and publish state
        msg.data = digitalRead(TOUCH_PIN); 
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

// ///////////////////// SINGLE SET UP FUNCTION ///////////////////////////////
void setup() 
{
	// Configure serial transport
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(2000);

	// Initialize allocator
	allocator = rcl_get_default_allocator();

	// Create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// Create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_touch_node", "", 
		&support));

	// Create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"micro_ros_platformio_touch_pub"));

	// Create timer,
	const unsigned int timer_timeout = 250;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Configure pins
    pinMode(TOUCH_PIN, INPUT);
}

// /////////////////////////// LOOP IMPLEMENTATION ///////////////////////////
void loop() 
{
	// Delay required to avoid over-heating ESP32
	delay(100);

	// Spin
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}