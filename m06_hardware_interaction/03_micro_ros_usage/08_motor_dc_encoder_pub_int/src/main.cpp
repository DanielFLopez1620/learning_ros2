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
#include <std_msgs/msg/int64.h>

// //////////////////////// GLOBAL DEFINITIONS ////////////////////////////////
// --------------------------- Definitions ------------------------------------
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for ESP32 ramework with serial transport.
#endif

// Ports of the program:
#define ENC_A_PIN 32  // Channel A of motor encoder
#define ENC_B_PIN 33  // Channel B of motor encoder

// ------------------------- ROS 2 related definitions ------------------------
// Define publisher
rcl_publisher_t publisher;

// Define message
std_msgs__msg__Int64 msg;

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

// Volatile counter for reading encoder
volatile long long int counter = 0;

// ///////////////////////////// FUNTION PROTOTYPES ///////////////////////////

void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void IRAM_ATTR handleEnc();

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
	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_encoder_node", "", 
		&support));

	// Create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
		"micro_ros_platformio_encoder_publisher"));

	// Create timer,
	const unsigned int timer_timeout = 100;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Set up pins
    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    // Link change of state of encoder A to a function
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEnc, CHANGE);

}

// /////////////////////////// LOOP IMPLEMENTATION ///////////////////////////
void loop() 
{
	// Delay required to avoid over-heating ESP32
	delay(100);

	// Spin
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

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
		char buff[64] = {0};
        noInterrupts();
		msg.data = counter;
        interrupts();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

/**
 * Handle encoder interrupt for a two-channel encoder. Based on the signal
 * launched by A consider two cases:
 * 
 * If A == B, the direction is clockwise
 * If A != B the direction is counterclockwise
 * 
 * The analysis was made based on oscilloscope visualization
 */
void IRAM_ATTR handleEnc() 
{
    if(digitalRead(ENC_B_PIN) != digitalRead(ENC_A_PIN))
    {
        counter--;
    }
    else
    {
        counter++;
    }
}