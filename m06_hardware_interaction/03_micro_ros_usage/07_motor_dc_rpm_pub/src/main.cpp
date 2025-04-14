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
#include <std_msgs/msg/float32.h>

// //////////////////////// GLOBAL DEFINITIONS ////////////////////////////////
// --------------------------- Definitions ------------------------------------
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for ESP32 ramework with serial transport.
#endif

// Ports of the program:
#define ENC_A_PIN 32 // Channel A of motor encoder 
#define ENC_B_PIN 33 // Channel B of motor encoder 

// ------------------------- ROS 2 related definitions ------------------------
// Define publisher
rcl_publisher_t publisher;

// Define message
std_msgs__msg__Float32 msg;

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

// Volatile counter for encoder readings
volatile long pulse_count = 0;

// Lectures per revolution, varies depending on the motor.
// In my case with a 1000 RPM JGA25-371 DC Motor is 204 aprox.
const int pulses_per_rev = 204;

// Previous time required for vel calculation
unsigned long last_time = 0;

// Previous count of encoder for vel calculation
long last_pulse_count = 0;

// ///////////////////////////// FUNTION PROTOTYPES ///////////////////////////

void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void IRAM_ATTR handleEncoder();

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
	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_rpm_node", "", 
		&support));

	// Create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"micro_ros_platformio_rpm_publisher"));

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

	// Set up encoder pins
	pinMode(ENC_A_PIN, INPUT);
	pinMode(ENC_B_PIN, INPUT);

	// Attach interrupt on rising edge of channel A
	attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoder, RISING);

	// Set up time
	last_time = millis();
}

// /////////////////////////// LOOP IMPLEMENTATION ///////////////////////////
void loop() 
{
	// Delay required to avoid over-heating ESP32
	delay(100);

	// Spin
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}


// ///////////////////////// FUNCTION DEFINITIONS ////////////////////////////

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
	// Consider current time and calculate diff
	unsigned long now = millis();
	unsigned long elapsed = now - last_time;

	// Each 100 ms
	if (elapsed >= 100) 
	{
		// Block interrupts to update the pulse count
		noInterrupts();
		long pulses = pulse_count;
		interrupts();

		// Calculate the difference in time
		long delta = pulses - last_pulse_count;

		// Calculate RPM: (delta / pulses_per_rev) / (elapsed / 60000)
		msg.data = (delta * 600.0) / pulses_per_rev;

		// Update data before next iteration
		last_pulse_count = pulses;
		last_time = now;

		// Publish rpms
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

/**
 * According to the state of B when the encoder A is rising, it will consider
 * the increment or decrement of the encouder count.
 * 
 * If A == B, it is counterclockwise
 * If A != B, it is clockwise
 * 
 */
void IRAM_ATTR handleEncoder() 
{
	if (digitalRead(ENC_B_PIN)) 
	{
		pulse_count--;
	} 
	else 
	{
		pulse_count++;
	}
}