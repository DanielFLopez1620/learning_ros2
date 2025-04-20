#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/int64.h>

#include "constants.hpp"
#include "encoder.hpp"
#include "hardware.hpp"
#include "motor.hpp"
#include "pid.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void IRAM_ATTR read_left_enc();
void IRAM_ATTR read_right_enc();
void adjust_motors_speed();
void set_motor_speed(int left_speed, int right_speed);
void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_tm);
void cmd_motor_callback(const void *msgin);


rcl_publisher_t enc_left_pub;
rcl_publisher_t enc_right_pub;
rcl_subscription_t motor_speed_sub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Int64 enc_left_value;
std_msgs__msg__Int64 enc_right_value;

std_msgs__msg__Int64MultiArray cmd_msg_speed;



diff::MotorDriver motor_left(diff::HARDWARE::ML_EN,
    diff::HARDWARE::ML_FORW, diff::HARDWARE::ML_BACW);

diff::MotorDriver motor_right(diff::HARDWARE::MR_EN,
    diff::HARDWARE::MR_FORW, diff::HARDWARE::MR_BACW);

diff::EncoderDriver enc_left(diff::HARDWARE::ML_ENCA, diff::HARDWARE::ML_ENCB);
diff::EncoderDriver enc_right(diff::HARDWARE::MR_ENCA, diff::HARDWARE::MR_ENCB);

diff::ControlPID pid_left(diff::ROBOT_CONST::PID_KP, diff::ROBOT_CONST::PID_KD,
    diff::ROBOT_CONST::PID_KI, diff::ROBOT_CONST::PID_KO, 
    diff::ROBOT_CONST::PWM_MAX, diff::ROBOT_CONST::PWM_MIN);

diff::ControlPID pid_right(diff::ROBOT_CONST::PID_KP, diff::ROBOT_CONST::PID_KD,
    diff::ROBOT_CONST::PID_KI, diff::ROBOT_CONST::PID_KO, 
    diff::ROBOT_CONST::PWM_MAX, diff::ROBOT_CONST::PWM_MIN);

bool state = false;

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_diff_ctl_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &enc_left_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "diff_ctl_left_enc"
    ));

    RCCHECK(rclc_publisher_init_default(
        &enc_right_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "diff_ctl_right_enc"
    ));

    RCCHECK(rclc_subscription_init_default(
        &motor_speed_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
        "diff_ctl_motor_cmd"
    ));

    const unsigned int timer_timeout = 1000 / diff::ROBOT_CONST::PID_RATE;
    
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true
    ));

    cmd_msg_speed.data.capacity = 2;
    cmd_msg_speed.data.size = 0;
    cmd_msg_speed.data.data = (int64_t*) malloc(cmd_msg_speed.data.capacity * sizeof(int64_t));

    cmd_msg_speed.layout.dim.capacity = 2;
    cmd_msg_speed.layout.dim.size = 0;
    cmd_msg_speed.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(
        cmd_msg_speed.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    for(size_t i = 0; i < cmd_msg_speed.layout.dim.capacity; ++i)
    {
        cmd_msg_speed.layout.dim.data[i].label.capacity = 2;
        cmd_msg_speed.layout.dim.data[i].label.size = 0;
        cmd_msg_speed.layout.dim.data[i].label.data = (char*) malloc(
            cmd_msg_speed.layout.dim.data[i].label.capacity * sizeof(char));
    }

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_subscription(&executor,
        &motor_speed_sub,
        &cmd_msg_speed,
        &cmd_motor_callback,
        ON_NEW_DATA
    ));

    enc_left_value.data = 0.0;
    enc_right_value.data = 0.0;

    motor_left.begin();
    motor_right.begin();

    enc_left.begin();
    enc_right.begin();
    
    attachInterrupt(diff::HARDWARE::ML_ENCA, &read_left_enc, CHANGE);
    attachInterrupt(diff::HARDWARE::MR_ENCA, &read_right_enc, CHANGE);

    pid_left.reset(enc_left.read());
    pid_right.reset(enc_right.read());
}

void loop()
{
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void IRAM_ATTR read_left_enc()
{
    enc_left.readEnc();
}

void IRAM_ATTR read_right_enc()
{
    enc_right.readEnc();
}

void adjust_motors_speed()
{
    int motor_left_sp = 0;
    int motor_right_sp = 0;

    pid_left.compute(enc_left.read(), motor_left_sp);
    pid_right.compute(enc_right.read(), motor_right_sp);

    if(pid_left.enabled())
    {
        motor_left.set_speed(motor_left_sp);
    }

    if(pid_right.enabled())
    {
        motor_right.set_speed(motor_right_sp);
    }
}

void set_motor_speed(int left_speed, int right_speed)
{
    const int motor_left_sp = left_speed;
    const int motor_right_sp = right_speed;

    if(motor_left_sp == 0)
    {
        motor_left.set_speed(0);
        pid_left.reset(enc_left.read());
        pid_left.disable();
    }
    else
    {
        pid_left.enable();
    }
    
    if(motor_right_sp == 0)
    {
        motor_right.set_speed(0);
        pid_right.reset(enc_right.read());
        pid_right.disable();
    }
    else
    {
        pid_right.enable();
    }

    pid_left.setSetpoint(motor_left_sp / diff::ROBOT_CONST::PID_RATE);
    pid_right.setSetpoint(motor_right_sp / diff::ROBOT_CONST::PID_RATE);
}

void error_loop()
{
    while(1)
    {
        delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_tm)
{
    RCLC_UNUSED(last_call_tm);
    if(timer != NULL)
    {
        adjust_motors_speed();
        enc_left_value.data = enc_left.read();
        enc_right_value.data = enc_right.read();

        RCSOFTCHECK(rcl_publish(&enc_left_pub, (const void*)&enc_left_value, NULL));
        RCSOFTCHECK(rcl_publish(&enc_right_pub, (const void*)&enc_right_value, NULL));
    }
}

void cmd_motor_callback(const void *msgin)
{
    const std_msgs__msg__Int64MultiArray * msg = (const std_msgs__msg__Int64MultiArray *) msgin;
    set_motor_speed(msg->data.data[0], msg->data.data[1]);
}