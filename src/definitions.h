#include <SimplePWM.h>
#include <SimpleGPIO.h>
#include <SimpleSerialBT.h>
#include <QuadratureEncoder.h>
#include <HBridge.h>
#include <PID.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

const uint8_t motor_count = 3;
const uint8_t bldc_count = 1;
const uint8_t servo_count = 1;

SimplePWM step[motor_count];
SimpleGPIO dir[motor_count];
SerialBT bt;
SimpleGPIO enc[motor_count];
HBridge bldc[bldc_count];
QuadratureEncoder bldc_encoder[bldc_count];
PID bldc_pid[bldc_count];

const uint8_t step_pin[motor_count] = {23, 19, 17};
const uint8_t step_ch[motor_count] = {0, 1, 2};
const uint8_t dir_pin[motor_count] = {22, 18, 16};
const uint8_t enc_pin[motor_count] = {21, 34, 35};
uint8_t bldc_pin[bldc_count][2] = {{32, 33}};
uint8_t bldc_ch[bldc_count][2] = {{3, 4}};
uint8_t bldc_encoder_pin[bldc_count][2] = {{26, 27}};
uint8_t servo_pin[servo_count] = {25};

int64_t prev_time, current_time, dt_us = 100000;

char buffer[128];
int freq[motor_count] = {0, 0, 0};
volatile int count[motor_count] = {0, 0, 0};
float angle[motor_count] = {0.0f, 0.0f, 0.0f};
float target_angle[motor_count] = {0.0f, 0.0f, 0.0f};
float degrees_per_count[motor_count] = {1.0f, 1.0f, 1.0f};
float kp[motor_count] = {0.0f, 0.0f, 0.0f};
float bldc_reference[bldc_count] = {0.0f};
float bldc_gain[bldc_count][3] = {{1.0f, 0.0f, 0.0f}};
float bldc_saturation[bldc_count] = {100.0f};
float bldc_u[bldc_count] = {0.0f};
float bldc_error[bldc_count] = {0.0f};
float bldc_degrees_per_edge[bldc_count] = {0.3644462f};
float bldc_angle[bldc_count] = {0.0f};
float bldc_speed[bldc_count] = {0.0f};
float servo_angle[servo_count] = {90.0f};
const float servo_min_angle = 0.0f;
const float servo_max_angle = 180.0f;
const uint32_t servo_min_pulse_us = 500;
const uint32_t servo_max_pulse_us = 2500;
const uint32_t servo_period_us = 20000;
volatile uint32_t servo_pulse_us[servo_count] = {1500};
esp_timer_handle_t servo_period_timer[servo_count];
esp_timer_handle_t servo_pulse_timer[servo_count];

enum BldcControlMode
{
    BLDC_NO_CONTROL,
    BLDC_SPEED,
    BLDC_ANGLE,
};

BldcControlMode bldc_mode[bldc_count] = {BLDC_NO_CONTROL};

TimerConfig timer[motor_count] = {
    {LEDC_TIMER_0, 100},
    {LEDC_TIMER_1, 100},
    {LEDC_TIMER_2, 100},
};

TimerConfig bldc_timer[bldc_count] = {
    {LEDC_TIMER_3, 25000, LEDC_TIMER_10_BIT},
};
