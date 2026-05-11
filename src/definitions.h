#include <SimplePWM.h>
#include <SimpleGPIO.h>
#include <SimpleSerialBT.h>
#include <SimpleUART.h>
#include <QuadratureEncoder.h>
#include <HBridge.h>
#include <PID.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

const uint8_t motor_count = 3;
const uint8_t bldc_count = 1;

SimplePWM step[motor_count];
SimpleGPIO dir[motor_count];
SerialBT bt;
SimpleUART serial_monitor(115200);
SimpleGPIO enc[motor_count];
HBridge bldc[bldc_count];
QuadratureEncoder bldc_encoder[bldc_count];
PID bldc_pid[bldc_count];

const uint8_t step_pin[motor_count] = {23, 19, 17};
const uint8_t step_ch[motor_count] = {0, 1, 2};
const uint8_t dir_pin[motor_count] = {22, 18, 13};
const uint8_t enc_pin[motor_count] = {21, 16, 12};

const uint8_t motor4_l298n_in1_pin = 32;
const uint8_t motor4_l298n_in2_pin = 33;
const uint8_t motor4_l298n_in1_pwm_ch = 3;
const uint8_t motor4_l298n_in2_pwm_ch = 4;
const uint8_t motor4_encoder_a_pin = 26;
const uint8_t motor4_encoder_b_pin = 27;

uint8_t bldc_pin[bldc_count][2] = {{motor4_l298n_in1_pin, motor4_l298n_in2_pin}};
uint8_t bldc_ch[bldc_count][2] = {{motor4_l298n_in1_pwm_ch, motor4_l298n_in2_pwm_ch}};
uint8_t bldc_encoder_pin[bldc_count][2] = {{motor4_encoder_a_pin, motor4_encoder_b_pin}};

int64_t prev_time, current_time, dt_us = 5000;

char buffer[128];
const bool debug_enabled = false;
const bool debug_state_enabled = false;
int freq[motor_count] = {0, 0, 0};
volatile int count[motor_count] = {0, 0, 0};
float angle[motor_count] = {0.0f, 0.0f, 0.0f};
float target_angle[motor_count] = {0.0f, 0.0f, 0.0f};
float degrees_per_count[motor_count] = {1.0f, 1.0f, 1.0f};
const float fixed_gain = 0.2f;
float kp[motor_count] = {fixed_gain, fixed_gain, fixed_gain};
float bldc_reference[bldc_count] = {0.0f};
float bldc_gain[bldc_count][3] = {{1.0f, 0.0f, 0.0f}};
float bldc_saturation[bldc_count] = {100.0f};
float bldc_u[bldc_count] = {0.0f};
float bldc_error[bldc_count] = {0.0f};
float bldc_degrees_per_edge[bldc_count] = {0.3644462f};
float bldc_angle[bldc_count] = {0.0f};
float bldc_speed[bldc_count] = {0.0f};

enum BldcControlMode
{
    BLDC_NO_CONTROL,
    BLDC_SPEED,
    BLDC_POSITION,
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
