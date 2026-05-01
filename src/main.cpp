#include <definitions.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>

void IRAM_ATTR encoderHandler(void *arg)
{
    uint8_t gpio = (uint8_t)(uintptr_t)arg;
    uint8_t motor = motor_count;

    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (enc_pin[i] == gpio)
        {
            motor = i;
            break;
        }
    }

    if (motor >= motor_count)
        return;

    if (dir[motor].get())
    {
        count[motor]++;
    }
    else
    {
        count[motor]--;
    }
}

static float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

uint32_t servoAngleToPulse(float angle)
{
    angle = clampFloat(angle, servo_min_angle, servo_max_angle);
    float ratio = (angle - servo_min_angle) / (servo_max_angle - servo_min_angle);
    return servo_min_pulse_us + (uint32_t)(ratio * (servo_max_pulse_us - servo_min_pulse_us));
}

void IRAM_ATTR servoPulseEnd(void *arg)
{
    uint8_t servo = (uint8_t)(uintptr_t)arg;
    if (servo < servo_count)
        gpio_set_level((gpio_num_t)servo_pin[servo], 0);
}

void IRAM_ATTR servoPeriodStart(void *arg)
{
    uint8_t servo = (uint8_t)(uintptr_t)arg;
    if (servo >= servo_count)
        return;

    gpio_set_level((gpio_num_t)servo_pin[servo], 1);
    esp_timer_start_once(servo_pulse_timer[servo], servo_pulse_us[servo]);
}

void setMotorTarget(uint8_t motor, float target, float gain)
{
    if (motor >= motor_count)
        return;

    target_angle[motor] = target;
    kp[motor] = gain;
}

void setServoTarget(uint8_t servo, float target)
{
    if (servo >= servo_count)
        return;

    servo_angle[servo] = clampFloat(target, servo_min_angle, servo_max_angle);
    servo_pulse_us[servo] = servoAngleToPulse(servo_angle[servo]);
}

void setBldcTarget(uint8_t motor, int mode, float reference, float p, float i, float d)
{
    if (motor >= bldc_count)
        return;

    if (mode < BLDC_NO_CONTROL || mode > BLDC_ANGLE)
        return;

    bldc_mode[motor] = (BldcControlMode)mode;
    bldc_reference[motor] = reference;
    bldc_gain[motor][0] = p;
    bldc_gain[motor][1] = i;
    bldc_gain[motor][2] = d;
    bldc_pid[motor].setup(bldc_gain[motor], dt_us / 1000000.0f, bldc_saturation[motor]);
}

void readBluetoothCommand()
{
    int len = bt.available();
    if (!len)
        return;

    if (len >= (int)sizeof(buffer))
        len = sizeof(buffer) - 1;

    memset(buffer, 0, sizeof(buffer));
    bt.read(buffer, len);

    float all_pos[motor_count];
    float all_kp[motor_count];
    int motor;
    float target;
    float gain;
    int bldc_motor, bldc_control_mode;
    float bldc_ref, p, i, d;
    int servo_motor;
    float servo_ref;

    if (sscanf(buffer, "S,%d,%f", &servo_motor, &servo_ref) == 2)
    {
        if (servo_motor >= 1 && servo_motor <= servo_count)
            setServoTarget(servo_motor - 1, servo_ref);
        return;
    }

    if (sscanf(buffer, "B,%d,%d,%f,%f,%f,%f",
               &bldc_motor, &bldc_control_mode, &bldc_ref, &p, &i, &d) == 6)
    {
        if (bldc_motor >= 1 && bldc_motor <= bldc_count)
            setBldcTarget(bldc_motor - 1, bldc_control_mode, bldc_ref, p, i, d);
        return;
    }

    if (sscanf(buffer, "%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2],
               &bldc_control_mode, &bldc_ref, &p, &i, &d,
               &servo_ref) == 12)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i], all_kp[i]);
        setBldcTarget(0, bldc_control_mode, bldc_ref, p, i, d);
        setServoTarget(0, servo_ref);
        return;
    }

    if (sscanf(buffer, "%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2],
               &bldc_control_mode, &bldc_ref, &p, &i, &d) == 11)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i], all_kp[i]);
        setBldcTarget(0, bldc_control_mode, bldc_ref, p, i, d);
        return;
    }

    if (sscanf(buffer, "%f,%f,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2]) == 6)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i], all_kp[i]);
        return;
    }

    if (sscanf(buffer, "%d,%f,%f", &motor, &target, &gain) == 3)
    {
        if (motor >= 1 && motor <= motor_count)
            setMotorTarget(motor - 1, target, gain);
        return;
    }

    if (sscanf(buffer, "%f,%f", &target, &gain) == 2)
        setMotorTarget(0, target, gain);
}

void updateStepMotor(uint8_t motor)
{
    angle[motor] = count[motor] * degrees_per_count[motor];
    float error = target_angle[motor] - angle[motor];
    float u = kp[motor] * error;
    freq[motor] = (int)u;

    if (abs(freq[motor]) < 5)
    {
        step[motor].setDuty(0);
        return;
    }

    if (freq[motor] < 0)
    {
        dir[motor].set(0);
        if (freq[motor] < -750)
            freq[motor] = -750;
        step[motor].setFrequency(abs(freq[motor]));
    }
    else
    {
        dir[motor].set(1);
        if (freq[motor] > 750)
            freq[motor] = 750;
        step[motor].setFrequency(freq[motor]);
    }

    step[motor].setDuty(10);
}

void updateBldcMotor(uint8_t motor)
{
    bldc_angle[motor] = bldc_encoder[motor].getAngle();
    bldc_speed[motor] = bldc_encoder[motor].getSpeed();

    switch (bldc_mode[motor])
    {
    case BLDC_NO_CONTROL:
        bldc_u[motor] = bldc_reference[motor];
        break;
    case BLDC_SPEED:
        bldc_error[motor] = bldc_reference[motor] - bldc_speed[motor];
        bldc_u[motor] = bldc_pid[motor].calculate(bldc_error[motor]);
        break;
    case BLDC_ANGLE:
        bldc_error[motor] = bldc_reference[motor] - bldc_angle[motor];
        bldc_u[motor] = bldc_pid[motor].calculate(bldc_error[motor]);
        break;
    }

    if (std::fabs(bldc_u[motor]) < 0.01f)
        bldc[motor].setStop(false);
    else
        bldc[motor].setDuty(bldc_u[motor]);
}

void controlMotors()
{
    for (uint8_t i = 0; i < motor_count; i++)
        updateStepMotor(i);

    for (uint8_t i = 0; i < bldc_count; i++)
        updateBldcMotor(i);
}

extern "C" void app_main()
{
    bt.begin("btmj2");
    for (uint8_t i = 0; i < motor_count; i++)
    {
        step[i].setup(step_pin[i], step_ch[i], &timer[i]);
        dir[i].setup(dir_pin[i], GPIO_MODE_INPUT_OUTPUT);
        enc[i].setup(enc_pin[i], GPIO_MODE_INPUT);
        enc[i].addInterrupt(GPIO_INTR_POSEDGE, encoderHandler);
    }

    for (uint8_t i = 0; i < servo_count; i++)
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << servo_pin[i]);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level((gpio_num_t)servo_pin[i], 0);

        esp_timer_create_args_t pulse_args = {};
        pulse_args.callback = servoPulseEnd;
        pulse_args.arg = (void *)(uintptr_t)i;
        pulse_args.name = "servo_pulse";
        esp_timer_create(&pulse_args, &servo_pulse_timer[i]);

        esp_timer_create_args_t period_args = {};
        period_args.callback = servoPeriodStart;
        period_args.arg = (void *)(uintptr_t)i;
        period_args.name = "servo_period";
        esp_timer_create(&period_args, &servo_period_timer[i]);

        setServoTarget(i, servo_angle[i]);
        esp_timer_start_periodic(servo_period_timer[i], servo_period_us);
    }

    for (uint8_t i = 0; i < bldc_count; i++)
    {
        bldc[i].setup(bldc_pin[i], bldc_ch[i], &bldc_timer[i], 19.0f);
        bldc_encoder[i].setup(bldc_encoder_pin[i], bldc_degrees_per_edge[i]);
        bldc_pid[i].setup(bldc_gain[i], dt_us / 1000000.0f, bldc_saturation[i]);
    }

    prev_time = esp_timer_get_time();

    while (1)
    {
        current_time = esp_timer_get_time();

        if (current_time - prev_time >= dt_us)
        {
            prev_time = current_time;
            readBluetoothCommand();
            controlMotors();

            printf("%.2f,%.2f,%.2f | %d,%d,%d | %.2f,%.2f,%.2f | %.2f\n",
                   angle[0], angle[1], angle[2],
                   freq[0], freq[1], freq[2],
                   bldc_angle[0], bldc_speed[0], bldc_u[0],
                   servo_angle[0]);
        }
    }
}
