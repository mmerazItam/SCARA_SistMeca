#include <definitions.h>
#include <ScaraRobotControl.h>
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
        count[motor] = count[motor] + 1;
    }
    else
    {
        count[motor] = count[motor] - 1;
    }
}

void setMotorTarget(uint8_t motor, float target)
{
    if (motor >= motor_count)
        return;

    target_angle[motor] = target;
    kp[motor] = fixed_gain;

    if (debug_enabled)
        printf("[STEP] motor=%u target=%.2f gain=%.2f\n", motor + 1, target_angle[motor], kp[motor]);
}

void setBldcTarget(uint8_t motor,
                   BldcControlMode control_mode,
                   float reference,
                   float p,
                   float i,
                   float d)
{
    if (motor >= bldc_count)
        return;

    bldc_mode[motor] = control_mode;
    bldc_reference[motor] = reference;
    bldc_gain[motor][0] = p;
    bldc_gain[motor][1] = i;
    bldc_gain[motor][2] = d;
    bldc_pid[motor].setup(bldc_gain[motor], dt_us / 1000000.0f, bldc_saturation[motor]);

    if (debug_enabled)
        printf("[BLDC] motor=%u mode=%d ref=%.2f kp=%.2f ki=%.2f kd=%.2f\n",
               motor + 1, bldc_mode[motor], bldc_reference[motor],
               bldc_gain[motor][0], bldc_gain[motor][1], bldc_gain[motor][2]);
}

void setBldcTarget(uint8_t motor, float reference)
{
    setBldcTarget(motor, BLDC_POSITION, reference, fixed_gain, 0.0f, 0.0f);
}

void setBldcPositionTarget(uint8_t motor, float reference)
{
    setBldcTarget(motor, BLDC_POSITION, reference, fixed_gain, 0.0f, 0.0f);
}

void stopBldc(uint8_t motor)
{
    if (motor >= bldc_count)
        return;

    bldc_u[motor] = 0.0f;
    bldc_error[motor] = 0.0f;
    bldc[motor].setDuty(0.0f);
}

static char serial_command_buffer[sizeof(buffer)];
static size_t serial_command_len = 0;

void trimCommand(char *command)
{
    size_t len = strlen(command);
    while (len > 0 && (command[len - 1] == '\r' ||
                       command[len - 1] == '\n' ||
                       command[len - 1] == ' ' ||
                       command[len - 1] == '\t'))
    {
        command[len - 1] = '\0';
        len--;
    }
}

bool processCommand(const char *source, char *command)
{
    trimCommand(command);
    if (command[0] == '\0')
        return false;

    if (debug_enabled)
        printf("[%s] cmd=\"%s\"\n", source, command);

    float all_pos[motor_count];
    float all_kp[motor_count];
    int motor;
    float target;
    float gain;
    int bldc_motor, bldc_control_mode;
    float bldc_ref, p, i, d;
    float ignored;
    float scara_x, scara_y, scara_z, scara_phi;
    int scara_elbow;

    if (sscanf(command, "K,%f,%f,%f,%f,%d",
               &scara_x, &scara_y, &scara_z, &scara_phi, &scara_elbow) == 5)
    {
        bool ok = setScaraTarget(scara_x, scara_y, scara_z, scara_phi,
                                 scara_elbow == 1 ? SCARA_ELBOW_UP : SCARA_ELBOW_DOWN);
        if (debug_enabled)
            printf("[%s] scara ok=%d x=%.2f y=%.2f z=%.2f phi=%.2f elbow=%d\n",
                   source, ok, scara_x, scara_y, scara_z, scara_phi, scara_elbow);
        return true;
    }

    if (sscanf(command, "K,%f,%f,%f,%f",
               &scara_x, &scara_y, &scara_z, &scara_phi) == 4)
    {
        bool ok = setScaraTarget(scara_x, scara_y, scara_z, scara_phi);
        if (debug_enabled)
            printf("[%s] scara ok=%d x=%.2f y=%.2f z=%.2f phi=%.2f elbow=0\n",
                   source, ok, scara_x, scara_y, scara_z, scara_phi);
        return true;
    }

    if (sscanf(command, "B,%d,%d,%f,%f,%f,%f",
               &bldc_motor, &bldc_control_mode, &bldc_ref, &p, &i, &d) == 6)
    {
        if (bldc_motor >= 1 && bldc_motor <= bldc_count)
            setBldcTarget(bldc_motor - 1, (BldcControlMode)bldc_control_mode, bldc_ref, p, i, d);
        return true;
    }

    if (sscanf(command, "B,%d,%f", &bldc_motor, &bldc_ref) == 2)
    {
        if (bldc_motor >= 1 && bldc_motor <= bldc_count)
            setBldcTarget(bldc_motor - 1, bldc_ref);
        return true;
    }

    if (sscanf(command, "%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2],
               &bldc_control_mode, &bldc_ref, &p, &i, &d,
               &ignored) == 12)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i]);
        setBldcTarget(0, (BldcControlMode)bldc_control_mode, bldc_ref, p, i, d);
        return true;
    }

    if (sscanf(command, "%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2],
               &bldc_control_mode, &bldc_ref, &p, &i, &d) == 11)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i]);
        setBldcTarget(0, (BldcControlMode)bldc_control_mode, bldc_ref, p, i, d);
        return true;
    }

    if (sscanf(command, "%f,%f,%f,%f",
               &all_pos[0],
               &all_pos[1],
               &all_pos[2],
               &bldc_ref) == 4)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i]);
        setBldcTarget(0, bldc_ref);
        return true;
    }

    if (sscanf(command, "%f,%f,%f,%f,%f,%f",
               &all_pos[0], &all_kp[0],
               &all_pos[1], &all_kp[1],
               &all_pos[2], &all_kp[2]) == 6)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i]);
        return true;
    }

    if (sscanf(command, "%f,%f,%f",
               &all_pos[0],
               &all_pos[1],
               &all_pos[2]) == 3)
    {
        for (uint8_t i = 0; i < motor_count; i++)
            setMotorTarget(i, all_pos[i]);
        return true;
    }

    if (sscanf(command, "%d,%f,%f", &motor, &target, &gain) == 3)
    {
        if (motor >= 1 && motor <= motor_count)
            setMotorTarget(motor - 1, target);
        return true;
    }

    if (sscanf(command, "%d,%f", &motor, &target) == 2)
    {
        if (motor >= 1 && motor <= motor_count)
            setMotorTarget(motor - 1, target);
        return true;
    }

    if (sscanf(command, "%f,%f", &target, &gain) == 2)
    {
        setMotorTarget(0, target);
        return true;
    }

    if (debug_enabled)
        printf("[%s] unhandled cmd=\"%s\"\n", source, command);

    return false;
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
    processCommand("BT", buffer);
}

void readSerialCommand()
{
    int available = serial_monitor.available();
    while (available > 0)
    {
        char c;
        int read_len = serial_monitor.read(&c, 1);
        if (read_len <= 0)
            break;

        if (c == '\r' || c == '\n')
        {
            if (serial_command_len > 0)
            {
                serial_command_buffer[serial_command_len] = '\0';
                processCommand("SERIAL", serial_command_buffer);
                serial_command_len = 0;
            }
        }
        else if (serial_command_len < sizeof(serial_command_buffer) - 1)
        {
            serial_command_buffer[serial_command_len++] = c;
        }
        else
        {
            serial_command_buffer[serial_command_len] = '\0';
            processCommand("SERIAL", serial_command_buffer);
            serial_command_len = 0;
        }

        available = serial_monitor.available();
    }
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
        bldc_error[motor] = 0.0f;
        break;
    case BLDC_SPEED:
        bldc_error[motor] = bldc_reference[motor] - bldc_speed[motor];
        bldc_u[motor] = bldc_pid[motor].calculate(bldc_error[motor]);
        break;
    case BLDC_POSITION:
        bldc_error[motor] = bldc_reference[motor] - bldc_angle[motor];
        bldc_u[motor] = bldc_pid[motor].calculate(bldc_error[motor]);
        break;
    }

    bldc[motor].setDuty(bldc_u[motor]);

    if (debug_enabled)
        printf("[BLDC] motor=%u mode=%d ref=%.2f angle=%.2f error=%.2f speed=%.2f u=%.2f\n",
               motor + 1, bldc_mode[motor], bldc_reference[motor], bldc_angle[motor],
               bldc_error[motor], bldc_speed[motor], bldc_u[motor]);
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
            readSerialCommand();
            controlMotors();

            if (debug_state_enabled)
            {
                printf("[STATE] step_angle=%.2f,%.2f,%.2f step_target=%.2f,%.2f,%.2f freq=%d,%d,%d bldc_ref=%.2f bldc_angle=%.2f bldc_error=%.2f bldc_speed=%.2f bldc_u=%.2f\n",
                       angle[0], angle[1], angle[2],
                       target_angle[0], target_angle[1], target_angle[2],
                       freq[0], freq[1], freq[2],
                       bldc_reference[0], bldc_angle[0], bldc_error[0], bldc_speed[0], bldc_u[0]);
            }
        }
    }
}
