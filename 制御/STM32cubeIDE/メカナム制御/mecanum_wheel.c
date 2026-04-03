#include "mecanum_wheel.h"

MecanumOutput_t g_mecanum_output = {0};
JoystickInput_t g_joystick_input = {0};
float g_wheel_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // [FL, FR, RL, RR]

float abs_float(float value) {
    return (value < 0.0f) ? -value : value;
}

float max_float(float a, float b) {
    return (a > b) ? a : b;
}

float clamp_float(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

float apply_deadzone(float value, float deadzone) {
    if (abs_float(value) < deadzone) {
        return 0.0f;
    }

    float sign = (value >= 0.0f) ? 1.0f : -1.0f;
    float abs_val = abs_float(value);
    return sign * (abs_val - deadzone) / (1.0f - deadzone);
}

float vector_magnitude(float x, float y) {
    return sqrtf(x * x + y * y);
}

static void set_motor_output(MotorOutput_t* motor, float wheel_speed) {
    motor->normalized_speed = wheel_speed;

    if (wheel_speed >= 0.0f) {
        motor->direction = 0; // 正転
        motor->speed = (uint16_t)(wheel_speed * MAX_MOTOR_OUTPUT);
    } else {
        motor->direction = 1; // 逆転
        motor->speed = (uint16_t)((-wheel_speed) * MAX_MOTOR_OUTPUT);
    }
}

MecanumOutput_t Mecanum_CalculateMotorOutput(JoystickInput_t joystick) {
    MecanumOutput_t output = {0};

    float vx = joystick.move_x;
    float vy = joystick.move_y;
    float vw = joystick.rotate_z;

    vx = apply_deadzone(vx, JOYSTICK_DEADZONE);
    vy = apply_deadzone(vy, JOYSTICK_DEADZONE);
    vw = apply_deadzone(vw, JOYSTICK_DEADZONE);

    vx = clamp_float(vx, -1.0f, 1.0f);
    vy = clamp_float(vy, -1.0f, 1.0f);
    vw = clamp_float(vw, -1.0f, 1.0f);

    float combined_linear_speed = vector_magnitude(vx, vy);
    if (combined_linear_speed > MAX_COMBINED_SPEED) {
        float scale = MAX_COMBINED_SPEED / combined_linear_speed;
        vx *= scale;
        vy *= scale;
    }

    float wheel_fl = vy - vx - vw;
    float wheel_fr = vy + vx + vw;
    float wheel_rl = vy + vx - vw;
    float wheel_rr = vy - vx + vw;

    g_wheel_speeds[0] = wheel_fl;
    g_wheel_speeds[1] = wheel_fr;
    g_wheel_speeds[2] = wheel_rl;
    g_wheel_speeds[3] = wheel_rr;

    float max_wheel_speed = max_float(
        max_float(abs_float(wheel_fl), abs_float(wheel_fr)),
        max_float(abs_float(wheel_rl), abs_float(wheel_rr))
    );

    float scale_factor = 1.0f;
    if (max_wheel_speed > 1.0f) {
        scale_factor = 1.0f / max_wheel_speed;
        wheel_fl *= scale_factor;
        wheel_fr *= scale_factor;
        wheel_rl *= scale_factor;
        wheel_rr *= scale_factor;
    }

    output.max_wheel_speed = max_wheel_speed;
    output.scale_factor = scale_factor;

    set_motor_output(&output.fl, wheel_fl);
    set_motor_output(&output.fr, wheel_fr);
    set_motor_output(&output.rl, wheel_rl);
    set_motor_output(&output.rr, wheel_rr);

    return output;
}

void Mecanum_UpdateGlobalOutputs(JoystickInput_t joystick) {

    g_joystick_input = joystick;

    g_mecanum_output = Mecanum_CalculateMotorOutput(joystick);
}
