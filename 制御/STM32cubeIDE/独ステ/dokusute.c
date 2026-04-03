#include "dokusute.h"

FourWISOutput_t g_fourwis_output = {0};
JoystickInput_t g_joystick_input = {0};
float g_wheel_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float g_prev_angles[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}

static void optimize_steering_angle(float* angle, float* speed, float prev_angle) {
    float angle_diff = normalize_angle(*angle - prev_angle);

    if (abs_float(angle_diff) > M_PI / 2.0f) {
        if (angle_diff > 0.0f) {
            *angle = normalize_angle(*angle - M_PI);
        } else {
            *angle = normalize_angle(*angle + M_PI);
        }
        *speed = -*speed;
    }
}

static void set_steer_motor_output(SteerMotorOutput_t* motor, float angle, float wheel_speed, int wheel_index) {
    optimize_steering_angle(&angle, &wheel_speed, g_prev_angles[wheel_index]);

    g_prev_angles[wheel_index] = angle;

    motor->angle = angle;
    motor->normalized_speed = wheel_speed;

    if (wheel_speed >= 0.0f) {
        motor->direction = 0;
        motor->speed = (uint16_t)(wheel_speed * MAX_MOTOR_OUTPUT);
    } else {
        motor->direction = 1;
        motor->speed = (uint16_t)((-wheel_speed) * MAX_MOTOR_OUTPUT);
    }
}

FourWISOutput_t FourWIS_CalculateMotorOutput(JoystickInput_t joystick) {
    FourWISOutput_t output = {0};

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

    float l = HALF_WHEELBASE;
    float w = HALF_TREAD;

    float vx_fl = vx - vw * w;
    float vy_fl = vy + vw * l;

    float vx_fr = vx + vw * w;
    float vy_fr = vy + vw * l;

    float vx_rl = vx - vw * w;
    float vy_rl = vy - vw * l;

    float vx_rr = vx + vw * w;
    float vy_rr = vy - vw * l;

    float speed_fl = vector_magnitude(vx_fl, vy_fl);
    float speed_fr = vector_magnitude(vx_fr, vy_fr);
    float speed_rl = vector_magnitude(vx_rl, vy_rl);
    float speed_rr = vector_magnitude(vx_rr, vy_rr);

    g_wheel_speeds[0] = speed_fl;
    g_wheel_speeds[1] = speed_fr;
    g_wheel_speeds[2] = speed_rl;
    g_wheel_speeds[3] = speed_rr;

    float max_wheel_speed = max_float(
        max_float(speed_fl, speed_fr),
        max_float(speed_rl, speed_rr)
    );

    float scale_factor = 1.0f;
    if (max_wheel_speed > 1.0f) {
        scale_factor = 1.0f / max_wheel_speed;
        speed_fl *= scale_factor;
        speed_fr *= scale_factor;
        speed_rl *= scale_factor;
        speed_rr *= scale_factor;
    }

    output.max_wheel_speed = max_wheel_speed;
    output.scale_factor = scale_factor;

    float angle_fl, angle_fr, angle_rl, angle_rr;

    if (speed_fl < 0.001f) {
        angle_fl = g_prev_angles[0];
    } else {
        angle_fl = atan2f(vy_fl, vx_fl);
    }

    if (speed_fr < 0.001f) {
        angle_fr = g_prev_angles[1];
    } else {
        angle_fr = atan2f(vy_fr, vx_fr);
    }

    if (speed_rl < 0.001f) {
        angle_rl = g_prev_angles[2];
    } else {
        angle_rl = atan2f(vy_rl, vx_rl);
    }

    if (speed_rr < 0.001f) {
        angle_rr = g_prev_angles[3];
    } else {
        angle_rr = atan2f(vy_rr, vx_rr);
    }

    set_steer_motor_output(&output.fl, angle_fl, speed_fl, 0);
    set_steer_motor_output(&output.fr, angle_fr, speed_fr, 1);
    set_steer_motor_output(&output.rl, angle_rl, speed_rl, 2);
    set_steer_motor_output(&output.rr, angle_rr, speed_rr, 3);

    return output;
}

void FourWIS_UpdateGlobalOutputs(JoystickInput_t joystick) {
    g_joystick_input = joystick;
    g_fourwis_output = FourWIS_CalculateMotorOutput(joystick);
}
