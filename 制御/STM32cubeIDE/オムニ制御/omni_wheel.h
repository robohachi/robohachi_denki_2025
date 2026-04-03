#ifndef INC_OMNI_WHEEL_H_
#define INC_OMNI_WHEEL_H_

#include <stdint.h>
#include <math.h>

#define MAX_MOTOR_OUTPUT 900
#define JOYSTICK_DEADZONE 0.05f
#define MAX_COMBINED_SPEED 1.0f
#define SQRT2 1.41421356f

typedef struct {
    float move_x;
    float move_y;
    float rotate_z;
} JoystickInput_t;

typedef struct {
    uint8_t direction;
    uint16_t speed;
    float normalized_speed;
} MotorOutput_t;

typedef struct {
    MotorOutput_t fl;
    MotorOutput_t fr;
    MotorOutput_t rl;
    MotorOutput_t rr;
    float max_wheel_speed;
    float scale_factor;
} OmniOutput_t;

extern OmniOutput_t g_omni_output;
extern JoystickInput_t g_joystick_input;
extern float g_wheel_speeds[4];

OmniOutput_t Omni_CalculateMotorOutput(JoystickInput_t joystick);
void Omni_UpdateGlobalOutputs(JoystickInput_t joystick);

float abs_float(float value);
float max_float(float a, float b);
float clamp_float(float value, float min, float max);
float apply_deadzone(float value, float deadzone);
float vector_magnitude(float x, float y);

#endif /* INC_OMNI_WHEEL_H_ */
