#ifndef INC_DOKUSUTE_H_
#define INC_DOKUSUTE_H_

#include <stdint.h>
#include <math.h>

#define MAX_MOTOR_OUTPUT 999
#define JOYSTICK_DEADZONE 0.05f
#define MAX_COMBINED_SPEED 1.0f

#define WHEELBASE 0.5f  // ホイールベース [m] (前後軸間距離)
#define TREAD 0.4f      // トレッド [m] (左右輪間距離)

#define HALF_WHEELBASE (WHEELBASE / 2.0f)  // l = L/2
#define HALF_TREAD (TREAD / 2.0f)          // w = T/2

typedef struct {
    float move_x;    // -1.0 ~ 1.0 (前後)
    float move_y;    // -1.0 ~ 1.0 (左右, 右が正)
    float rotate_z;  // -1.0 ~ 1.0 (回転, 反時計回りが正)
} JoystickInput_t;

typedef struct {
    float angle;              // ステアリング角度 [rad]
    float normalized_speed;   // -1.0 ~ 1.0
    uint16_t speed;           // モーター出力値 (0 ~ MAX_MOTOR_OUTPUT)
    uint8_t direction;        // 0: 正転, 1: 逆転
} SteerMotorOutput_t;

typedef struct {
    SteerMotorOutput_t fl;  // 左前
    SteerMotorOutput_t fr;  // 右前
    SteerMotorOutput_t rl;  // 左後
    SteerMotorOutput_t rr;  // 右後
    float max_wheel_speed;
    float scale_factor;
} FourWISOutput_t;

extern FourWISOutput_t g_fourwis_output;
extern JoystickInput_t g_joystick_input;
extern float g_wheel_speeds[4];

FourWISOutput_t FourWIS_CalculateMotorOutput(JoystickInput_t joystick);
void FourWIS_UpdateGlobalOutputs(JoystickInput_t joystick);

#endif /* INC_DOKUSUTE_H_ */
