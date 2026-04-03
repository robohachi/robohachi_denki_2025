/*
 * robomas.h
 *
 *  Created on: Aug 26, 2025
 *      Author: yuzuk
 */

#ifndef INC_ROBOMAS_H_
#define INC_ROBOMAS_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// 定数定義
// =============================================================================
#define ROBOMAS_CUR_LP_COEFF (0.1f)
#define ROBOMAS_MAX_MOTORS   (6)

// =============================================================================
// タイマー構造体と関数
// =============================================================================
typedef struct {
    uint32_t start_time_ms;
    uint32_t start_cycles;
    bool dwt_initialized;
} Timer_t;

// タイマー関数宣言
void Timer_init(Timer_t* timer);
void Timer_reset(Timer_t* timer);
uint32_t Timer_get_ms(Timer_t* timer);
uint32_t Timer_get_us(Timer_t* timer);

// =============================================================================
// RoboMaster制御関連の構造体定義
// =============================================================================

// 制御モード列挙型
typedef enum {
    ROBOMAS_MODE_DISABLE = 0,
    ROBOMAS_MODE_CURRENT = 1,
    ROBOMAS_MODE_RPM = 2,
    ROBOMAS_MODE_ANGLE = 3,
} robomas_mode_t;

// 制御コマンド構造体
typedef struct {
    uint8_t mode;
    int16_t cur;
    int16_t rpm;
    int64_t ang;
} __attribute__((__packed__)) robomas_power_t;

// PIDパラメータ構造体
typedef struct {
    int16_t max_cur;
    int8_t direction;
    float rpm_p;
    float rpm_i;
    float rpm_d;
    float ang_p;
    float ang_i;
    float ang_d;
    int32_t pid_i_max;
} __attribute__((__packed__)) robomas_param_t;//PID等パラメータ

// センサーデータ構造体
typedef struct {
    uint8_t is_received : 1;
    uint8_t is_connected : 1;
    uint8_t dummy : 6;
    int16_t rpm;
    int16_t cur;
    int16_t set_cur;
    int64_t ang; // [8191 / rota]
} __attribute__((__packed__)) robomas_sensor_t;

// 個別モータ制御構造体
typedef struct {
    robomas_power_t power;
    robomas_param_t param;

    // PID制御用変数
    int32_t rpm_u_i;
    int16_t rpm_u_befor;
    int32_t ang_u_i;
    int16_t ang_u_befor;

    // センサーデータ
    robomas_sensor_t sens;
    int16_t now_cur;
    float filt_cur;
    uint16_t ang_befor;
    int32_t rota_count;
} robomas_motor_t;

// RoboMasterメイン構造体
typedef struct {
    robomas_motor_t motors[ROBOMAS_MAX_MOTORS];
    Timer_t sensor_timer;
    Timer_t output_timer;
    CAN_HandleTypeDef* hcan;
    bool safety_enabled;
    bool initialized;
} RoboMaster_t;

// =============================================================================
// 関数宣言
// =============================================================================

// 初期化・制御関数
void RoboMaster_init(RoboMaster_t* rm, CAN_HandleTypeDef* hcan);
void RoboMaster_process(RoboMaster_t* rm);
bool RoboMaster_can_callback(RoboMaster_t* rm, uint16_t can_id, const uint8_t* data, size_t len);

// 制御コマンド設定関数
void RoboMaster_set_motor_current(RoboMaster_t* rm, uint8_t motor_id, int16_t current);
void RoboMaster_set_motor_rpm(RoboMaster_t* rm, uint8_t motor_id, int16_t rpm);
void RoboMaster_set_motor_angle(RoboMaster_t* rm, uint8_t motor_id, int64_t angle);
void RoboMaster_disable_motor(RoboMaster_t* rm, uint8_t motor_id);
void RoboMaster_disable_all_motors(RoboMaster_t* rm);

// セーフティ制御
void RoboMaster_set_safety(RoboMaster_t* rm, bool enabled);
bool RoboMaster_get_safety(RoboMaster_t* rm);

// センサーデータ取得
robomas_sensor_t RoboMaster_get_sensor_data(RoboMaster_t* rm, uint8_t motor_id);
bool RoboMaster_is_motor_connected(RoboMaster_t* rm, uint8_t motor_id);

// PIDパラメータ設定
void RoboMaster_set_pid_params(RoboMaster_t* rm, uint8_t motor_id,
                              float rpm_p, float rpm_i, float rpm_d,
                              float ang_p, float ang_i, float ang_d);
void RoboMaster_set_motor_direction(RoboMaster_t* rm, uint8_t motor_id, int8_t direction);
void RoboMaster_set_max_current(RoboMaster_t* rm, uint8_t motor_id, int16_t max_cur);

// デバッグ・ステータス関数
void RoboMaster_print_status(RoboMaster_t* rm);
void RoboMaster_reset_motor(RoboMaster_t* rm, uint8_t motor_id);

#ifdef __cplusplus
}
#endif



#endif /* INC_ROBOMAS_H_ */
