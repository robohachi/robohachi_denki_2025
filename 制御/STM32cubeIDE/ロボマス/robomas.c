#include "robomas.h"

static void setup_can_filter(RoboMaster_t* rm);
static void set_output(RoboMaster_t* rm);
static void send_motor_currents(RoboMaster_t* rm, int32_t set_cur[ROBOMAS_MAX_MOTORS], uint8_t status_map[ROBOMAS_MAX_MOTORS]);
static int32_t rpm_pid(robomas_motor_t* motor);
static int32_t ang_pid(robomas_motor_t* motor);
static void motor_reset(robomas_motor_t* motor);
static void init_dwt(void);

void Timer_init(Timer_t* timer) {
    timer->start_time_ms = 0;
    timer->start_cycles = 0;
    timer->dwt_initialized = false;
    Timer_reset(timer);
}

void Timer_reset(Timer_t* timer) {
    timer->start_time_ms = HAL_GetTick();

    if (!timer->dwt_initialized) {
        init_dwt();
        timer->dwt_initialized = true;
    }
    timer->start_cycles = DWT->CYCCNT;
}

uint32_t Timer_get_ms(Timer_t* timer) {
    return HAL_GetTick() - timer->start_time_ms;
}

uint32_t Timer_get_us(Timer_t* timer) {
    uint32_t current_cycles = DWT->CYCCNT;
    uint32_t elapsed_cycles = current_cycles - timer->start_cycles;
    return elapsed_cycles / (SystemCoreClock / 1000000);
}

static void init_dwt(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void RoboMaster_init(RoboMaster_t* rm, CAN_HandleTypeDef* hcan) {
    if (rm == NULL || hcan == NULL) return;

    rm->hcan = hcan;
    rm->safety_enabled = false;
    rm->initialized = false;

    for (int i = 0; i < ROBOMAS_MAX_MOTORS; i++) {
        motor_reset(&rm->motors[i]);
    }

    Timer_init(&rm->sensor_timer);
    Timer_init(&rm->output_timer);

    // CANフィルター設定を有効化
    setup_can_filter(rm);

    rm->initialized = true;
}

void RoboMaster_process(RoboMaster_t* rm) {
    if (rm == NULL || !rm->initialized) return;

    if (Timer_get_us(&rm->sensor_timer) > 900) {
        Timer_reset(&rm->sensor_timer);

        for (int i = 0; i < ROBOMAS_MAX_MOTORS; i++) {
            robomas_motor_t* motor = &rm->motors[i];
            motor->filt_cur += ROBOMAS_CUR_LP_COEFF * (motor->now_cur - motor->filt_cur);
            motor->sens.cur = (int16_t)motor->filt_cur;
            motor->sens.is_received = 0;
        }
    }

    if (Timer_get_ms(&rm->output_timer) > 10) {
        Timer_reset(&rm->output_timer);
        set_output(rm);
    }
}

bool RoboMaster_can_callback(RoboMaster_t* rm, uint16_t can_id, const uint8_t* data, size_t len) {
    if (rm == NULL || data == NULL) return false;

    // 有効なモータIDかを先に確認してからインデックスを計算（配列外アクセス防止）
    if (can_id < 0x201 || can_id > 0x200 + ROBOMAS_MAX_MOTORS) return false;
    uint8_t mot_id = (uint8_t)(can_id - 0x201);
    if (mot_id < ROBOMAS_MAX_MOTORS) {
        robomas_motor_t* motor = &rm->motors[mot_id];
        int8_t direction = motor->param.direction;

        motor->sens.is_connected = 1;
        motor->sens.is_received = 1;

        uint16_t ang = ((int16_t)data[0] << 8) | data[1];
        motor->sens.rpm = (((int16_t)data[2] << 8) | data[3]) * direction;
        motor->now_cur = (((int16_t)data[4] << 8) | data[5]) * direction;

        if (motor->ang_befor < ang) {
            if (4096 < (ang - motor->ang_befor)) {
                motor->rota_count--;
            }
        } else {
            if (4096 < (motor->ang_befor - ang)) {
                motor->rota_count++;
            }
        }

        motor->sens.ang = ((motor->rota_count * 8191) + (int64_t)ang) * direction;
        motor->ang_befor = ang;
        return true;
    }
    return false;
}

void RoboMaster_set_motor_current(RoboMaster_t* rm, uint8_t motor_id, int16_t current) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;

    rm->motors[motor_id].power.mode = ROBOMAS_MODE_CURRENT;
    rm->motors[motor_id].power.cur = current;
}

void RoboMaster_set_motor_rpm(RoboMaster_t* rm, uint8_t motor_id, int16_t rpm) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;

    rm->motors[motor_id].power.mode = ROBOMAS_MODE_RPM;
    rm->motors[motor_id].power.rpm = rpm;
}

void RoboMaster_set_motor_angle(RoboMaster_t* rm, uint8_t motor_id, int64_t angle) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;

    rm->motors[motor_id].power.mode = ROBOMAS_MODE_ANGLE;
    rm->motors[motor_id].power.ang = angle;
}

void RoboMaster_disable_motor(RoboMaster_t* rm, uint8_t motor_id) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;

    rm->motors[motor_id].power.mode = ROBOMAS_MODE_DISABLE;
}

void RoboMaster_disable_all_motors(RoboMaster_t* rm) {
    if (rm == NULL) return;

    for (int i = 0; i < ROBOMAS_MAX_MOTORS; i++) {
        RoboMaster_disable_motor(rm, i);
    }
}

void RoboMaster_set_safety(RoboMaster_t* rm, bool enabled) {
    if (rm == NULL) return;
    rm->safety_enabled = enabled;
}

bool RoboMaster_get_safety(RoboMaster_t* rm) {
    if (rm == NULL) return true;
    return rm->safety_enabled;
}

robomas_sensor_t RoboMaster_get_sensor_data(RoboMaster_t* rm, uint8_t motor_id) {
    robomas_sensor_t empty = {0};
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return empty;

    return rm->motors[motor_id].sens;
}

bool RoboMaster_is_motor_connected(RoboMaster_t* rm, uint8_t motor_id) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return false;
    return rm->motors[motor_id].sens.is_connected;
}

void RoboMaster_set_pid_params(RoboMaster_t* rm, uint8_t motor_id,
                              float rpm_p, float rpm_i, float rpm_d,
                              float ang_p, float ang_i, float ang_d) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;

    robomas_param_t* param = &rm->motors[motor_id].param;
    param->rpm_p = rpm_p;
    param->rpm_i = rpm_i;
    param->rpm_d = rpm_d;
    param->ang_p = ang_p;
    param->ang_i = ang_i;
    param->ang_d = ang_d;
}

void RoboMaster_set_motor_direction(RoboMaster_t* rm, uint8_t motor_id, int8_t direction) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;
    rm->motors[motor_id].param.direction = (direction >= 0) ? 1 : -1;
}

void RoboMaster_set_max_current(RoboMaster_t* rm, uint8_t motor_id, int16_t max_cur) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;
    rm->motors[motor_id].param.max_cur = max_cur;
}

// ロボマス用CANフィルター設定（修正版）
static void setup_can_filter(RoboMaster_t* rm) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x200 << 5;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0xFF0 << 5;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(rm->hcan, &sFilterConfig);
}

static void set_output(RoboMaster_t* rm) {
    uint8_t status_map[ROBOMAS_MAX_MOTORS] = {0};
    int32_t set_cur[ROBOMAS_MAX_MOTORS];

    for (int i = 0; i < ROBOMAS_MAX_MOTORS; i++) {
        robomas_motor_t* motor = &rm->motors[i];
        status_map[i] = motor->sens.is_connected;

        robomas_mode_t mode = (robomas_mode_t)motor->power.mode;
        if (rm->safety_enabled) {
            mode = ROBOMAS_MODE_DISABLE;
        }

        switch (mode) {
            case ROBOMAS_MODE_CURRENT:
                set_cur[i] = motor->power.cur * motor->param.direction;
                motor->rpm_u_i = 0;
                motor->rpm_u_befor = 0;
                motor->ang_u_i = 0;
                motor->ang_u_befor = 0;
                break;

            case ROBOMAS_MODE_RPM:
                set_cur[i] = rpm_pid(motor);
                break;

            case ROBOMAS_MODE_ANGLE:
                set_cur[i] = ang_pid(motor);
                break;

            case ROBOMAS_MODE_DISABLE:
            default:
                set_cur[i] = 0;
                motor->rpm_u_i = 0;
                motor->rpm_u_befor = 0;
                motor->ang_u_i = 0;
                motor->ang_u_befor = 0;
                break;
        }

        if (motor->param.max_cur < set_cur[i]) {
            set_cur[i] = motor->param.max_cur;
        } else if (set_cur[i] < -motor->param.max_cur) {
            set_cur[i] = -motor->param.max_cur;
        }

        motor->sens.set_cur = set_cur[i];
    }

    send_motor_currents(rm, set_cur, status_map);
}

static void send_motor_currents(RoboMaster_t* rm, int32_t set_cur[ROBOMAS_MAX_MOTORS], uint8_t status_map[ROBOMAS_MAX_MOTORS]) {
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // モータ1-4への制御コマンド送信は常に実行
    tx_header.StdId = 0x200;
    tx_data[0] = (uint8_t)(set_cur[0] >> 8); tx_data[1] = (uint8_t)set_cur[0];
    tx_data[2] = (uint8_t)(set_cur[1] >> 8); tx_data[3] = (uint8_t)set_cur[1];
    tx_data[4] = (uint8_t)(set_cur[2] >> 8); tx_data[5] = (uint8_t)set_cur[2];
    tx_data[6] = (uint8_t)(set_cur[3] >> 8); tx_data[7] = (uint8_t)set_cur[3];
    HAL_CAN_AddTxMessage(rm->hcan, &tx_header, tx_data, &tx_mailbox);

    if ((status_map[4] + status_map[5]) != 0) {
        tx_header.StdId = 0x1FF;
        tx_data[0] = (uint8_t)(set_cur[4] >> 8); tx_data[1] = (uint8_t)set_cur[4];
        tx_data[2] = (uint8_t)(set_cur[5] >> 8); tx_data[3] = (uint8_t)set_cur[5];
        tx_data[4] = 0; tx_data[5] = 0;
        tx_data[6] = 0; tx_data[7] = 0;
        HAL_CAN_AddTxMessage(rm->hcan, &tx_header, tx_data, &tx_mailbox);
    }
}

static int32_t rpm_pid(robomas_motor_t* motor) {
    int32_t rpm_u = motor->power.rpm - motor->sens.rpm;

    motor->rpm_u_i += rpm_u;
    if (!(-motor->param.pid_i_max < motor->rpm_u_i && motor->rpm_u_i < motor->param.pid_i_max)) {
        motor->rpm_u_i -= rpm_u;
    }

    int32_t rpm_u_d = rpm_u - motor->rpm_u_befor;
    motor->rpm_u_befor = rpm_u;

    int32_t output = motor->param.rpm_p * rpm_u + motor->param.rpm_i * motor->rpm_u_i + motor->param.rpm_d * rpm_u_d;
    return (output + motor->power.cur) * motor->param.direction;
}

static int32_t ang_pid(robomas_motor_t* motor) {
    int32_t ang_u = motor->power.ang - motor->sens.ang;

    motor->ang_u_i += ang_u;
    if (!(-motor->param.pid_i_max < motor->ang_u_i && motor->ang_u_i < motor->param.pid_i_max)) {
        motor->ang_u_i -= ang_u;
    }

    int32_t ang_u_d = ang_u - motor->ang_u_befor;
    motor->ang_u_befor = ang_u;

    int32_t target_rpm = motor->param.ang_p * ang_u + motor->param.ang_i * motor->ang_u_i + motor->param.ang_d * ang_u_d;

    if (motor->power.rpm < target_rpm) {
        target_rpm = motor->power.rpm;
    } else if (target_rpm < -motor->power.rpm) {
        target_rpm = -motor->power.rpm;
    }

    int32_t rpm_u = target_rpm - motor->sens.rpm;
    motor->rpm_u_i += rpm_u;
    if (!(-motor->param.pid_i_max < motor->rpm_u_i && motor->rpm_u_i < motor->param.pid_i_max)) {
        motor->rpm_u_i -= rpm_u;
    }

    int32_t rpm_u_d = rpm_u - motor->rpm_u_befor;
    motor->rpm_u_befor = rpm_u;

    int32_t output = motor->param.rpm_p * rpm_u + motor->param.rpm_i * motor->rpm_u_i + motor->param.rpm_d * rpm_u_d;
    return (output + motor->power.cur) * motor->param.direction;
}

static void motor_reset(robomas_motor_t* motor) {
    motor->power.mode = ROBOMAS_MODE_DISABLE;
    motor->power.cur = 0;
    motor->power.rpm = 0;
    motor->power.ang = 0;

    motor->param.max_cur = 10000;
    motor->param.direction = 1;
    motor->param.rpm_p = 5.0f;
    motor->param.rpm_i = 0.1f;
    motor->param.rpm_d = 0.05f;
    motor->param.ang_p = 2.0f;
    motor->param.ang_i = 0.0f;
    motor->param.ang_d = 0.0f;
    motor->param.pid_i_max = 4000;

    motor->rpm_u_i = 0;
    motor->rpm_u_befor = 0;
    motor->ang_u_i = 0;
    motor->ang_u_befor = 0;

    motor->sens.is_connected = 0;
    motor->sens.is_received = 0;
    motor->sens.rpm = 0;
    motor->sens.cur = 0;
    motor->sens.set_cur = 0;
    motor->sens.ang = 0;

    motor->now_cur = 0;
    motor->filt_cur = 0.0f;
    motor->ang_befor = 0;
    motor->rota_count = 0;
}

void RoboMaster_reset_motor(RoboMaster_t* rm, uint8_t motor_id) {
    if (rm == NULL || motor_id >= ROBOMAS_MAX_MOTORS) return;
    motor_reset(&rm->motors[motor_id]);
}

void RoboMaster_print_status(RoboMaster_t* rm) {
    /* デバッグ出力が必要な場合はここにUART送信等を実装する */
    (void)rm;
}
