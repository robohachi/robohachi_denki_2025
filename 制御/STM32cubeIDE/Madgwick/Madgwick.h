/*
 * Madgwick.h
 *
 *  Created on: Mar 21, 2025
 *      Author: yuzuk
 */

/*
 * Madgwick.h
 * BMX055センサーデータを使用した姿勢推定のためのMadgwickフィルタ実装
 */

#ifndef SRC_MADGWICK_H_
#define SRC_MADGWICK_H_

#include "main.h"
#include <math.h>

// 定数定義
#define MADGWICK_BETA 0.1f         // フィルタゲイン
#define SAMPLE_FREQ   10.0f        // サンプリング周波数 (Hz)
#define PI            3.14159265f

// クォータニオン構造体
typedef struct {
    float q0, q1, q2, q3;
} Quaternion_t;

// オイラー角構造体
typedef struct {
    float roll;     // x軸周りの回転 (ロール)
    float pitch;    // y軸周りの回転 (ピッチ)
    float yaw;      // z軸周りの回転 (ヨー)
} EulerAngles_t;

// グローバル変数
extern Quaternion_t g_quat;
extern EulerAngles_t g_euler;

// 関数プロトタイプ
void MadgwickInit(void);
void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void QuaternionToEuler(Quaternion_t *q, EulerAngles_t *euler);

#endif /* SRC_MADGWICK_H_ */
