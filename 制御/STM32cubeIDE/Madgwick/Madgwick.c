/*
 * Madgwick.c
 *
 *  Created on: Mar 21, 2025
 *      Author: yuzuk
 */
/*
 * Madgwick.c
 * BMX055センサーデータを使用した姿勢推定のためのMadgwickフィルタ実装
 */

#include "Madgwick.h"

// グローバル変数
Quaternion_t g_quat = {1.0f, 0.0f, 0.0f, 0.0f};  // 初期クォータニオン
EulerAngles_t g_euler = {0.0f, 0.0f, 0.0f};      // オイラー角

/**
 * @brief Madgwickフィルタの初期化
 */
void MadgwickInit(void)
{
    // クォータニオンを初期化
    g_quat.q0 = 1.0f;
    g_quat.q1 = 0.0f;
    g_quat.q2 = 0.0f;
    g_quat.q3 = 0.0f;

    // オイラー角を初期化
    g_euler.roll = 0.0f;
    g_euler.pitch = 0.0f;
    g_euler.yaw = 0.0f;
}

/**
 * @brief Madgwickフィルタの更新
 * @param gx ジャイロセンサX軸値 (rad/s)
 * @param gy ジャイロセンサY軸値 (rad/s)
 * @param gz ジャイロセンサZ軸値 (rad/s)
 * @param ax 加速度センサX軸値 (g)
 * @param ay 加速度センサY軸値 (g)
 * @param az 加速度センサZ軸値 (g)
 * @param mx 地磁気センサX軸値 (uT)
 * @param my 地磁気センサY軸値 (uT)
 * @param mz 地磁気センサZ軸値 (uT)
 */
void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float q0 = g_quat.q0;
    float q1 = g_quat.q1;
    float q2 = g_quat.q2;
    float q3 = g_quat.q3;
    float beta = MADGWICK_BETA;

    // レート計算
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 加速度が正規化可能な場合
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 加速度を正規化
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 地磁気が正規化可能な場合
        if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            // 地磁気を正規化
            recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // 補助変数
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // 基準方向の磁場
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // 勾配降下アルゴリズム修正
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            // 勾配を正規化
            recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // クォータニオン更新
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }
    }

    // クォータニオン積分
    q0 += qDot1 * (1.0f / SAMPLE_FREQ);
    q1 += qDot2 * (1.0f / SAMPLE_FREQ);
    q2 += qDot3 * (1.0f / SAMPLE_FREQ);
    q3 += qDot4 * (1.0f / SAMPLE_FREQ);

    // クォータニオン正規化
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    g_quat.q0 = q0 * recipNorm;
    g_quat.q1 = q1 * recipNorm;
    g_quat.q2 = q2 * recipNorm;
    g_quat.q3 = q3 * recipNorm;

    // オイラー角に変換
    QuaternionToEuler(&g_quat, &g_euler);
}

/**
 * @brief クォータニオンからオイラー角に変換
 * @param q クォータニオン
 * @param euler オイラー角
 */
void QuaternionToEuler(Quaternion_t *q, EulerAngles_t *euler)
{
    // ロール（x軸周り）
    euler->roll = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3),
                        1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2)) * 180.0f / PI;

    // ピッチ（y軸周り）
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1)
        euler->pitch = copysignf(90.0f, sinp); // ジンバルロック時は±90度（すでに度数法）
    else
        euler->pitch = asinf(sinp) * 180.0f / PI;

    // ヨー（z軸周り）
    euler->yaw = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2),
                       1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3)) * 180.0f / PI;
}



