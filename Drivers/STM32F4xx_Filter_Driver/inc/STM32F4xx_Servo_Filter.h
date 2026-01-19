/* 
 * File: servo_filter.h
 * Description: Hybrid Median-Kalman Filter for Circular Angular Data
 * Target: STM32 (Cortex-M4/M7)
 */

#ifndef SERVO_FILTER_H
#define SERVO_FILTER_H

#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f

// --- Utility Functions ---
/**
 * @brief Constrain angle to [0, 360] (degrees)
 * @param x Angle in degrees
 * @return Constrained angle in [0, 360] (degrees)
 */
static inline float constrainAngle_360(float x) {
    x = fmodf(x, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x;
}

/**
 * @brief Calculate the shortest difference between two angles (degrees)
 * @param a First angle (degrees)
 * @param b Second angle (degrees)
 * @return Difference a - b in range [-180, 180]
 */
static inline float angleDiff(float a, float b) {
    float diff = a - b;
    while (diff < -180.0f) diff += 360.0f;
    while (diff > 180.0f)  diff -= 360.0f;
    return diff;
}

// --- Filter Structures ---

// 3-Tap Median Filter
typedef struct {
    float buffer[3];
    uint8_t head;
} Median3_t;

// 1D Kalman Filter (State: Position, Velocity)
typedef struct {
    // State Vector
    float angle;      // x (degrees)
    float velocity;   // x[2] (degrees/sec)
    
    // Covariance Matrix P (2x2) - Symmetric, store upper triangle
    float P[2][2];

    // Tuning Parameters
    float Q_angle;    // Process noise: angle variance
    float Q_vel;      // Process noise: velocity variance
    float R_meas;     // Measurement noise variance
    
    // Timing
    float dt;           // Loop time in seconds
    float default_dt;   // Default dt if first update
    uint32_t last_tick; // Last update tick (ms)
} KalmanState_t;

// --- Function Prototypes ---
/**
 * @brief Initialize Median Filter
 * @param f Pointer to Median3_t structure
 * @return None
 */
void Median3_Init(Median3_t* f);

/**
 * @brief Apply Median Filter
 * @param f Pointer to Median3_t structure
 * @param input New input sample
 * @return Filtered output
 */
float Median3_Apply(Median3_t* f, float input);

/**
 * @brief Initialize Kalman Filter
 * @param k Pointer to KalmanState_t structure
 * @param default_dt Default loop time in seconds (used for first update)
 * @param Q_ang Process noise variance for angle
 * @param Q_vel Process noise variance for velocity
 * @param R Measurement noise variance
 * @return None
 */
void Kalman_Init(KalmanState_t* k, float default_dt, float Q_ang, float Q_vel, float R);

/**
 * @brief Update Kalman Filter with new measurement
 * @param k Pointer to KalmanState_t structure
 * @param measAngle New measured angle (degrees)
 * @return Updated angle estimate (degrees)
 */
float Kalman_Update(KalmanState_t* k, float measAngle);

#endif // SERVO_FILTER_H