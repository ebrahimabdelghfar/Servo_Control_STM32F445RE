/* File: servo_filter.c */
#include "STM32F4xx_Servo_Filter.h"

void Median3_Init(Median3_t* f) {
    for(int i=0; i<3; i++) f->buffer[i] = 0.0f;
    f->head = 0;
}

float Median3_Apply(Median3_t* f, float input) {
    // 1. Insert into circular buffer
    f->buffer[f->head] = input;
    f->head = (f->head + 1) % 3;

    float a = f->buffer[0];
    float b = f->buffer[1];
    float c = f->buffer[2];

    // 2. Circular Unwrapping Logic
    // We assume the true spread is < 180.
    // Map b and c relative to a to handle wrap-around.
    float b_unwrapped = a + angleDiff(b, a);
    float c_unwrapped = a + angleDiff(c, a);

    // 3. Sorting Network (3-input)
    // We want the median of {0, b-a, c-a} + a
    // Simpler: just sort a, b_u, c_u
    float mn, md, mx; // min, median, max

    if (a > b_unwrapped) {
        if (b_unwrapped > c_unwrapped) { // a > b > c
            md = b_unwrapped;
        } else if (a > c_unwrapped) {    // a > c > b
            md = c_unwrapped;
        } else {                         // c > a > b
            md = a;
        }
    } else { // b >= a
        if (a > c_unwrapped) {           // b > a > c
            md = a;
        } else if (b_unwrapped > c_unwrapped) { // b > c > a
            md = c_unwrapped;
        } else {                         // c > b > a
            md = b_unwrapped;
        }
    }

    // 4. Wrap result back to [0, 360)
    return constrainAngle_360(md);
}

void Kalman_Init(KalmanState_t* k, float default_dt, float Q_ang, float Q_vel, float R) {
    k->angle = 0.0f;
    k->velocity = 0.0f;
    k->default_dt = default_dt;
    k->dt = default_dt;
    k->last_tick = 0;  // Will be set on first update
    k->Q_angle = Q_ang;
    k->Q_vel = Q_vel;
    k->R_meas = R;
    
    // Initialize covariance matrix
    k->P[0][0] = 100.0f; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;   k->P[1][1] = 100.0f;
}

float Kalman_Update(KalmanState_t* k, float measAngle) {
    // --- 0. CALCULATE DT ---
    uint32_t current_tick = HAL_GetTick();
    float dt;
    if (k->last_tick == 0) {
        // First update, use default dt
        dt = k->default_dt;
    } else {
        // Calculate elapsed time in seconds
        uint32_t elapsed_ms = current_tick - k->last_tick;
        dt = (float)elapsed_ms / 1000.0f;
        // Clamp dt to reasonable bounds (0.1ms to 1s)
        if (dt < 0.0001f) dt = k->default_dt;
        if (dt > 1.0f) dt = k->default_dt;
    }
    k->last_tick = current_tick;
    k->dt = dt;  // Store for reference

    // --- 1. PREDICT STEP ---
    // State Prediction: x = F * x
    // angle = angle + velocity * dt
    float newAngle = k->angle + k->velocity * dt;
    newAngle = constrainAngle_360(newAngle); // Keep state bounded
    float newVel = k->velocity;

    // Covariance Prediction: P = F*P*F' + Q
    // F = [1 dt; 0 1]
    float P00 = k->P[0][0] + dt * (k->P[0][1] + k->P[1][0]) + dt*dt * k->P[1][1] + k->Q_angle;
    float P01 = k->P[0][1] + dt * k->P[1][1];
    float P10 = k->P[1][0] + dt * k->P[1][1];
    float P11 = k->P[1][1] + k->Q_vel;

    // --- 2. UPDATE STEP ---
    // Innovation (Measurement Residual) with Circular Handling
    float y = angleDiff(measAngle, newAngle); 

    // Innovation Covariance: S = H*P*H' + R
    // H = [1 0], so S = P00 + R
    float S = P00 + k->R_meas;

    // Kalman Gain: K = P*H' * inv(S)
    float K0 = P00 / S;
    float K1 = P10 / S;

    // State Update: x = x + K*y
    k->angle = constrainAngle_360(newAngle + K0 * y);
    k->velocity = newVel + K1 * y;

    // Covariance Update: P = (I - K*H) * P
    float P00_new = (1.0f - K0) * P00;
    float P01_new = (1.0f - K0) * P01;
    float P10_new = P10 - K1 * P00;
    float P11_new = P11 - K1 * P01;

    k->P[0][0] = P00_new;
    k->P[0][1] = P01_new;
    k->P[1][0] = P10_new;
    k->P[1][1] = P11_new;

    return k->angle;
}