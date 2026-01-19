/*
 * pid_control.c
 */

#include "STM32_F4xx_PID.h"
#include "stm32f4xx_hal.h"

// Helper function to normalize angle to [-180, 180]
static float constrainAngle(float x) {
    x = fmod(x + 180, 360);
    if (x < 0) x += 360;
    return x - 180;
}

void PID_Init(ServoPID *pid, float kp, float ki, float kd, float i_lim, float out_lim) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integralLimit = i_lim;
    pid->outputLimit = out_lim;
    pid->integralAccumulator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->deadband = 20.0f; // As per datasheet spec
    pid->lastUpdateTime = HAL_GetTick();
    pid->minDt = 0.001f;   // 1ms minimum
    pid->maxDt = 0.1f;     // 100ms maximum (prevents large jumps after pause)
}

void PID_Reset(ServoPID *pid, float current_pos) {
    pid->integralAccumulator = 0.0f;
    pid->prevMeasurement = current_pos;
    pid->lastUpdateTime = HAL_GetTick();
}

float PID_Compute(ServoPID *pid, float setpoint, float measurement) {
    // 0. Calculate adaptive dt from elapsed time
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - pid->lastUpdateTime) / 1000.0f; // Convert ms to seconds
    // clamp 0 
    if (setpoint <= 0.0f && setpoint >= -0.0f) {
        setpoint=0.5f;
    }
    // Clamp dt to reasonable bounds
    if (dt < pid->minDt) dt = pid->minDt;
    if (dt > pid->maxDt) dt = pid->maxDt;
    
    pid->lastUpdateTime = now;
    // 1. Calculate Error with Wrap-Around Logic
    float error = setpoint - measurement;
    error = constrainAngle(error); 

    // 2. Proportional Term
    float P = pid->Kp * error;

    // 3. Integral Term with Anti-Windup Clamping
    pid->integralAccumulator += error * dt;
    
    // Clamp the integrator
    if (pid->integralAccumulator > pid->integralLimit) 
        pid->integralAccumulator = pid->integralLimit;
    if (pid->integralAccumulator < -pid->integralLimit) 
        pid->integralAccumulator = -pid->integralLimit;
        
    float I = pid->Ki * pid->integralAccumulator;

    // 4. Derivative Term on Measurement (Kick Avoidance) 
    // Calculate change in measurement, handling wrap-around
    float deltaMeasurement = measurement - pid->prevMeasurement;
    deltaMeasurement = constrainAngle(deltaMeasurement); 
    
    // Note negative sign: D-term resists motion
    float D = -pid->Kd * (deltaMeasurement / dt);

    // 5. Compute Total Control Effort
    float output = P + I + D;

    // 6. Deadband Compensation
    // If output is too low to move motor, boost it or zero it
    // The servo deadband is approx +/- 20us. 
    // If the PID wants to move (output!= 0), we must step outside the deadband.
    if (output > 0.1f) output += pid->deadband;
    if (output < -0.1f) output -= pid->deadband;

    // 7. Output Saturation
    if (output > pid->outputLimit) output = pid->outputLimit;
    if (output < -pid->outputLimit) output = -pid->outputLimit;

    // 8. Store state for next iteration
    pid->prevMeasurement = measurement;

    return output;
}