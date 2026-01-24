#ifndef __SERVO_CONFIG_H
#define __SERVO_CONFIG_H
/**
 * @file servo_config.h
 * @brief Configuration parameters for the FB5311M Servo Driver
*/
#define SERVO_MIN_ANGLE         -180.0f    // Minimum angle in degrees
#define SERVO_MAX_VOLTAGE       1.65f    // Maximum voltage in volts
#define REF_NUCLEO_VOLTAGE       3.3f    // Minimum voltage in volts
#define SERVO_MAX_ANGLE        180.0f  // Maximum angle in degrees
#define SERVO_PWM_FREQUENCY     50      // PWM frequency in Hz
#define SERVO_KP          25.0f    // Proportional gain for control  
#define SERVO_KI          5.0f   // Integral gain for control
#define SERVO_KD          0.1f  // Derivative gain for control
#define SERVO_FILTER_Q_ANGLE    0.1f    // Kalman filter process noise for angle
#define SERVO_FILTER_Q_VEL      0.1f    // Kalman filter process noise for velocity
#define SERVO_FILTER_R_MEAS     0.01f    // Kalman filter measurement noise
#define PWM_CENTER             1500    // Center PWM pulse width in microseconds
#define SERVO_CONTROL_FREQUENCY  100.0f  // Control loop frequency in Hz
#define INTEGRAL_WINDUP_GUARD  1000.0f   // Integral windup guard value 
#define OUTPUT_LIMIT          2000.0f // Maximum output limit
#define ADC_RESOLUTION 4095.0f  // 12-bit ADC
#define ADC_BUFFER_SIZE 20    // Size of the ADC buffer
#define MAX_PWM_PULSE_WIDTH 2000  // Maximum PWM pulse width in microseconds
#define MIN_PWM_PULSE_WIDTH 500   // Minimum PWM pulse width in microseconds

#endif /* __SERVO_CONFIG_H */