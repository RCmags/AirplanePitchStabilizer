/* Configure gyroscope and accelerometer */
#include <basicMPU6050.h> 

// Gyro settings:
#define         LP_FILTER   6           // Low pass filter.                    Value from 0 to 6  -> 5hz cutoff
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3  -> +-250 deg/s
#define         ACCEL_SENS  1           // Accelerometer sensitivity.          Value from 0 to 3  -> +-4g
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET =  552;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -241;       // The accelerometer must be calibrated for the flight controller to work properly.
constexpr int   AZ_OFFSET = -3185; 

// Set parameters:
basicMPU6050<LP_FILTER , GYRO_SENS ,  
             ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET , AY_OFFSET , AZ_OFFSET
             > imu;
