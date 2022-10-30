//=============== IMU sensor ===============
/* Configure gyroscope and accelerometer */
#include <basicMPU6050.h> 

// Gyro settings:
#define         LP_FILTER   6           // Low pass filter.                    Value from 0 to 6  -> 5hz cutoff
#define         GYRO_SENS   1           // Gyro sensitivity.                   Value from 0 to 3  -> +-250 deg/s
#define         ACCEL_SENS  1           // Accelerometer sensitivity.          Value from 0 to 3  -> +-4g
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.  

// Accelerometer offset:
constexpr int   AX_OFFSET = 0;       // Adjust these values so the accelerometer outputs (0,0,1)g if held level. 
constexpr int   AY_OFFSET = 0;       // The z-axis must be calibrated for the flight controller to work properly.
constexpr int   AZ_OFFSET = 0; 

// Set parameters:
basicMPU6050<LP_FILTER , GYRO_SENS ,  
             ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET , AY_OFFSET , AZ_OFFSET
             > imu;

//============= Sensor fusion ==============
/* IMU sensor fusion filter - horizontal estimate */
#include <imuFilter.h>
imuFilter fusion;

#define GAIN          0.1     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
                                 If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only) */

#define SD_ACCEL      0.2     /* Standard deviation of acceleration. Accelerations relative to (0,0,1)g outside of this band are suppresed.
                                 Accelerations within this band are used to update the orientation. [Measured in g-force] */                          

void setupFusion() {
  vec3_t accel = { imu.ax(), imu.ay(), imu.az() };
  #ifdef NEGATE_ACCEL
    accel = -accel;
  #endif
  fusion.setup( accel.x, accel.y, accel.z ); 
}
    
void updateFusion() {
  // measurements
  vec3_t angvel = { imu.gx(), imu.gy(), imu.gz() };
  
  vec3_t accel = { imu.ax(), imu.ay(), imu.az() };
  #ifdef NEGATE_ACCEL
    accel = -accel;
  #endif
  
  // remove centrifucal acceleration
  const vec3_t vel = vec3_t(0, VELOCITY, 0);
  vec3_t acc_cent = vel.cross(angvel);

  constexpr float INV_GRAVITY = 1.0 / GRAVITY;
  acc_cent *= INV_GRAVITY; 
  accel -= acc_cent;
  
  fusion.update( angvel.x, angvel.y, angvel.z, 
                 accel.x, accel.y, accel.z, 
                 GAIN, SD_ACCEL );  
}
