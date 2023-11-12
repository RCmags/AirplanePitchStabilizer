//============= Sensor fusion ==============
/* IMU sensor fusion filter - horizontal estimate */
#include <imuFilter.h>

//----------------------- Settings -------------------------

#define GAIN          0.1     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector */

#define SD_ACCEL      0.2     /* Standard deviation of acceleration. Accelerations relative to (0,0,1)g outside of this band are suppresed */                          

//----------------------------------------------------------
imuFilter fusion;

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
