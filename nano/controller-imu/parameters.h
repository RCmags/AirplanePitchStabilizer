/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
//                  1. Control inputs 
//----------------------------------------------------------
#define GAIN_PITCH        1.0                 // Change in target angle of attack due pitch commands. 
#define GAIN_ROLL        -0.75                // Asymmetric sweep, CH1

//----------------------------------------------------------
//                  2. Servo trims
//----------------------------------------------------------
#define TRIM_LEFT         0       // us       // left  wing 
#define TRIM_RIGHT        0       // us       // right wing

//----------------------------------------------------------
//                  3. Output PWM signal
//----------------------------------------------------------
#define PWM_MID           1500    // us       // Center/middle servo position
#define PWM_CHANGE        600     // us       // Maximum change in servo position

//----------------------------------------------------------
//                  4. Pitch Stabilization
//----------------------------------------------------------
  // I. PID Controller [negate to reverse]                       
#define GAIN_INT          1400.0              // Integral gain. Adjusts spring response
#define GAIN_PROP         800.0               // Proportional gain. Adjusts damping response
#define ANGLE_MAX         25      // deg      // Angle at which integral term saturates
  
  // II. Self levelling
#define GAIN_ANG_ROLL     100                 // Gain to set roll input to self-level aircraft
#define GAIN_ANG_PITCH   -180                 // Gain to set pitch rate to self-level aircraft 
#define ANGLE_TRIM       -9.5     // deg      // target flight angle relative to horizon
  
  // III. Maximum control rates                          
#define ACCEL_MIN         0.1     // g-force  // Minimum allowable wing load. Seen at max down elevator
#define ACCEL_MAX         3.0     // g-force  // Maximum allowable wing load. Seen at max up elevator
                                              /* NOTE: ACCEL_MIN < ACCEL_MAX */
#define VELOCITY          9.0     // m/s      // Average airspeed. A larger speed reduces the pitch rate 
#define GAIN_ACCEL        0.0                 // Load correction gain. Restricts maximum and minimum accelerations 
                                              /* GAIN_ACC must be positive */

//----------------------------------------------------------
//                  5. Input signal filter
//----------------------------------------------------------
#define INPUT_DEADBAND    24      // us       // deadband near center-stick to prevent integrator drift.
#define INPUT_CHANGE      18      // us       // Change in PWM signal needed to update receiver inputs

//----------------------------------------------------------
//                  6. Settings
//---------------------------------------------------------- 
#define NEGATE_ACCEL                          // Uncomment if sensor reads -1.0g when aircraft is upright (reversed z-axis). 
#define NEGATE_GYRO                           // Uncomment if pitch correction is backwards
#define USING_AUTO_LEVEL                      // Uncomment to enable self-leveling correction
#define USING_WEIGHT_SHIFT                    // Uncomment for weight-shift pitch control.  
                                              /* Enables gain that modifies pitch corrections with angle of attack */
//#define USING_MANUAL_CONTROL                  // Uncomment to dissable pitch stabilization and enable manual input

//----------------------------------------------------------
//                  7. Miscellaneous 
//---------------------------------------------------------- 
#define PIN_SERVO_LEFT    2
#define PIN_SERVO_RIGHT   3
#define NUMBER_MEAN       50                  // number of readings used for input calibration
#define GRAVITY           9.81    // m/^2     // acceleration due to gravity [m/s^2]
