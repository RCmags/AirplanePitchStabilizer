/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
// 1. Control inputs [negate to reverse]
#define GAIN_PITCH        1.0                 // Change in target angle of attack due pitch commands. 
#define GAIN_ROLL        -0.7                 // Asymmetric sweep, CH1

// 2. Servo trims
#define TRIM_LEFT         0       // us       // left  wing 
#define TRIM_RIGHT       -0       // us       // right wing

// 3. Output PWM signal
#define PWM_MID           1500    // us       // Center/middle servo position
#define PWM_CHANGE        620     // us       // Maximum change in servo position
 
// 4. Pitch Stabilization
  // 1. PID Controller [negate to reverse]                       
#define GAIN_INT          1200.0              // Integral gain. Adjusts spring response
#define GAIN_PROP         800.0               // Proportional gain. Adjusts damping response
#define ANGLE_MAX         25      // deg      // Angle at which integral term saturates
  // 2. target rate                          
#define ACCEL_MIN         0.2     // g-force  // Minimum allowable wing load. Seen at max down elevator
#define ACCEL_MAX         3.0     // g-force  // Maximum allowable wing load. Seen at max up elevator
                                              /* NOTE: ACCEL_MIN < ACCEL_MAX */
#define VELOCITY          9.0     // m/s      // Average airspeed. A larger speed reduces the pitch rate 
#define GAIN_ACCEL        0.0                 // Load correction gain. Restricts maximum and minimum accelerations 
                                              /* GAIN_ACC must be positive */
// 5. Input signal filter
#define INPUT_DEADBAND    50      // us       // deadband near center-stick to prevent integrator drift.
#define INPUT_CHANGE      12      // us       // Change in PWM signal needed to update receiver inputs

// 6. Settings
#define NEGATE_ACCEL                          // Uncomment if sensor reads -1.0g when aircraft is upright (reversed z-axis). 
#define NEGATE_GYRO                           // Uncomment if pitch correction is backwards
#define USING_WEIGHT_SHIFT                    // Uncomment for weight-shift pitch control.  
                                              /* Enables gain that modifies pitch corrections with angle of attack */
