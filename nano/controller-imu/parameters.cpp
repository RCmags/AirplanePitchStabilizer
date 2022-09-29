/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
// 1. Control inputs [negate to reverse direction]
#define GAIN_PITCH        1.0                 // Change in target angle of attack due pitch commands. 
#define GAIN_ROLL         0.5                 // Asymmetric sweep, CH1

// 2. Servo trims
#define TRIM_LEFT         0       // us       // left  wing 
#define TRIM_RIGHT        0       // us       // right wing

// 3. Output PWM signal
#define PWM_MID           1500    // us       // Center/middle servo position
#define PWM_MIN           1000    // us       // Minimum servo position
#define PWM_MAX           2000    // us       // Maximum servo position
                                            
// 4. Pitch Stabilization
  // 1. PID Controller                        NOTE: Proportional on angular velocity
#define GAIN_INT          1000.0              // Integral gain. Adjusts spring response
#define GAIN_PROP         50.0                // Proportional gain. Adjusts damping response
#define ANGLE_MAX         15      // deg      // Angle at which integral term saturates
  // 2. target rate
#define ACCEL_MIN         0.2     // g-force  // Minimum allowable wing load. Seen at max down elevator
#define ACCEL_MAX         3.0     // g-force  // Maximum allowable wing load. Seen at max up elevator
#define VELOCITY          9.0     // m/s      // Average airspeed. A larger speed reduces the pitch rate
#define GAIN_ACCEL        0.0                 // Load correction gain. Restricts maximum and minimum accelerations

// 5. Input signal filter
#define INPUT_CHANGE      40      // us       // deadband near center-stick to prevent integrator drift.

// 6. Settings
#define USING_WEIGHT_SHIFT                    // Uncomment for weight-shift pitch control.  
                                              /* Enables gain that modifies pitch corrections with angle of attack */ 
