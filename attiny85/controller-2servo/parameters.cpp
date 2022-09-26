/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
// 1. Control inputs [negate to reverse direction]
#define GAIN_PITCH        0.5               // Change in target angle of attack due pitch commands. 
#define GAIN_ROLL         0.5               // Asymmetric sweep, CH1

// 2. Servo trims
#define TRIM_LEFT         0       // us     // left  wing 
#define TRIM_RIGHT        0       // us     // right wing

// 3. Output PWM signal
#define PWM_MID           1500    // us     // Center/middle servo position
#define PWM_MIN           1000    // us     // Minimum servo position
#define PWM_MAX           2000    // us     // Maximum servo position

// 4. Signal filters
#define INPUT_CHANGE      12      // us     // Change in PWM signal needed to update receiver inputs
#define ALPHA             0.05              /* Gain of alpha-beta filter applied to sensor input. 
                                               A smaller value smoothens the reading at the cost of slower response */
// 5. Pitch Stabilization
  // 1. PID Controller
#define GAIN_PROP         40.0              // Proportional gain. Adjusts spring response
#define GAIN_DERIV        0.0               // Derivative gain. Adjusts damping response
  // 2. Input restrictions
#define AOA_TRIM          8.0     // deg    // Target angle of attack in level flight 
#define AOA_MIN           1.0     // deg    // Minimum target angle of attack. Prevents wings unloading
#define AOA_MAX           15.0    // deg    // Maximum taget angle of attack. Prevents stall
                                            /* NOTE: AOA_MIN < AOA_TRIM < AOA_MAX */
// 6. Sensor calibrationg
#define ANGLE_MAX         95      // deg    // Largest deflection of angle sensor 
#define ANGLE_MIN         -95     // deg    // Smallest deflection of angle sensor
                                            /* NOTE: ANGLE_MIN < ANGLE_MAX */
#define ANALOG_MAX        750               // AnalogRead output at largest deflection
#define ANALOG_MIN        250               // AnalogRead output at smallest deflection
#define ANALOG_OFFSET     220               // Offset to align sensor with zero-lift angle of wing

// 7. Settings
//#define USING_WEIGHT_SHIFT                  // Uncomment for weight-shift pitch control.  
                                            /* Enables gain that modifies pitch corrections with angle of attack */ 
