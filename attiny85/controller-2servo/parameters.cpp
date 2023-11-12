/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
//                  1. Control inputs
//----------------------------------------------------------
#define GAIN_PITCH       -1.0               // Change in target angle of attack due pitch commands 
#define GAIN_ROLL         0.65               // Asymmetric sweep/ailerons

//----------------------------------------------------------
//                  2. Servo trims
//----------------------------------------------------------
#define TRIM_LEFT         0       // us     // left  wing 
#define TRIM_RIGHT        0       // us     // right wing
                                            /* Adjust so the aircraft maintains altitude without rolling or pitching */

//----------------------------------------------------------
//                  3. Output PWM signal
//----------------------------------------------------------
#define PWM_MID           1500    // us     // Center/middle servo position
#define PWM_CHANGE        800     // us     // Maximum change in servo position relative to middle
#define PWM_CHANGE_PITCH  800     // us     // Maximum change in servo position for pitch 
#define GAIN_CORRECTION   0.6               // Correction factor to make servos move symmetrically

//----------------------------------------------------------
//                  4. Signal filters
//----------------------------------------------------------
#define INPUT_CHANGE      12      // us     // Change in PWM signal needed to update receiver inputs
#define ALPHA             0.15              /* Gain of alpha-beta filter applied to sensor input. 
                                               A smaller value smoothens the reading at the cost of slower response */
//----------------------------------------------------------
//                  5. Pitch Stabilization
//----------------------------------------------------------
  // I. PID Controller [negate to reverse] 
#define GAIN_PROP         90.0              // Proportional gain. Adjusts spring response
#define GAIN_DERIV        1.0               // Derivative gain. Adjusts damping response. Must have same sign as GAIN_PROP
  
  // II. Input restrictions
#define AOA_TRIM          1.5     // deg    // Target angle of attack in level flight 
#define AOA_MIN          -15.0    // deg    // Minimum target angle of attack. Prevents wings unloading
#define AOA_MAX           15.0    // deg    // Maximum taget angle of attack. Prevents stall
                                            /* NOTE: AOA_MIN < AOA_TRIM < AOA_MAX */

//----------------------------------------------------------
//                  6. Sensor calibrationg
//----------------------------------------------------------
#define ANGLE_MAX         15      // deg    // Largest deflection of angle sensor 
#define ANGLE_MIN        -15      // deg    // Smallest deflection of angle sensor
                                            /* NOTE: ANGLE_MIN < ANGLE_MAX */
#define ANALOG_MAX        370               // AnalogRead output at largest deflection
#define ANALOG_MIN        95                // AnalogRead output at smallest deflection
#define ANALOG_OFFSET     0                // Offset to make the sensor read zero at the wing's zero-lift angle of attack

//----------------------------------------------------------
//                  7. Settings
//----------------------------------------------------------
//#define USING_WEIGHT_SHIFT                  // Uncomment for weight-shift pitch control.  
                                            /* Enables gain that modifies pitch corrections with angle of attack */ 
//#define USING_MANUAL_CONTROL                // Uncomment to dissable pitch stabilization and enable manual input
