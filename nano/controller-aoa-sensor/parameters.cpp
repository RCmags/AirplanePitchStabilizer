/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
//                  1. Control inputs 
//----------------------------------------------------------
#define GAIN_PITCH        0.5               // Change in target angle of attack due pitch commands. 
#define GAIN_ROLL         0.5               // Asymmetric sweep/ailerons

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
#define PWM_CHANGE        500     // us     // Maximum change in servo position
                                            
//----------------------------------------------------------
//                  4. Pitch Stabilization
//----------------------------------------------------------
  // I. PID Controller [negate to reverse]
#define GAIN_PROP         40.0              // Proportional gain. Adjusts spring response
#define GAIN_DERIV        0.0               // Derivative gain. Adjusts damping response. Must have same sign as GAIN_PROP
  
  // II. Input restrictions
#define AOA_TRIM          8.0     // deg    // Target angle of attack in level flight 
#define AOA_MIN           1.0     // deg    // Minimum target angle of attack. Prevents wings unloading
#define AOA_MAX           15.0    // deg    // Maximum taget angle of attack. Prevents stall
                                            /* NOTE: AOA_MIN < AOA_TRIM < AOA_MAX */
                                            
//----------------------------------------------------------
//                  5. Sensor filter
//----------------------------------------------------------
#define ALPHA             0.05              /* Gain of alpha-beta filter applied to sensor input. 
                                            A smaller value smoothens the reading at the cost of slower response */

//----------------------------------------------------------
//                  6. Sensor calibration
//---------------------------------------------------------- 
#define ANGLE_MAX         90      // deg    // Largest deflection of angle sensor 
#define ANGLE_MIN        -90      // deg    // Smallest deflection of angle sensor
                                            /* NOTE: ANGLE_MIN < ANGLE_MAX */
#define ANALOG_MAX        870               // AnalogRead output at largest deflection
#define ANALOG_MIN        180               // AnalogRead output at smallest deflection
#define ANALOG_OFFSET     0                 // Offset to align sensor with zero-lift angle of wing

//----------------------------------------------------------
//                  7. Settings
//---------------------------------------------------------- 
//#define USING_WEIGHT_SHIFT                  // Uncomment for weight-shift pitch control.  
                                            /* Enables gain that modifies pitch corrections with angle of attack */ 
//#define USING_MANUAL_CONTROL                // Uncomment to dissable pitch stabilization and enable manual input

//----------------------------------------------------------
//                  8. Miscellaneous 
//----------------------------------------------------------
#define SENSOR_PIN        A7
#define PIN_SERVO_LEFT    2
#define PIN_SERVO_RIGHT   3
