/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION *s/
/*=========================================================*/
//                  1. Control input
//---------------------------------------------------------- 
#define GAIN_PITCH        0.5               // Change in target angle of attack due pitch commands 

//----------------------------------------------------------
//                  2. Servo trim
//---------------------------------------------------------- 
#define TRIM              0       // us     // wing servo 
                                            /* adjust so the aircraft does not pitch in level flight */

//----------------------------------------------------------
//                  3. Output PWM signal
//----------------------------------------------------------
#define PWM_MID           1500    // us     // Center/middle servo position
#define PWM_CHANGE        500     // us     // Maximum change in servo position relative to middle

// 4. Signal filters
#define INPUT_CHANGE      12      // us     // Change in PWM signal needed to update receiver inputs
#define ALPHA             0.05              /* Gain of alpha-beta filter applied to sensor input. 
                                               A smaller value smoothens the reading at the cost of slower response */

//----------------------------------------------------------
//                  5. Pitch Stabilization
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
//                  6. Sensor calibration
//----------------------------------------------------------
#define ANGLE_MAX         15      // deg    // Largest deflection of angle sensor 
#define ANGLE_MIN        -15      // deg    // Smallest deflection of angle sensor
                                            /* NOTE: ANGLE_MIN < ANGLE_MAX */
#define ANALOG_MAX        512               // AnalogRead output at largest deflection
#define ANALOG_MIN        0                 // AnalogRead output at smallest deflection
#define ANALOG_OFFSET     0                 // Offset to make the sensor read zero at the wing's zero-lift angle of attack

//----------------------------------------------------------
//                  7. Settings
//----------------------------------------------------------
//#define USING_WEIGHT_SHIFT                  // Uncomment for weight-shift pitch control.  
                                            /* Enables gain that modifies pitch corrections with angle of attack */ 
//#define USING_MANUAL_CONTROL                // Uncomment to dissable pitch stabilization and enable manual inputs
