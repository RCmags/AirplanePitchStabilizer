/* FLYING WING CONTROLLER */

/* Arduino NANO sketch to stabilize an RC flying wing controlled by weight-shift or trailing edge flaps. 
 * It commands two servos and applies V-tail mixing to stabilize the pitch axis.
 * Author: RCmags https://github.com/RCmags
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 8 -> Receiver CH1
// Pin 9 -> Receiver CH2
// Pin A7 -> Analog sensor
  // Outputs:
// Pin 2  -> Left  wing servo
// Pin 3  -> Right wing servo 

//=================== Code ===================
#include <Servo.h>
#include "parameters.cpp"

//----- constants
constexpr int PWM_MID_L = PWM_MID + TRIM_LEFT;
constexpr int PWM_MID_R = PWM_MID - TRIM_RIGHT;

//----- global variables 
Servo servo[2]; 
volatile uint16_t pwm_input[2] = {0};

//----- Input signals
// PORTB = {8 .. 13} -> using pins {8 .. 9} = B00000011

/* port change interrupt to read PWM inputs from receiver */
ISR( PCINT0_vect ) {
  static uint32_t initial_time[2] = {0}; 
  static uint8_t port_last = PINB;  
  
  // port changes
  uint32_t current_time = micros(); 
  uint8_t port_rise = ~port_last & PINB;
  uint8_t port_fall = port_last & ~PINB;
  
  // find changing pins
  for( uint8_t index = 0; index < 2; index += 1) {
    uint8_t mask = B00000001 << index;  // Start at PCINT0
    if( port_rise & mask ) {                
        initial_time[index] = current_time;
    } else if ( port_fall & mask ) {       
        pwm_input[index] = current_time - initial_time[index];
    }
  }
  port_last = PINB;    
}

void setupISR() {
  // enable PORTB interrupts  
  PCICR |= (1 << PCIE0);
  // enable PCINT for pins 8-9     
  PCMSK0 |= (1 << PCINT0);   
  PCMSK0 |= (1 << PCINT1);
  // set as input
  pinMode(8, INPUT_PULLUP);   
  pinMode(9, INPUT_PULLUP);
}

//----- Input filter

/* scale and center inputs */
void scaleInputs(float* output) {
  output[0] = float( int(pwm_input[0]) - PWM_MID )*GAIN_ROLL;    // roll
  output[1] = float( int(pwm_input[1]) - PWM_MID )*GAIN_PITCH;   // pitch
}

//----- Analog sensor

/* read sensor and calibrate to degrees */
float readSensor() {
  constexpr float SLOPE = float(ANGLE_MAX - ANGLE_MIN) / float(ANALOG_MAX - ANALOG_MIN);
  constexpr int   CONST = float(ANALOG_MAX + ANALOG_MIN)/2 + ANALOG_OFFSET;
  // scale reading
  int value = analogRead(SENSOR_PIN) - CONST;     
  return SLOPE*float(value);
}

/* remove noise from analog sensor and calculate derivative */
void filterSensor(float *xbar, float *dxdt) {
  // store state
  static uint32_t last_time = micros();
  static float x[2] = {0}, v = 0;

  // time change
  uint32_t curr_time = micros();
  float dt  = float(curr_time - last_time)*1e-6;
  last_time = curr_time; 

  // alpha-beta filter:
  constexpr float BETA = ALPHA * ALPHA * 0.25;
    // error
  float xi = readSensor();
  float dx = 0.5*( xi - x[1] );
    // position
  float xnew = x[0];
  xnew += v*dt;  
  xnew += ALPHA * dx; 
    // velocity
  v += BETA * dx/dt;

  // store past position
  x[1] = x[0];
  x[0] = xnew;

  // return smoothed value and derivative
  *xbar = xnew;
  *dxdt = v;
}

//----- PID controller

#ifdef USING_WEIGHT_SHIFT

/* gain to maintain constant torque indifferent of angle of attack */
float gainAngle(float angle) {
  return angle == 0 ? 0 : AOA_TRIM/angle;
}
#endif

/* PID controller to stabilize pitch axis */
float PIDcontroller(float input) {
  float angle; float angle_deriv; filterSensor(&angle, &angle_deriv);
  
  // desired angle of attack
  constexpr float SCALE = 10.0 / 500.0;     // Convert 500 ms to 10 degrees
  
  float targ_angle = SCALE*input + AOA_TRIM;
  targ_angle = constrain(targ_angle, AOA_MIN, AOA_MAX);
  
  // use P and D terms
  float output = GAIN_PROP*(angle - targ_angle) + GAIN_DERIV*angle_deriv;
  
  #ifdef USING_WEIGHT_SHIFT
    return output * gainAngle(angle); 
  #else
    return output;
  #endif
}

//----- Servos

void setupServos() {    
  pinMode(PIN_SERVO_LEFT, OUTPUT);
  pinMode(PIN_SERVO_RIGHT, OUTPUT);
  servo[0].attach(PIN_SERVO_LEFT);
  servo[1].attach(PIN_SERVO_RIGHT);
  // default position
  servo[0].writeMicroseconds(PWM_MID_L);
  servo[1].writeMicroseconds(PWM_MID_R);
}

//----- Main loop

void setup() {
  setupISR();
  setupServos();
  pinMode(SENSOR_PIN, INPUT_PULLUP); // sensor
}

void loop() {
  // combine inputs
  float input[2]; scaleInputs(input);
  
  #ifdef USING_MANUAL_CONTROL
    float output = input[1];                    // directly use rx input
  #else
    float output = PIDcontroller( -input[1] );  // pitch input controls target AoA
  #endif
  
  float mix1 = input[0] + output;
  float mix2 = input[0] - output;
  
  // command servos  
  mix1 = constrain(mix1,-PWM_CHANGE, PWM_CHANGE);
  mix2 = constrain(mix2,-PWM_CHANGE, PWM_CHANGE);

  servo[0].writeMicroseconds( PWM_MID_L + mix1 );    // left wing
  servo[1].writeMicroseconds( PWM_MID_R + mix2 );    // right wing
}
