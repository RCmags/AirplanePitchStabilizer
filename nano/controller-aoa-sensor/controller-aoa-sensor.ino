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
// Pin 6  -> Left  wing servo
// Pin 7  -> Right wing servo 

//=================== Code ===================
#include <Servo.h>
#include "parameters.cpp"

//----- constants
#define SENSOR_PIN    A7

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
  pinMode(9, INPUT_PULLUP); ; 
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
  float value = analogRead(SENSOR_PIN) - CONST;     
  return SLOPE*value;
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
  constexpr float SCALE = 5.0 / 500.0;     // Convert 500 ms to 5 degrees
  
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
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  servo[0].attach(6);
  servo[1].attach(7);
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
  
  float output = PIDcontroller( -input[1] );  // pitch input controls target AoA
  float mix1 = input[0] + output;
  float mix2 = input[0] - output;
  
  // command servos
  mix1 += PWM_MID + TRIM_LEFT;
  mix2 += PWM_MID + TRIM_RIGHT;
  
  mix1 = constrain(mix1, PWM_MIN, PWM_MAX );
  mix2 = constrain(mix2, PWM_MIN, PWM_MAX );

  servo[0].writeMicroseconds( mix1 );    // left wing
  servo[1].writeMicroseconds( mix2 );    // right wing
}
