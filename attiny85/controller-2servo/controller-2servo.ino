/* FLYING WING CONTROLLER */

/* ATTiny85 sketch to stabilize an RC flying wing controlled by weight-shift or trailing edge flaps. 
 * It commands two servos and applies V-tail mixing to stabilize the pitch axis.
 * Author: RCmags https://github.com/RCmags
 * 
 * NOTE: Code is intended to be used with ATTinyCore
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 0 -> Receiver CH1
// Pin 1 -> Receiver CH2
// Pin 2 -> middle of voltage divider 
  // Outputs:
// Pin 3 -> Left  wing servo
// Pin 4 -> Right wing servo 

/* VOLTAGE DIVIDER is required for analog input */
/* GND --/\/\/\-- MIDDLE --/\/\/\-- SENSOR_OUT  */
/*        100K              100K                */

//=================== Code ===================
#include <Servo_ATTinyCore.h>
#include "parameters.cpp"

//---- global variables 
Servo servo[2]; 
volatile uint16_t pwm_input[2] = {0};

//----- Input signals
// PORTB = {0 .. 5} -> using pins {0 .. 1} = B00000011

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
  // enable pin change interrupts
  GIMSK |= (1 << PCIE);     
  // set pins as PCINT
  PCMSK |= (1 << PCINT0);   
  PCMSK |= (1 << PCINT1);
  // set as input
  pinMode(PB0, INPUT_PULLUP);   
  pinMode(PB1, INPUT_PULLUP); ; 
}

//----- Input filter

/* remove noise from PWM inputs then scale and center */
void filterInputs(float* output) {
  static int filter[2] = {0};  
  
  // apply deadband filter
  for( uint8_t index = 0; index < 2; index += 1 ) {
    int change = int( pwm_input[index] ) - filter[index]; 
    filter[index] = change >  INPUT_CHANGE ? int(pwm_input[index]) - INPUT_CHANGE :
                    change < -INPUT_CHANGE ? int(pwm_input[index]) + INPUT_CHANGE : filter[index];
  }
  // scale and center inputs
  output[0] = float( filter[0] - PWM_MID )*GAIN_ROLL;    // roll
  output[1] = float( filter[1] - PWM_MID )*GAIN_PITCH;   // pitch
}

//----- Analog sensor

/* read sensor and calibrate to degrees */
float readSensor() {
  constexpr float SLOPE = float(ANGLE_MAX - ANGLE_MIN) / float(ANALOG_MAX - ANALOG_MIN);
  constexpr int   CONST = (ANALOG_MAX + ANALOG_MIN)/2 + ANALOG_OFFSET;
  // scale reading
  int value = analogRead(1) - CONST;      // PB2 -> ADC1 
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
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  servo[0].attach(PB3);
  servo[1].attach(PB4);
}

//----- Main loop

void setup() {
  setupISR();
  setupServos();
  pinMode(PB2, INPUT_PULLUP); // sensor
}

void loop() {
  // combine inputs
  float input[2]; filterInputs(input);

  #ifdef USING_MANUAL_CONTROL
    float output = input[1];                    // directly use rx input
  #else
    float output = PIDcontroller( -input[1] );  // pitch input controls target AoA
  #endif
  
  float mix1 = input[0] + output; 
  float mix2 = input[0] - output;
  
  // command servo
  mix1 += TRIM_LEFT;
  mix2 += TRIM_RIGHT;
  
  mix1 = constrain(mix1, -PWM_CHANGE, PWM_CHANGE);
  mix2 = constrain(mix2, -PWM_CHANGE, PWM_CHANGE)*GAIN_CORRECTION;

  servo[0].writeMicroseconds( PWM_MID + mix1 );    // left wing
  servo[1].writeMicroseconds( PWM_MID + mix2 );    // right wing 
}
