/* FLYING WING CONTROLLER */

/* Arduino NANO sketch to stabilize an RC flying wing controlled by weight-shift or trailing edge flaps. 
 * It commands two servos and applies V-tail mixing to stabilize the pitch axis.
 * Author: RCmags https://github.com/RCmags
 * 
 * NOTE: 
 * This sketch uses an MPU6050 IMU (gyroscope and accelerometer) to stabilize pitch. It assumes that only 
 * lift and gravity act on the aircraft, and that any external torques are negligible. 
 * 
 * This works for some aircraft, but for a general solution, use the version of the controller that uses 
 * an angle-of-attack sensor. 
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 8  -> Receiver CH1
// Pin 9  -> Receiver CH2
// Pin A5 -> Sensor SCL
// Pin A4 -> Sensor SDA
  // Outputs:
// Pin 4  -> Left  wing servo
// Pin 5  -> Right wing servo 

//=================== Code ===================
#include <Servo.h>
#include "imu.cpp"
#include "parameters.cpp"

//----- constants
#define NUMBER_MEAN    50      // number of readings used for calibration mean
#define GRAVITY        9.81    // acceleration due to gravity [m/s^2]

constexpr float ANGVEL_SCALE = GRAVITY / VELOCITY;   
constexpr float ANGVEL_MIN   = (ACCEL_MIN - 1) * ANGVEL_SCALE;
constexpr float ANGVEL_MAX   = (ACCEL_MAX - 1) * ANGVEL_SCALE; 

//----- global variables 
Servo servo[2]; 
volatile uint16_t pwm_input[2] = {0};
int               pwm_mean[2]  = {PWM_MID};

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

/* average of rx signals */
void calibrateInputs() {
  uint32_t sum[2] = {0};
  for( uint8_t count = 0; count < NUMBER_MEAN; count += 1 ) {
    sum[0] += pwm_input[0];
    sum[1] += pwm_input[1];
    delay(20);                  // period of 50hz pwm signal
  } 
  pwm_mean[0] = int(sum[0] / NUMBER_MEAN);
  pwm_mean[1] = int(sum[1] / NUMBER_MEAN);
}

/* scale and center inputs */
void scaleInputs(float* output) {
  output[0] = float( int(pwm_input[0]) - pwm_mean[0] )*GAIN_ROLL;    // roll
  output[1] = float( int(pwm_input[1]) - pwm_mean[1] )*GAIN_PITCH;   // pitch
}

//----- PID controller

#ifdef USING_WEIGHT_SHIFT

/* gain to maintain constant torque indifferent of wing load */
float gainAccel(float accel) {
  return accel == 0 ? 0 : 1/accel;
}
#endif

/* time integral of variable */
float integral(float input) {
  static uint32_t last_time = micros();
  static float value[2] = {0};

  // update timer
  uint32_t curr_time = micros();
  float dt = float(curr_time - last_time)*1e-6;
  last_time = curr_time;

  // trapezoidal rule
  constexpr float MAX = DEG_TO_RAD * ANGLE_MAX;

  value[0] += 0.5*( input + value[1] )*dt;
  value[1] = input;

  // prevent windup
  value[0] = constrain(value[0], -MAX, MAX);
  return value[0];
}

/* make variable zero within bounds */
float deadband(float input, const float MIN, const float MAX) {
  return input > MAX ? input - MAX :
         input < MIN ? input - MIN : 0;  
}

/* restrict target rate to restrict wing loading */
float restrictRate(float input, float accel) {
  // limit target angular velocity
  constexpr float SCALE = (50 * DEG_TO_RAD) / 500.0;     // Convert 500 ms to 50 deg/s

  input = deadband(input, -INPUT_CHANGE, INPUT_CHANGE);
  input *= SCALE;
  input = constrain(input, ANGVEL_MIN, ANGVEL_MAX); 

  // reduce rate at acceleration limits
  accel = deadband(accel, ACCEL_MIN, ACCEL_MAX);
  return input - float(GAIN_ACCEL)*accel;
}

/* PID controller to stabilize pitch axis */
float PIDcontroller(float input) {
  // target state
  float accel = imu.az();                 // z-component aligns with lift
  float angvel_t = restrictRate(input, accel);
  
  // controller
  float angvel = imu.gx() - angvel_t;     
  float angle = integral(angvel);
    // use P and I terms
  float output = float(GAIN_INT)*angle + float(GAIN_PROP)*angvel;

  #ifdef USING_WEIGHT_SHIFT
    return output * gainAccel(accel); 
  #else
    return output;
  #endif
}

//----- Servos

void setupServos() {    
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  servo[0].attach(4);
  servo[1].attach(5);
  // default position
  servo[0].writeMicroseconds(PWM_MID);
  servo[1].writeMicroseconds(PWM_MID);
}

//----- Main loop

void setup() {
  setupISR();
  setupServos();
  // sensors
  imu.setup();
  imu.setBias();
  // receiver signals
  calibrateInputs();
}

void loop() {
  // update gyro calibration 
  imu.updateBias();
  
  // combine inputs
  float input[2]; scaleInputs(input);
  
  float output = PIDcontroller( -input[1] );  
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
