//Description: Code to actively stabilize an airplane with elevons.
//             The angle of attack is measure with a potentiometer attached to a weathervane.        

//============ Libraries ============
#include <PinChangeInterrupt.h>
#include <Servo.h>


//============ Constants ============

  //-- Potentiometer (angle of attack) sensor:
const int POT_PIN               = A7;

    //Calibration:
const int POT_MID               = 500 + 10; // Increase offset for greater AoA - more pitch up [ Note: AoA  vane gets moved due to prop wash]
const int POT_CHANGE            = 400;
const int POT_DEADBAND          = 0;

    //Stabilization PID coefficients:
const float POT_GAIN_PROP       = -2.5;     // flight appears stable at higher settings [increase?]
const float POT_GAIN_DERIV      = -0.2;     //Induces wobble at speed
const float POT_GAIN_INT        = -0.0;     //Integral gain induces delayed control - No major stability improvement
const float POT_DECAY_INT       = 0.005;
const float RX_INT_DEADBAND     = 2;

  //-- Input and Output PWM signals:
const int PWM_CHANGE            = 550;
const int PWM_MID               = 1500;

  //-- Finite Impulse Response filter coefficients:
    //Updating data:
const int REFRESH_DELAY         = 10;
const float SENS_DECAY          = 0.4;
const int N_FILTER              = 2;
const float INFINITESIMAL       = float(REFRESH_DELAY)/1000.0;

    //Time Derivative:
const float COEFF_DERIV[]       = {1, 6, 14, 14, 0, -14, -14, -6, -1};       
const float DENOM_DERIV         = INFINITESIMAL * 128 ;     
const int N_DERIV               = sizeof( COEFF_DERIV )/sizeof( COEFF_DERIV[0] );
    
  //-- Miscellaneous:
    //Indeces:
const int PITCH = 1;
const int ROLL  = 0;
const int NOW   = 0;
const int PAST  = 1;

  //Servo output signals:
const int PIN_LEFT  = 4;
const int PIN_RIGHT = 5;
const int TRIM_LEFT = 0;
const int TRIM_RIGHT = 0;

  //-- Receiver PWM inputs:
    //Filtering:
const int RX_INPUT_DEADBAND     = 4;
const float RX_INPUT_DECAY      = 0.4;
const float RX_INPUT_GAIN[]     = {0.5 , 0.7};

    //Pins:
const int RX_INPUT_PINS[]       = {2, 3};
const int RX_N_INPUTS = sizeof( RX_INPUT_PINS )/sizeof( RX_INPUT_PINS[0] );

    //Input calibration:
const int RX_CALIB_PASSES       = 5;
const float RX_CALIB_COUNT      = 10;
const int RX_CALIB_DELAY        = 20;

      //Calibration blinking:
const int LED_DELAY             = 200;
const int BLINKS_BEGIN_CALIB    = 2;
const int BLINKS_END_CALIB      = 5;


//============ Variables ============

  //-- Receiver PWM signals:
volatile int pwm_raw[RX_N_INPUTS] = {0};
int pwm_input[RX_N_INPUTS] = {0};
int pwm_init[RX_N_INPUTS] = {0};

  //-- Potentiometer sensor:
int sensor_value[N_DERIV] = {0};
uint32_t time_last = millis();

  //-- Servo outputs:
Servo servoLeft, servoRight;


//============ Functions ============

  //-- Deadband functions:

    //Deadband with adjustable outputs:
float deadbandGeneral( float input, float band, float output_in, float output_out ) {
  if( input >= band ) {
    return output_out - band;
  } else if( input <= -band ) {
    return output_out + band;
  } else {
    return output_in;
  }
}

    //Deadband that defaults to zero or returns input:
float deadband( float input, float band ) {
  return deadbandGeneral( input, band, 0, input );
}

  //-- Receiver PWM signals:

    //ISR to calculate signal:
void checkPwmInput( void ) {
  
  static bool last_state[RX_N_INPUTS] = {0};
  static uint32_t curr_time[RX_N_INPUTS] = {0};

  for( int index = 0 ; index < RX_N_INPUTS ; index += 1 ) {
    
    if( digitalRead( RX_INPUT_PINS[index] ) == HIGH && last_state[index] == LOW ) {
      last_state[index] = HIGH;
      curr_time[index] = micros(); 
    }

    if( digitalRead( RX_INPUT_PINS[index] ) == LOW && last_state[index] == HIGH ) {
      last_state[index] = LOW;
      curr_time[index] = micros() - curr_time[index];
      int curr_input = curr_time[index] - PWM_MID - pwm_init[index];
      if( ( curr_input - pwm_raw[index] ) >= RX_INPUT_DEADBAND ) {
        pwm_raw[index] = curr_input - RX_INPUT_DEADBAND;  
      } else if ( (curr_input - pwm_raw[index] ) <= -RX_INPUT_DEADBAND ) {
        pwm_raw[index] = curr_input + RX_INPUT_DEADBAND;
      }
    }
  }
}

    //Centering raw inputs: 
void centerPwmInputs( void ) {

  for( int pass = 0; pass < RX_CALIB_PASSES; pass += 1 ) {
  
    int average[RX_N_INPUTS] = {0};

    for( int index = 0; index < RX_N_INPUTS; index += 1 ) {
      
      for( int count = 0; count < RX_CALIB_COUNT; count += 1 ) {
        delay(RX_CALIB_DELAY);
        average[index] += pwm_raw[index];  
      }
    }
  
    for( int index = 0; index < RX_N_INPUTS; index += 1 ) {
      pwm_init[index] += average[index]/RX_CALIB_COUNT;
    }
  }
}

  //-- Potentiometer Angle of attack sensor:

    //Store raw sensor values in array: 
void storeSensor( void ) {

  for( int index = N_DERIV - 1; index > 0; index -= 1 ) {
    sensor_value[index] = sensor_value[index - 1];
  }
}

    //Apply deadband filter and constrain values:
void filterSensorValues( void ) {
  static float output[N_FILTER] = {0};
  
  output[0] += ( float(analogRead(POT_PIN) - POT_MID) - output[0] )*SENS_DECAY;
  
  for( int index = 1; index < N_FILTER; index += 1 ) {
    output[index] += ( output[index-1] - output[index] )*SENS_DECAY;
  }
  
  sensor_value[NOW] = int( output[N_FILTER-1] );
}

    //FIR filter applied to sensor values:
float FirFilter( const float coeff[], const float denom, const int n_coeff ) {
  
  float output = 0;
  
  for( int index = 0; index < n_coeff ; index += 1 ) {
    output += float( sensor_value[index] ) * coeff[index];
  }
  return output/denom;
}

    //- PID stabilization:
    
      //Time derivative:
float sensorDeriv ( void ) {
  return FirFilter( COEFF_DERIV, DENOM_DERIV, N_DERIV );
}

      //Leaky Trapezoidal integration with exponential decay:
float sensorInt( float input ) {
  
  static float output = 0;

  float area_slice = ( sensor_value[NOW] + sensor_value[PAST] )/2.0 - deadband( pwm_input[PITCH] , RX_INT_DEADBAND );
  output += ( area_slice*POT_GAIN_INT - output )*POT_DECAY_INT; 
  output = constrain( output , -PWM_CHANGE - input , PWM_CHANGE - input );

  return output;
}

 //-- Miscellaneous:

  //Flash inbuilt LED:
void blinkLed( int n_times ) {  
  for( int index = 0; index < n_times; index += 1 ) {
    digitalWrite( LED_BUILTIN, HIGH );
    delay( LED_DELAY );
    digitalWrite( LED_BUILTIN, LOW );
    delay( LED_DELAY );
  }
}


//============ Main Functions ============

void setup() {

  //Enabling pin change interrupts for RX signals:
  for( int index = 0; index < RX_N_INPUTS ; index += 1 ) {
    pinMode( RX_INPUT_PINS[index] , INPUT_PULLUP );
    attachPinChangeInterrupt( digitalPinToPCINT( RX_INPUT_PINS[index] ), checkPwmInput , CHANGE );
  }

  //Setting pins:
  pinMode( POT_PIN, INPUT );
  pinMode( PIN_LEFT, OUTPUT );
  pinMode( PIN_RIGHT, OUTPUT );
  pinMode( LED_BUILTIN, OUTPUT );

  //Enabling servo outputs:
  servoLeft.attach(PIN_LEFT);
  servoRight.attach(PIN_RIGHT);

  //Calibrate RX signals:
  blinkLed( BLINKS_BEGIN_CALIB );
  centerPwmInputs();
  blinkLed( BLINKS_END_CALIB );  
}

void loop() {
  
  filterSensorValues();
  
  //Update every refresh delay:
  if( (millis() - time_last) > REFRESH_DELAY ) {
    time_last = millis();

    //Filter input signals:
    storeSensor();
    

    //Combine PID components:
    int pitch = sensor_value[NOW]*POT_GAIN_PROP + pwm_raw[PITCH]*RX_INPUT_GAIN[PITCH];
    pitch += sensorInt(pitch) + sensorDeriv()*POT_GAIN_DERIV;

    int roll_input = pwm_raw[ROLL]*RX_INPUT_GAIN[ROLL];

    //Set servo signals:
    servoLeft.writeMicroseconds( PWM_MID + constrain( TRIM_LEFT + pitch + roll_input, -PWM_CHANGE, PWM_CHANGE ) );
    servoRight.writeMicroseconds( PWM_MID + constrain( TRIM_RIGHT - pitch + roll_input, -PWM_CHANGE, PWM_CHANGE ) );
  }
}

