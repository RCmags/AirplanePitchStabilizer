// DESCRIPTION: Use this sketch to calibrate the angle of attack sensor. 
/* Change the below parameters to find the analog outputs of the sensor. 
 * The in-built LED will turn on when the sensor matches the thresholds.
*/

// NOTE: this is intended for the ATTiny85 in the absense of a serial output.

/* Analog Read pinout:
 * Pin | Analog  
 * ============
 * PB2 |   1  
 * PB3 |   3
 * PB4 |   2
*/

// --- Parameters ---- 
/* Copy these values into the parameters file of the flight controller */
           
#define ANALOG_MAX        370     // Maximum target reading. Measure and record the sensor angle at this value
#define ANALOG_MIN        95      // Minimum target reading. Measure and record the sensor angle at this value 
#define ANALOG_DIFF       10       // Allowable error at middle position
#define ANALOG_OFFSET     0      // Bias to make the sensor read zero at the wing's zero-lift angle of attack

// depedant constants
constexpr int ANALOG_MID = 0.5*(ANALOG_MAX + ANALOG_MIN) + ANALOG_OFFSET;
constexpr int ANALOG_MID_P = ANALOG_MID + ANALOG_DIFF;
constexpr int ANALOG_MID_N = ANALOG_MID - ANALOG_DIFF; 

void setup() {
  // set pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB2, INPUT);
}

void loop() {
  int value = analogRead(1);    // PB2 -> ADC1

  if( (value < ANALOG_MIN || value > ANALOG_MAX) ||  
      (value < ANALOG_MID_P && value > ANALOG_MID_N) ) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

/* Note: AnalogRead can saturate at a different value when the code commands servos and reads PWM signals.	
         Use a voltage divider to scale the signal to a voltage that prevents analogRead from saturating. */
