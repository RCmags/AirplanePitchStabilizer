/* Analog Read pinout
 * Pin | Analog  
 * PB2 |   1  
 * PB3 |   3
 * PB4 |   2
*/

#define ANALOG_MAX    750
#define ANALOG_MIN    250
#define ANALOG_DIFF   10

constexpr int ANALOG_MID = 0.5*(ANALOG_MAX + ANALOG_MIN);
constexpr int ANALOG_MID_P = ANALOG_MID + ANALOG_DIFF;
constexpr int ANALOG_MID_N = ANALOG_MID - ANALOG_DIFF; 

void setup() {
  // set pins used in controller
  pinMode(PB0, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  // set pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PB2, INPUT);
}

void loop() {
  int value = analogRead(1);

  if( (value < ANALOG_MIN   || value > ANALOG_MAX) ||  
      (value < ANALOG_MID_P && value > ANALOG_MID_N ) ) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


// Note: AnalogRead outputs different values when code does not command servos and read PWM signals.
