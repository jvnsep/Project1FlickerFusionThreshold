#include <Arduino.h>
/* 
  ***************************************************************************************************
  ** Project 1: Flicker Fusion Threshold                                                           **
  ***************************************************************************************************
  This is an embedded system project which measure flicker fusion threshold.
  The measure of frequency on how much human perception of LED blinking.

  The circuit:
    1. Teensy 3.1/3.2 Microcontroller
    2. 2pcs. 360Ω resistor in parallel connected from Pin 12 to LED
    3. this LED then conneted to ground
    4. 10kΩ potentiometer with wiper connected from Pin 14 or A0 
       and other two terminal from 3.3V to ground  
    5. a push button switch connected from Pin 11 to ground 

  Created: 19 April 2019 by Joven Sepulveda 
            Weltec School of Engineering
            MG7013 Embedded Systems
*/

// constant set pin numbers:
const uint8_t LEDPIN = 12;                   // output pin for the LED
const uint8_t POTENTIOMETERPIN = A0;         // input pin for the potentiometer
const uint8_t BUTTONPIN = 11;                // input pin for the push button

// variables of changing values
uint16_t adcValue = 0;                      // variable storage for ADC value from potentiometer
float frequencyValue = 0;                   // variable storage for calculated value of frequency

// forward declaration of function
void pushbutton();                          // function for push button
void blinkLED();                            // function for LED blinking

// initialize serial monitor, analog resoution, pin modes and interrupt function
void setup() {
  Serial.begin(500000);                     // sets serial data rate to 500,000 bps
  analogReadResolution(16);                 // sets resolution 16 bits, so analogRead() be 0 to 65536
  pinMode(LEDPIN, OUTPUT);                  // declare the LEDPIN as an OUTPUT
  pinMode(BUTTONPIN, INPUT_PULLUP);         // declare the BUTTONPIN as an INPUT PULLUP
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), pushbutton, FALLING); // push button external interrupts
}

// loop checks analog value and will send output signal for LED blink
void loop() {
  adcValue = analogRead(POTENTIOMETERPIN);   // read the adcvalue from the potentiometer:
  blinkLED();                                // call function blinkLED
}

// 50% duty cycle LED blink and calculate frequency
void blinkLED(){
  static uint32_t previousTime_micro = 0;    // first call 0 for initial value for previousTime_micro
  if ((micros() - previousTime_micro) >= (adcValue)){ // if elapse time is equal or greater than adcValue
    // LED change state
    if (digitalRead(LEDPIN) == LOW){         // if LED is off
      digitalWrite(LEDPIN, HIGH);            // switch LED on
    } else {                                 // else if LED on
      digitalWrite(LEDPIN, LOW);             // switch LED off
    }  
    frequencyValue = (float) 1000000/((micros() - previousTime_micro)*2); // calculate frequency of LED blink
    previousTime_micro = micros();           // reset previousTime_micro
  }
}

// function for attachinterrupt at instant push button and allow debounce delay
void pushbutton(){
  noInterrupts();                           // disables external interrupts
  const uint32_t DEBOUNCEDELAY = 100;       // constant debouce delay of 100ms 
  static uint32_t previousTime_ms = 0;      // first call variable 
  static bool lastbuttonstate = LOW;        // previous state from push button
  // If state is due to noise or press
  if (digitalRead(BUTTONPIN) != lastbuttonstate){ 
    previousTime_ms = millis();             // reset the previous time
  }
  // if elapse time is equal or greater than DEBOUNCEDELAY
  if((millis() - previousTime_ms) >= DEBOUNCEDELAY){ 
    // serial monitor output
    Serial.print("ADC Value: ");            // print label
    Serial.println(adcValue);               // print ADC input value cause by potentiometer value
    Serial.print("Frequency: ");            // print label
    Serial.print(frequencyValue, 2);        // print frequency
    Serial.println(" Hz");                  // print label
  } 
  lastbuttonstate = digitalRead(BUTTONPIN); // save the reading latest button state
  interrupts();                             // allow external interrupts
}
