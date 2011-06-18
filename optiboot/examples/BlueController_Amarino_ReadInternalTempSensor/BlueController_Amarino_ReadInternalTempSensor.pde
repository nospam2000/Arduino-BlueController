/*
  Send sensor data from Arduino to Amarino on Android phone
  (needs SensorGraph and Amarino app installed and running on Android)
  Needs a Pico Power ATmega MUC (anything which has a P after the number, e.g. ATmega328P, ATmega88PA)

  Temperature reading taken from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1252144888#9
  Amarino code taken from SensorGraphTutorial (MeetAndroid lib example)

  Brought together by Michael Dreher <michael(at)5dot1.de>

  Infos about the temperature sensor can be found in the following documents:
     AVR121: Enhancing ADC resolution by oversampling (with example code)
     AVR122: Calibration of the AVR's internal temperature reference
     AVR120: Characterization and Calibration of the ADC on an AVR
     doc8271, chapter 23.8 "Temperature Measurement"
*/
 
#include <avr/io.h>
#include <MeetAndroid.h>

#define NUMBER_OF_SAMPLES (100)

void setup()
{
  Serial.begin(19200);
  delay(100);
  chipTempRaw(); // discard first sample
}

void loop()
{
  float rawTemp = 0.0f;
  for (int i=0; i<NUMBER_OF_SAMPLES; i++) {
    rawTemp += chipTempRaw(); // calculate sum
  }
  rawTemp /= (float)NUMBER_OF_SAMPLES; // calculate average

// either use 1 for Amarino mode or 0 for ASCII mode here
#if 1
  // Amarino mode for Android phone
  meetAndroid.receive(); // you need to keep this in your loop() to receive events
  
  // send result to Android
  meetAndroid.send(chipTemp(rawTemp));
#else
  // ASCII mode for Serial Monitor
  Serial.print("Raw: ");
  Serial.print(rawTemp);
  Serial.print("  Temp: ");
  Serial.print(chipTemp(rawTemp));
  Serial.println(" C");
#endif

  delay(100);
}

float chipTemp(float raw) {
  const float chipTempOffset = 336.59f; // these values are very dependent on your ATmega and your environment (e.g. voltage)
  const float chipTempCoeff = 1.10f;
  return((raw - chipTempOffset) / chipTempCoeff);
}

int chipTempRaw(void) {
  while((ADCSRA & _BV(ADSC))) ;;                 // Wait for any ongoing conversion to complete

  uint8_t oldMux = ADMUX;
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);   // Set internal 1.1V reference, temperature reading
  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE));            // Clear auto trigger and interrupt enable
  ADCSRA |= _BV(ADEN) | _BV(ADSC);               // Enable AD and start conversion
  while((ADCSRA & _BV(ADSC))) ;;                 // Wait until conversion is finished

  ADMUX = oldMux;
  return(ADC);
}
 

