#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
int16_t adc1;
int16_t adc2;

void setup() {
 Serial.begin(9600); //gain
 // ads.setGain(GAIN_TWOTHIRDS);// 2/3x +/- 6.144V 1bit = 0.1875mV default
 // ads.setGain(GAIN_ONE); // 1x +/- 4.096V 1bit = 0.125mV
 // ads.setGain(GAIN_TWO); // 2x +/- 2.048V 1bit = 0.0625mV
 // ads.setGain(GAIN_FOUR); // 4x +/- 1.024V 1bit = 0.03125mV
 // ads.setGain(GAIN_EIGHT); // 8x +/- 0.512V 1bit = 0.015625mV
 // ads.setGain(GAIN_SIXTEEN); // 16x +/- 0.256V 1bit = 0.0078125mV
 ads.begin();
}
void loop() {
 adc1 = ads.readADC_SingleEnded(1);
 adc2 = ads.readADC_SingleEnded(2);
 Serial.print("Analog input pin 1: ");
 Serial.println(adc1);

  Serial.print("Analog input pin 2: ");
 Serial.println(adc2);
 delay(300);
}
