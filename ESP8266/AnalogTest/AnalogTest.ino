#include <ArduinoSort.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
int16_t adc1;
int16_t adc2;

void setup() {
  Serial.begin(115200);
  ads.begin();
  Serial.println("Starting");
}

void loop() {
  
  
  Serial.print(ads.readADC_SingleEnded(1)); Serial.print(" :: ");
  Serial.print(getAdcAvg(1)); Serial.print("|"); Serial.println("");

  delay(10);
}


// Get the avg Analog value - ignoring outliers and returning -1 if we're floating
int getAdcAvg(int _channel) {
  int vals[5];
  for (int i=0;i<5;i++) {
    vals[i] = ads.readADC_SingleEnded(_channel);
  }

  sortArray(vals, 5);
  float avg = (vals[1]+vals[2]+vals[3])/3; // We ignore biggest and smallest

  float avgUpper = avg+100;
  float avgLower = avg-100;
  for(int i=1;i<4;i++){
    if (vals[i] > avgUpper || vals[i] < avgLower)
    return -1;  
  }

  return (int)avg;
}
