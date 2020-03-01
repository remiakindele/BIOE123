
/* Running all subsystems simultaneously for testing

//}
// thermistor-1.ino Simple test program for a thermistor for Adafruit Learning System
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit
*/

#include <math.h>
#include <EEPROM.h>

// the value of the 'other' resistor
#define SERIESRESISTOR 10000    
// What pin to connect the sensor to
#define THERMISTORPIN A0 

#define THIRTY_MINUTES 1800000

float getInitialOD();
float getInitialCM();

//Pins
int tempPin = THERMISTORPIN; 
int peltPin = 9;
int agiPin = 12;
int airPin = 6; 
int ODPin = A1;
int redPin = 4;
int greenPin = 5;

//initial global values
float startingOD = getInitialOD();
float startingCM = getInitialCM();


//EEPROM 
int addr = 0;
unsigned long StartTime = millis();

void setup(void) {
  Serial.begin(9600);
  pinMode(tempPin, INPUT);
  pinMode(peltPin, OUTPUT);
  pinMode(airPin, OUTPUT);
  pinMode(agiPin, OUTPUT);
  pinMode(ODPin, INPUT);
}

void loop(void) {
  agitationControl();
  aerationControl();
  temperatureControl();
  testStore();
  opticalDensity();
}


int temperatureConversion(int res){
  int temp;
  if(res <= 10000 && res > 9575)
    temp = 25; 
  else if(res <= 9575 && res > 9170)
    temp = 26;
  else if(res <= 9170 && res > 8784)
    temp = 27;
  else if(res <= 8784 && res > 8416)
    temp = 28;
  else if(res <= 8416 && res > 8064)
    temp = 29;
  else if(res <= 8064 && res > 7730)
    temp = 30;
  else if(res <= 7730 && res > 7410)
    temp = 31; 
  else if(res <= 7410 && res > 7106)
    temp = 32;
  else if(res <= 7106 && res > 6815)
    temp = 33;
  else if(res <= 6815 && res > 6538)
    temp = 34;
  else if(res <= 6538 && res > 6273)
    temp = 35;
  else if(res <= 6273 && res > 6020)
    temp = 36;
  else if(res <= 6020 && res > 5778)
    temp = 37;
  else if(res <= 5778 && res > 5548)
    temp = 38;
  else if(res <= 5548 && res > 5327)
    temp = 39;
  else if(res <= 5327 && res > 5117)
    temp = 40;
  else if(res <= 5117 && res > 4915)
    temp = 41;
  else if(res <= 4915)
    temp = 42;
  else
    temp = 0;
  return temp;
}

float getInitialOD(){
  return getODMeasurement(); 
}
float getInitialCM(){
  float OD = getODMeasurement(); 
  float CM = getCMMeasurement();

  float difference = OD - CM; 
  return difference; 
}

float getOD(){ // returns a fold change
  float OD = getODMeasurement();
  float foldChangeOD = (OD - startingOD)/startingOD; 

  return foldChangeOD;  
}

float getCM(){ // returns a fold change difference
  float CM = getCMMeasurement() - getODMeasurement();
  float foldChangeCM = (CM - startingCM)/startingCM; 

  return foldChangeCM;
}

void agitationControl(){
  analogWrite(agiPin, 128);
  return;
}
void aerationControl(){
  analogWrite(airPin, 255);
  return;
}
void temperatureControl(){
  float reading = tempMeasurement();
  heatControl(reading);
  return;
}
float tempMeasurement(){
  float reading;
  reading = analogRead(THERMISTORPIN);
  
  // convert the value to resistance
  reading = (1023 / reading)  - 1;     // (1023/ADC - 1) 
  reading = SERIESRESISTOR / reading;  // 10K / (1023/ADC - 1)
  return reading;
}

void heatControl(float reading){

  int pwm = (((5900 - reading)/(6020-5778))*255) + 350;
  
  if(reading >= 5778 && reading <= 6020){
    analogWrite(peltPin, pwm);
  }
  else if(reading < 5778){ // catch all for if temperature dips too low
    analogWrite(peltPin, 0);
  }
  else if(reading > 6020){ // catch all for if temperature dips too high
    analogWrite(peltPin, 255); 
  }
  return;
}

float getODMeasurement(){
  
  redLight();
  
  unsigned long measurementStart = millis();
  unsigned long measurementTime = millis() - measurementStart;\
  int numMeasurements = 0;
  float measurement;
  float sumMeasurements = 0;
  
  while(measurementTime <= 10000){
    measurementTime = millis() - measurementStart;
    numMeasurements += 1; 
    measurement = analogRead(A1);     
    sumMeasurements += measurement; 
  }
  float average = sumMeasurements/numMeasurements;

  lightsOff();  
  return average;
}

float getCMMeasurement(){
  
  greenLight();
  
  unsigned long measurementStart = millis();
  unsigned long measurementTime = millis() - measurementStart;\
  int numMeasurements = 0;
  float measurement;
  float sumMeasurements = 0;
  
  while(measurementTime <= 10000){
    measurementTime = millis() - measurementStart;
    numMeasurements += 1; 
    measurement = analogRead(A1);     
    sumMeasurements += measurement; 
  }
  float average = sumMeasurements/numMeasurements;
  
  lightsOff();
  return average;
  
  return;
}

void redLight(){

  // insert blinking?
  
  analogWrite(redPin, 255);
  analogWrite(greenPin, 0); 
}

void greenLight(){

  // insert blinking?
  
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255); 
}

void lightsOff(){
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0); 
}
void opticalDensity(){

  float ODChange = getOD();
  float CMChange = getCM();

  Serial.println("Change in Optical Density: "); 
  Serial.print(ODChange);
  Serial.print("X increase in ");
  Serial.print((millis() - StartTime)/60000.0);
  Serial.print("minutes of operation.");

  Serial.println("Change in Color: ");
  Serial.print(CMChange);
  Serial.print("X increase in ");
  Serial.print((millis() - StartTime)/60000.0);
  Serial.print("minutes of operation.");

  return;
  
}
void testStore(){
  float reading = tempMeasurement();

  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;

  if(ElapsedTime > THIRTY_MINUTES/2){ // every 30 seconds
    int testVal = reading;
    testVal = temperatureConversion(testVal);
    EEPROM.write(addr, testVal);
    addr = addr + 1;
    if (addr == EEPROM.length()) {
      addr = 0;
    }
    delay(10);
    Serial.println("Data saved:");
    Serial.println(testVal);
    Serial.println(reading);
    StartTime = millis();
    return;
  }
  else
    return;
  return; 
}
