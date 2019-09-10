// /!\ OVERWRITE camVal so constant i2c communication without trigger
//SCB-19: red: SDA   black: SCL
//arduino: red: A4   black: A5

#include "Arduino.h"
#include <Wire.h>
#include "MotionSensorAcq.h"

#define _DEBUG_ // debug conditional compiling

//motionSensor
int motionArray[3];

//thermocamera
int thermRespirationPin = A5;
int thermRespValue = 0;

//Lick sensor
int lickPin = 8;
int valveActivationPin = 9;
int lickValue = 0;
int lickValveActivationValue = 0;

int DELAY_I2C = 1; //ms
int DELAY = 1;

int resDebug = 0;

//GPIO
int camPin = 15; //
int od1Pin = 16; //aurora 206 final valve

int camVal = 0;
int od1Val = 0;

//I2C message
char c[5]; //respiratory data
char cam[2];
char od1[2];
//Motion sensor data
char motionX[8];
char motionY[8];
char motionZ[8];
//therm resp
char thermRespVal[4];
//lick sensor
char lickVal[4];
char lickValveActivationVal[4];

int val0 = 0;
unsigned long maxTime1 = 0;
unsigned long maxTime0 = 0;
float rate = 0.0;
int samplingPeriod = 10; //ms
unsigned long time;
const int THD = 2000;

int dderiv1 = 0;
int dderiv0 = 0;
int detected = 0;
int firstDeriv = 0;
int secDeriv = 0;

unsigned long T1 = 0;
unsigned long T0 = 0;

//array to stock average values
const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;
int average0 = 0;

//average rate
const int numAvg = 30;
float readingsAvg[numAvg];      // the readings from the analog input
int readIndexAvg = 0;              // the index of the current reading
float totalAvg = 0.0;                  // the running total
float rateAvg = 0.0;                // the average

//array to store derivative values
const int numderiv = 5;
int deriv[numderiv];
unsigned long derivtime[numderiv];
int derivIndex = 0;

//array to store time in ms where derivative is max
const int numpeaktime = 5;
int peaktime[numpeaktime];
int peaktimeIndex = 0;

struct packet
{
  int raw, average;
};

unsigned long getAverageInterval(int* array, int size) {
  int avg = 0;
  for (int i = 1; i < size; i++) {
    avg += array[i] - array[i - 1];
  }
  return avg / (size - 1);
}

bool approxZero(int value, int eps) {
  return (abs(value) < eps);
}

bool belowZero(int value, int eps) {
  return (value < (0 - eps));
}

int getIndexOfMaximumValue(int* array, int size) {
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}

// the setup routine runs once when you press reset:
void setup() {
  //GPIO
  pinMode(camPin, INPUT);
  pinMode(od1Pin, INPUT);
  pinMode(thermRespirationPin, INPUT);
  Wire.begin();

  // initialize serial communication at 500000 bauds
  Serial.begin(500000);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  for (int thisPeak = 0; thisPeak < numderiv; thisPeak++) {
    deriv[numderiv] = 0;
  }
  for (int thisPeak = 0; thisPeak < numderiv; thisPeak++) {
    derivtime[numderiv] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    peaktime[numpeaktime] = 0;
  }
  for (int thisReading = 0; thisReading < numAvg; thisReading++) {
    readingsAvg[thisReading] = 0;
  }
  //Motion sensor Setup
  motionSensorSetup();
}

// the loop routine runs over and over again forever:
void loop() {

  //--MotionSensorAcquisition--
  motionSensorAcq(&motionArray[0]);

  //thermo respiration pin
  thermRespValue = analogRead(thermRespirationPin);

  //---ACQUISITION---
  camVal = digitalRead(camPin);
  od1Val = digitalRead(od1Pin);

  // respiration
  int val1 = analogRead(0);
  val1 = val1 * (5000 / 1023.0); //im mV

  total = total - readings[readIndex];

  readings[readIndex] = val1;
  deriv[derivIndex] = val1 - val0;
  time = millis();
  derivtime[derivIndex] = time;
  int maxIndex = getIndexOfMaximumValue(deriv, numderiv);
  peaktime[peaktimeIndex] = derivtime[maxIndex];
  maxTime1 = derivtime[maxIndex] * 1000;
  int testt = deriv[maxIndex];

  // add the reading to the total:
  total = total + readings[readIndex];

  // advance to the next position in the array:
  readIndex = readIndex + 1;
  derivIndex = derivIndex + 1;
  peaktimeIndex = peaktimeIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // if we're at the end of the array...
  if (derivIndex >= numderiv) {
    // ...wrap around to the beginning:
    derivIndex = 0;
  }

  // if we're at the end of the array...
  if (peaktimeIndex >= numpeaktime) {
    // ...wrap around to the beginning:
    peaktimeIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  //calculate average interval between peaks
  unsigned long avgPeriod = getAverageInterval(peaktime, numpeaktime);

  //float rate = 1/float(1000*avgPeriod);
  //float rate = 1/(maxTime1-maxTime0);

  dderiv1 = average - average0;

  firstDeriv = average - average0;
  secDeriv = dderiv1 - dderiv0;
  //--try find peak
  T0 = T1;
  if (approxZero(firstDeriv, 15) && belowZero(secDeriv, 10)) {
    T1 = millis();
    detected = 500;
  }

  if  ( (T1 - T0) > 200 )  {
    rate = (1000 / float(T1 - T0)) ;
    //average rate
    totalAvg = totalAvg - readingsAvg[readIndexAvg];
    readingsAvg[readIndexAvg] = rate;
    totalAvg = totalAvg + readingsAvg[readIndexAvg];
    readIndexAvg = readIndexAvg + 1;
    // calculate the average:
    rateAvg = totalAvg / float(numAvg);

  } //ms


  //---TRANSMISSION---
  // print to serial
  Serial.print(500); //millis()); used in optostim visualizer!
  Serial.print('\t');
  Serial.print(val1);

  //Serial.print(", ");
  //Serial.print(T1-T0);
  //Serial.print(", ");
  //Serial.print(rate);

  Serial.print("\n");

  //I2C Communication
  ((String)val1).toCharArray(c, 5);
  ((String)camVal).toCharArray(cam, 2);
  ((String)od1Val).toCharArray(od1, 2);
  ((String)motionArray[0]).toCharArray(motionX, 8);
  ((String)motionArray[1]).toCharArray(motionY, 8);
  ((String)motionArray[2]).toCharArray(motionZ, 8);
  ((String)thermRespValue).toCharArray(thermRespVal, 4);
  ((String)lickValue).toCharArray(lickVal, 4);
  ((String)lickValveActivationValue).toCharArray(lickValveActivationVal, 4);


  ////////////////////////////////////////////////////////////////////
  /*OVERWRITE camVal so constant i2c communication without trigger*/
  ////////////////////////////////////////////////////////////////////
  //camVal=1;

  if (camVal == 1) {

#ifdef _DEBUG_
    Serial.print("I2C ON, camVal == 1");
    Serial.print("\n");
#endif

    Wire.beginTransmission(0x00); // transmit to device with address 0
    Wire.write(c); Wire.write(" "); // sends strain gauge value
    Wire.write(cam); Wire.write(" ");// sends trigger, should be always 1
    Wire.write(od1); Wire.write(" "); // sends final valve
    Wire.write(motionX); Wire.write(" ");
    Wire.write(motionY); Wire.write(" ");
    Wire.write(motionZ); Wire.write(" ");
    Wire.write(thermRespVal); Wire.write(" ");
    Wire.write(lickVal); Wire.write(" ");
    Wire.write(lickValveActivationVal);
    resDebug = Wire.endTransmission();  // stop transmitting

#ifdef _DEBUG_
    Serial.print("resDebug");
    Serial.print(resDebug);
    Serial.print("\n");
#endif
    delay(DELAY_I2C);
  }
  else {
    Serial.print("I2C OFF, camVal != 1");
    Serial.print("\n");
  }

  dderiv0 = dderiv1;
  val0 = val1;
  average0 = average;
  maxTime0 = maxTime1;
  detected = 0;

  if (readIndexAvg >= numAvg) {
    // ...wrap around to the beginning:
    readIndexAvg = 0;
  }
  delay(DELAY);
}

