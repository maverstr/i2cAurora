// /!\ OVERWRITE cameraTriggerValue for constant i2c communication without trigger
//SCB-19: red: SDA   black: SCL
//arduino: red: A4   black: A5
//mega : data: 20, clock : 21

#include "Arduino.h"
#include <Wire.h>

//#define _DEBUG_ // debug conditional compiling


/*TO ADD A NEW DATA:
 * add the corresponding pin in the code: 
 *    int myDataPin = "my pin number"
 *
 * add an acquisition value variable
 *    int myDataValue;
 *    
 * add an I2C array to transmit this value.
 * This array should have the size of the number of characters you want to send +1.
 * i.e. to send 327, the array size is 4, because each char is one byte and we want to transmit bytes.
 *    int myDataI2cArray["my required size"];
 *    
 * add the pinMode in Setup()
 *    pinMode(myDataPin, INPUT);
 *    
 * in the acquisition part (dataAcquisition() function)
 * add your data reading
 *  if it is a digital value:
 *    myDataValue = digitalRead(myDataPin)
 *    
 *  if it is an analog value:
 *    myDataValue = analogRead(myDataPin);
 *    
 *    
 * add the value in plot:
 *     Serial.print("yourData: "); Serial.print(yourData); Serial.print(" ");
 *   Make sure that the last line of ArduinoPlotting is Serial.print(\n");
 *   
 * add the data in the bytes conversion function i2cDataTransform
 *    ((String)myDataValue).toCharArray(myDataI2cArray, my array size);
 *     
 * add the i2C communication line inbetween Wire.beginTransmission(0x00); and Wire.endTransmission();
 *    Wire.write(myDataI2cArray); Wire.write(" ");
  */

// ===============================
// =====         GPIO         ====
// ===============================

//motionSensor
int clockMouse1Pin = 6;
int dataMouse1Pin = 5;
int clockMouse2Pin = 3;
int dataMouse2Pin = 2;
#include "MotionSensorAcq.h"

//thermocamera
int thermRespirationPin = A5;

//Lick sensor
int lickPin = 8;
int lickValveActivationPin = 9;
int lickAirValveActivationPin = 10;

//strain gauge
int strainGaugePin = A0;

//camera trigger and stimulus
int cameraTriggerPin = 15; //
int stimulusPin = 16; //aurora 206 final valve

// ===============================
// =====  Acquistion Values   ====
// ===============================
int motionSensorValuesArray[3];
int thermRespValue;
int lickValue;
int lickValveActivationValue;
int lickAirValveActivationValue;
int strainGaugeValue;
int cameraTriggerValue; // == 1 when triggered
int stimulusValue;

// ===============================
// =====         I2C          ====
// ===============================
int DELAY_I2C = 1;
int i2cError = 0;

//I2C message
char strainGaugeI2cArray[5];
char cameraTriggerI2cArray[2];
char stimulusI2cArray[2];
//Motion sensor data
char motionXI2cArray[8];
char motionYI2cArray[8];
char motionZI2cArray[8];
//therm resp
char thermRespI2cArray[4];
//lick sensor
char lickValueI2cArray[2];
char lickValveActivationI2cArray[2];
char lickAirValveActivationI2cArray[2];

// the setup routine runs once when you press reset:
void setup() {
  pinMode(cameraTriggerPin, INPUT);
  pinMode(stimulusPin, INPUT);
  pinMode(thermRespirationPin, INPUT);
  pinMode(lickPin, INPUT);
  pinMode(lickValveActivationPin, INPUT);
  pinMode(lickAirValveActivationPin, INPUT);
  pinMode(strainGaugePin, INPUT);
  Wire.begin();
  Serial.begin(500000);
  motionSensorSetup();
}

void loop() {
  dataAcquisition();
  ArduinoPlot();
  i2cDataTransform();

  ////////////////////////////////////////////////////////////////////
  /*OVERWRITE cameraTriggerValue so constant i2c communication without trigger*/
  ////////////////////////////////////////////////////////////////////
  //cameraTriggerValue=1;

  if (cameraTriggerValue == 1) {
#ifdef _DEBUG_
    Serial.println("I2C ON, cameraTriggerValue == 1");
#endif
    i2cCommunication();

#ifdef _DEBUG_
    Serial.print("i2C error: "); Serial.println(i2cError);
#endif
    delay(DELAY_I2C);
  }
#ifdef _DEBUG_
  else {
    Serial.println("I2C OFF, cameraTriggerValue != 1");
  }
#endif
}

//-----------------------------------------------------------------------------------

// ===============================
// =====    Transmission      ====
// ===============================
void i2cDataTransform() {
  ((String)strainGaugeValue).toCharArray(strainGaugeI2cArray, 5);
  ((String)cameraTriggerValue).toCharArray(strainGaugeI2cArray, 2);
  ((String)stimulusValue).toCharArray(strainGaugeI2cArray, 2);
  ((String)motionSensorValuesArray[0]).toCharArray(motionXI2cArray, 8);
  ((String)motionSensorValuesArray[1]).toCharArray(motionYI2cArray, 8);
  ((String)motionSensorValuesArray[2]).toCharArray(motionZI2cArray, 8);
  ((String)thermRespValue).toCharArray(thermRespI2cArray, 4);
  ((String)lickValue).toCharArray(lickValueI2cArray, 2);
  ((String)lickValveActivationValue).toCharArray(lickValveActivationI2cArray, 2);
  ((String)lickAirValveActivationValue).toCharArray(lickAirValveActivationI2cArray, 2);
}

void i2cCommunication() {
  Wire.beginTransmission(0x00); // transmits to device with address 0
  Wire.write(strainGaugeI2cArray); Wire.write(" "); // sends strain gauge value
  Wire.write(cameraTriggerI2cArray); Wire.write(" ");// sends trigger, should be always 1
  Wire.write(strainGaugeI2cArray); Wire.write(" "); // sends final valve
  Wire.write(motionXI2cArray); Wire.write(" ");
  Wire.write(motionYI2cArray); Wire.write(" ");
  Wire.write(motionZI2cArray); Wire.write(" ");
  Wire.write(thermRespI2cArray); Wire.write(" ");
  Wire.write(lickValueI2cArray); Wire.write(" ");
  Wire.write(lickValveActivationI2cArray); Wire.write(" ");
  i2cError = Wire.endTransmission();
}

// ===============================
// =====   Arduino plotting   ====
// ===============================
void ArduinoPlot() {
  Serial.print("motionX: "); Serial.print(motionSensorValuesArray[0]); Serial.print(" ");
  Serial.print("motionY: "); Serial.print(-motionSensorValuesArray[1]); Serial.print(" "); //note the negative
  Serial.print("motionZ: "); Serial.print(motionSensorValuesArray[2]); Serial.print(" ");
  Serial.print("termResp: "); Serial.print(thermRespValue); Serial.print(" ");
  Serial.print("lickValue: "); Serial.print(lickValue *5000); Serial.print(" ");
  Serial.print("lickValveActivationValue: "); Serial.print(lickValveActivationValue *5000); Serial.print(" ");
  Serial.print("lickAirValveActivationValue: "); Serial.print(lickAirValveActivationValue *5000); Serial.print(" ");
  Serial.print("cameraTriggerValue: "); Serial.print(cameraTriggerValue *5000); Serial.print(" ");
  Serial.print("stimulusValue: "); Serial.print(stimulusValue *5000); Serial.print(" ");
  Serial.print("strainGaugeValue: "); Serial.print(strainGaugeValue); Serial.print(" ");

  Serial.print("\n"); //required to plot each time
}

// ===============================
// =====     Acquisition      ====
// ===============================
void dataAcquisition() {
  motionSensorAcq(&motionSensorValuesArray[0]);
  thermRespValue = analogRead(thermRespirationPin);
  lickValue = digitalRead(lickPin);
  lickValveActivationValue = digitalRead(lickValveActivationPin);
  lickAirValveActivationValue = digitalRead(lickAirValveActivationPin);
  cameraTriggerValue = digitalRead(cameraTriggerPin);
  stimulusValue = digitalRead(stimulusPin);
  strainGaugeValue = analogRead(strainGaugePin);
  strainGaugeValue *= (5000 / 1023.0); //in mV
}

