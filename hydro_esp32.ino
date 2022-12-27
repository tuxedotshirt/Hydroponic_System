/*
 * Temp Probe -> ESP32
 * Signal (Yellow) --------- 4 (4.7k resistor between signal and 3.3v)
 * VCC ( Red) -------------- 3.3v
 * GND (Black) ------------- GND
 */

 /*
  * TDS Meter -> ESP32
  * A ----------- 34
  * + ----------- 3.3v
  * - ----------- GND
  */

/*
 * Atlas EZO board -> ESP32
 * RX ---------------- SCL 22
 * TX ---------------- SDA 21
 * GND --------------- GND
 * VCC --------------- 3.3
 */

 /*
  * pH Up Pump -> ESP32
  * A1 ------------ 13
  * B1------------- 12
  * C1 ------------ 14
  * D1 ------------ 27
  */

   /*
  * pH Down Pump -> ESP32
  * A2 ------------ 26
  * B2------------- 25
  * C2 ------------ 33
  * D2 ------------ 32
  */
#include <Wire.h>
#include <EEPROM.h>
#include "src/Ezo_i2c/Ezo_i2c.h"
#include "src/Ezo_i2c/Ezo_i2c_util.h"
#include "src/DallasTemp/DallasTemperature.h"
#include "src/TDS/GravityTDS.h"
#include "ESP.h"
#include <OneWire.h>
#include <simpleStepper.h>
#include <simpleTimer.h>

#define TdsSensorPin 34
#define tempPin 33
#define oneWireBus 4
float temperature = 25,tdsValue = 0,kValue=1.0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


#define ecLowThreshold 1000
#define ecHighThreshold 2000
#define ecInterval 10000
#define ecAdjustInterval 5000
unsigned long ecStateInterval = 10000;
//unsigned long ecTimer = 0;
//unsigned long ecMillis = 0;
simpleTimer ecTimer(ecInterval);

#define phLowThreshold 5.9
#define phHighThreshold 6.9
#define phInterval 10000 //5 minutes is 300000 //Default timer
unsigned long phStateInterval = 10000;
#define phAdjustInterval 5000 //Tighter timer to use during adjustment checks
//unsigned long phTimer = 0;
//unsigned long phMillis = 0;
simpleTimer phTimer(phInterval);

double currentPh = 0;
void step1();
void step2();

simpleStepper phUp(13,12,14,27);
simpleStepper phDown(26,25,33,32);
#define NUMBER_OF_STEPS_PER_REV 512
#define DELAY 1

Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99


//Error Flags
bool phReadError = false;
bool ecReadError = false;
bool tempReadError = false;
bool phAdjustError = false;
bool ecAdjustError = false;

TaskHandle_t pHCore; //core 0
TaskHandle_t ecCore; // core 1

void setup() {
  Wire.begin();
  Serial.begin(9600);

  phTimer.initialize();
  ecTimer.initialize();
  
  pinMode(34, INPUT);
  pinMode(33, INPUT);
  sensors.begin();

  xTaskCreatePinnedToCore(pHTask, "pHTask", 10000, NULL, 1, &pHCore, 0);
  xTaskCreatePinnedToCore(ecTask, "ecTask", 10000, NULL, 1, &ecCore, 1);
  
}

void pHTask(void * pvParameters){
  for(;;){
      delay(500);
      if(phTimer.triggered()){
      //Serial.println(phStateInterval);
      Serial.println("Checking pH.");
      check_pH();
    }
  }
    
}

void ecTask(void * pvParameters){
 for(;;){
  delay(500);
  if(ecTimer.triggered()){
    Serial.println("Checking ec.");
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    //Serial.print("temp: ");
    //Serial.println(temperature);
    float analogValue = analogRead(TdsSensorPin);
    float voltage = analogValue/4096*3.3;
    float ecValue = (133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*kValue;
    float ecValue25  =  ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
    float tdsValue = ecValue25 * TdsFactor;
    Serial.print("ecValue: ");
    Serial.println(ecValue);
    //Serial.print("ecValue25: ");
    //Serial.println(ecValue25);
    //Serial.print("tdsValue: ");
    //Serial.println(tdsValue);
  }
 }
}

void loop() {
  
  //TODO: turn on nutrient pump
  //TODO: set adjustment interval
  /*
  if(ecTimer.triggered()){
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    Serial.print("temp: ");
    Serial.println(temperature);
    float analogValue = analogRead(TdsSensorPin);
    float voltage = analogValue/4096*3.3;
    float ecValue = (133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*kValue;
    float ecValue25  =  ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
    float tdsValue = ecValue25 * TdsFactor;
    Serial.print("ecValue: ");
    Serial.println(ecValue);
    Serial.print("ecValue25: ");
    Serial.println(ecValue25);
    Serial.print("tdsValue: ");
    Serial.println(tdsValue);
    //ecTimer = ecMillis;
  }
  if(phTimer.triggered()){
    Serial.println(phStateInterval);
    Serial.println("Checking pH.");
    //step1();
    //delay(1000);
    //step2();
    check_pH();

  }
*/
}

/*
void step1(){
   //send a read command. we use this command instead of PH.send_cmd("R"); 
  //to let the library know to parse the reading
  pH.send_read_cmd();                      
}
*/

void check_pH(){
  //int i = 0;
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);             //get the reading from the PH circuit
  Serial.println("  ");
  
  if (pH.get_error() == Ezo_board::SUCCESS){
    if (pH.get_last_received_reading() <= phLowThreshold) {                            //test condition against pH reading
      Serial.println("PH LEVEL TOO LOW");
      phTimer.setInterval(phAdjustInterval);
      //phStateInterval = 5000;//phAdjustInterval;
      phUp.forward(NUMBER_OF_STEPS_PER_REV, 2);
    }
    else if (pH.get_last_received_reading() >= phHighThreshold) {                          //test condition against pH reading
      Serial.println("PH LEVEL TOO HIGH");
      phTimer.setInterval(phAdjustInterval);
      phDown.forward(NUMBER_OF_STEPS_PER_REV, 2);
    }
    else {
      phTimer.setInterval(phInterval);
    }
  }
  else{
    Serial.println("ERROR");
  }
  Serial.println();
}
