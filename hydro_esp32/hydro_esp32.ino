/*
   Temp Probe -> ESP32
   Signal (Yellow) --------- 4 (4.7k resistor between signal and 3.3v)
   VCC ( Red) -------------- 3.3v
   GND (Black) ------------- GND
*/

/*
   TDS Meter -> ESP32
   A ----------- 34
   + ----------- 3.3v
   - ----------- GND
*/

/*
   Atlas EZO board -> ESP32
   RX ---------------- SCL 22
   TX ---------------- SDA 21
   GND --------------- GND
   VCC --------------- 3.3
*/

/*
   pH Up Pump -> ESP32
   A1 ------------ 13
   B1------------- 12
   C1 ------------ 14
   D1 ------------ 27
*/

/*
  pH Down Pump -> ESP32
  A2 ------------ 26
  B2------------- 25
  C2 ------------ 33
  D2 ------------ 32
*/

/*TODO:
   remove threshold values, determine mathematically instead.
   OTA updates
   Move wifi connection into function to be called if connection is lost
   allow user to enter wifi information, automate DB creation for new user
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
#include "secret.h"
#include "WiFi.h"
#include <HTTPClient.h>
#include "time.h"

//Temperature sensor
#define TdsSensorPin 34
#define oneWireBus 4
float temperature = 25, tdsValue = 0, kValue = 1.0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//EC Probe
#define ecLowThreshold 1000
#define ecHighThreshold 2000
#define ecInterval 10000
#define ecAdjustInterval 5000
unsigned long ecStateInterval = 10000;
simpleTimer ecTimer(ecInterval);
float ec = 0;

//pH Probe
#define phLowThreshold 4//5.9
#define phHighThreshold 5//6.9
#define phInterval 30000 //5 minutes is 300000 //Default timer
unsigned long phStateInterval = 10000;
#define phAdjustInterval 15000 //Tighter timer to use during adjustment checks
simpleTimer phTimer(phInterval);
Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99
float ph = 0;

//Stepper motors
simpleStepper phUp(13, 12, 14, 27);
//simpleStepper phDown(26, 25, 33, 32);
simpleStepper phDown(16, 17, 5, 18);
#define DELAY 2
#define stepsPerRev 512

//RTOS
TaskHandle_t monitorCore;
TaskHandle_t dataLogging;
SemaphoreHandle_t commSemaphore;

//communication
simpleTimer updateDB(10000);

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  phTimer.initialize();
  ecTimer.initialize();

  pinMode(34, INPUT);
  sensors.begin();

  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.
  commSemaphore = xSemaphoreCreateMutex();

  Serial.println(ssid);
  Serial.println(pswd);
  Serial.println(deploymentID);
  Serial.println("Connecting to wifi:");
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void dataLoggingTask(void *pvParameters) {
  for (;;) {
    delay(50);
    if (updateDB.triggered()) {
      if (WiFi.status() == WL_CONNECTED) {
        //static bool flag = true;
        struct tm timeinfo;
        String dateTimeString;
        if (getLocalTime(&timeinfo)) {
          char timeStringBuff[25]; //25 char array to hold dtg
          strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%B%d%H%M%S", &timeinfo); //YYYYMMDDHHMMSS
          String asString(timeStringBuff);
          asString.replace("January", "01");
          asString.replace("February", "02");
          asString.replace("March", "03");
          asString.replace("April", "04");
          asString.replace("May", "05");
          asString.replace("June", "06");
          asString.replace("July", "07");
          asString.replace("August", "08");
          asString.replace("September", "09");
          asString.replace("October", "10");
          asString.replace("November", "11");
          asString.replace("December", "12");
          //asString.replace(" ", "/");
          //Serial.print("Time:");
          //Serial.println(asString);
          dateTimeString = asString;
        }
        else {
          Serial.println("Failed to obtain time");
          dateTimeString = "000000000000"; //default time placeholder
          return;
        }

        //Take mutex, write url string, return semaphore.
        if (commSemaphore != NULL) {
          if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
            Serial.println("Grabbing mutex in dataLoggingTask");
            String urlFinal = "https://script.google.com/macros/s/" + deploymentID + "/exec?" + "device=" + device + "&dtg=" + dateTimeString + "&ph=" + ph + "&ec=" + ec + "&temp=" + temperature;
            xSemaphoreGive(commSemaphore);
            //Serial.print("POST data to spreadsheet:");
            //Serial.println(urlFinal);
            HTTPClient http;
            http.begin(urlFinal.c_str());
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
            int httpCode = http.GET();
            //Serial.print("HTTP Status Code: ");
            //Serial.println(httpCode);
            //---------------------------------------------------------------------
            //getting response from google sheet
            String payload;
            if (httpCode > 0) {
              payload = http.getString();
              //Serial.println("Payload: " + payload);
            }
            //---------------------------------------------------------------------
            http.end();
            Serial.println("Returned mutex in dataLoggingTask");
          }
        }
      }
    }
  }
}

void monitorTask(void * pvParameters) {
  for (;;) {
    delay(100);
    if (phTimer.triggered()) {
      Serial.println("Checking pH.");
      check_pH();
      Serial.println("Done checking pH.");
    }
    if (ecTimer.triggered()) {
      Serial.println("Checking ec.");
      check_ec();
      Serial.println("Done checking ec.");
    }
  }
}

//Puts the temperature compensated ecValue into the global variable
void check_ec() {
  sensors.requestTemperatures();
  float temperatureReading = sensors.getTempCByIndex(0);
  //Serial.print("temp: ");
  //Serial.println(temperature);
  float analogValue = analogRead(TdsSensorPin);
  float voltage = analogValue / 4096 * 3.3;
  float ecValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * kValue;
  float ecValue25  =  ecValue / (1.0 + 0.02 * (temperatureReading - 25.0)); //temperature compensation
  float tdsValue = ecValue25 * TdsFactor;
  //access shared resources to update transmitted value, but continues autonomously if value can't be accessed
  if (commSemaphore != NULL) {
    if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
      Serial.println("Grabbing mutex in check_ec");
      ec = ecValue25;
      temperature = temperatureReading;
      xSemaphoreGive(commSemaphore);
      Serial.println("Returned mutex in check_ec");
    }
  }
  /*
     turn on nutrient pump here
  */
  //Serial.print("ecValue: ");
  //Serial.println(ecValue);
  //Serial.print("ecValue25: ");
  //Serial.println(ecValue25);
  //Serial.print("tdsValue: ");
  //Serial.println(tdsValue);
}

void check_pH() {
  //int i = 0;
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);             //get the reading from the PH circuit
  //Serial.println("  ");
  float phReading = pH.get_last_received_reading();

  //access shared resource, but continues autonomously if db value can't be updated
  if (commSemaphore != NULL) {
    if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
      Serial.println("Grabbing mutex in check_pH");
      ph = phReading;
      xSemaphoreGive(commSemaphore);
      Serial.println("Returned mutex in check_pH");
    }
  }

  if (pH.get_error() == Ezo_board::SUCCESS) {
    if (phReading <= phLowThreshold) {                            //test condition against pH reading
      //Serial.println("PH LEVEL TOO LOW");
      phTimer.setInterval(phAdjustInterval);
      phUp.forward(stepsPerRev, DELAY);
      phUp.off();
    }
    else if (phReading >= phHighThreshold) {                          //test condition against pH reading
      //Serial.println("PH LEVEL TOO HIGH");
      phTimer.setInterval(phAdjustInterval);
      phDown.forward(stepsPerRev, DELAY);
      phDown.off();
    }
    else {
      phTimer.setInterval(phInterval);
    }
  }
  else {
    Serial.println("pH ERROR");
  }
}

//empty loop required by ESP32. Loops are handled in tasks.
void loop() {
}
