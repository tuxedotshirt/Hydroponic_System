#include <Wire.h>
#include <EEPROM.h>
#include "src/Ezo_i2c/Ezo_i2c.h"
#include "src/Ezo_i2c/Ezo_i2c_util.h"
#include "src/DallasTemp/DallasTemperature.h"
#include "src/TDS/GravityTDS.h"
#include "ESP.h"
#include <OneWire.h>
#include "src/SimpleTimer/simpleTimer.h"
#include "secret.h"
#include "WiFi.h"
#include <HTTPClient.h>
#include "time.h"
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Time.h>
#include "pinDefinitions.h"
#include "preferenceDefinitions.h"
#include "defaultValues.h"

//OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Temperature sensor
float temperature = 22, tdsValue = 0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//EC Probe
float ecSetting = 0;
float ecTemp = 0;
//#define ecInterval 5000

simpleTimer ecTimer(ecInterval);
float ec = 2000;
float kVal = 1.0;
float calibratedTemperature = 25.0;

//pH Probe
float phSetting = 7;
float phTemp = 0;

simpleTimer phTimer(phInterval);
Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99
float ph = 0;
float tempSlope = 0.0;
float tempYIntercept = 0.0;

float ratio1 = 2.5 * ECPumpOnFactor;
float ratio2 = 1.3 * ECPumpOnFactor;
float ratio3 = 2.5 * ECPumpOnFactor;

//RTOS
TaskHandle_t monitorCore;
TaskHandle_t dataLogging;
SemaphoreHandle_t commSemaphore;

simpleTimer updateDB(updateDBTime);
WiFiClient client;
BluetoothSerial SerialBT;
Preferences preferences;
simpleTimer wifiTimer(wifiConnect); //Try to connect to WiFi for 20 seconds
const char* ntpServer = "pool.ntp.org";
ESP32Time rtc(0);

//Lights
int lightOnTime = 700;
int lightOffTime = 2315;

//WIFI
String ssid_pass;
//String adapterString;
int adapterLength;
String bleMessage;

char *ssidArr;
char *passArr;
char* bleNameArr;
String bleNameString = "Hydroponic";
String bleNameTemp = "";

bool bleFlag = false;
simpleTimer changeVar(90000);

void dataLoggingTask(void *pvParameters);
void monitorTask(void * pvParameters);

simpleTimer mainPumpTimer(mainPumpWait);
//Http client for db
HTTPClient http;

//Time
String tz = "MST7MDT,M3.2.0,M11.1.0";

//Error flags
bool waterHigh = false;
bool waterLow = false;
int phChemCounter = 0;
int nutrientCounter = 0;
bool settingsChanged = false;

simpleTimer timeCheckTimer(timeCheck);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinSetup();
  delay(50);
  timerSetup();

  http.setTimeout(30000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  //Assign tasks to cores and create mutex
  commSemaphore = xSemaphoreCreateMutex();

  //In the event that the device has been named already, get whatever settings are available before calling settings
  getSettings();
  settings(digitalRead(SW));
  getSettings();
  connectWiFi();
  
  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.

  pHSetup();
}

void dataLoggingTask(void *pvParameters) {
  for (;;) {
    if (settingsChanged) {
      getSettings();
    }
    if (timeCheckTimer.triggered()) {
      lightControl();
      connectWiFi();
    }

    //If ble button has been pressed, restart to enter settings mode
    if (digitalRead(SW) == 0) {
      ESP.restart();
    }

    if (updateDB.triggeredNoReset()) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot update database. WIFI not connected.");
        writeMessage(F("WIFI NOT\nCONNECTED"));
        WiFi.disconnect();
        connectWiFi();
      }
      else {
        //initTime(tz);
        if (commSemaphore != NULL) {
          if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 1000 / portTICK_PERIOD_MS) == pdTRUE ) {
            Serial.println("Grabbing mutex in dataLoggingTask");
            String urlFinal = URL + "device=" + device + "&dtg=" + rtc.getTime("%d%m%y%H%M") + "&ph=" + ph + "&ec=" + ec + "&temp=" + temperature;
            xSemaphoreGive(commSemaphore);
            Serial.println(urlFinal);
            http.begin(urlFinal.c_str());
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

            int httpCode = http.GET();
            Serial.print("HTTP Status Code: ");
            Serial.println(httpCode);
            if (httpCode != 200 && httpCode != 400) {
              connectWiFi();
            }
            if (httpCode == 200) {
              updateDB.reset();
            }
            http.end();
            writeMessage(F("Updated database"));
            Serial.println("Returned mutex in dataLoggingTask");
          }
        }
      }
    }
    else {
      delay(10); //occupy watchdog timer.
    }
  }
}

void monitorTask(void * pvParameters) {
  for (;;) {
    delay(10);
    if (mainPumpTimer.triggered()) {
      mainPumpControl();
    }
    waterHigh = digitalRead(waterLevel);
    //delay(sensorReadDelay);
    //check_pH();
    //delay(sensorReadDelay);
    //check_ec();

    if (waterLow || waterHigh || phChemCounter >= phChemCounterLimit || nutrientCounter >= nutrientCounterLimit) {
      digitalWrite(errorLED, HIGH);
    }
    else {
      digitalWrite(errorLED, LOW);
    }
  }
}

//blocking function to prevent sensor readings when pump is on.
void mainPumpControl() {
  Serial.println("Circulation pump on");
  writeMessage(F("Circulation\npump on"));
  digitalWrite(circulation, HIGH);
  delay(mainPumpTime);
  Serial.println("Circulation pump off");
  digitalWrite(circulation, LOW);
  mainPumpTimer.reset();
  display.clearDisplay();
}

//empty loop required by ESP32. Loops are handled in tasks.
void loop() {}
