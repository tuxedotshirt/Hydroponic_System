
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

/*Preferences:
   ssid - string
   pwd - string
   phSetting - float
   ecSetting - float
*/

/*TODO:
    Check wifi connection periodically, attempt reconnect
    Set wifi hostname
    Set ble name on app
    x1 pin Add factory reset button or send reset request over ble
    Save deploymentID for DB from app
    x1 pin Main tank water level sensor
    ph and ec adjustment error - denotes chemicals are low
    x1 pin main pump relay or mosfett
    x1 pin air stone relay or mosfett
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
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <Cipher.h>


//Temperature sensor
#define TdsSensorPin 34
#define oneWireBus 4
float temperature = 25, tdsValue = 0, kValue = 1.0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//EC Probe
//#define ecLowThreshold 1000
//#define ecHighThreshold 2000
float ecSetting = 2000;
float ecTemp = 0;
#define ecInterval 10000
#define ecAdjustInterval 5000
unsigned long ecStateInterval = 10000;
simpleTimer ecTimer(ecInterval);
float ec = 0;

//pH Probe
#define phLowThreshold 4//5.9
#define phHighThreshold 5//6.9
float phSetting = 7;
float phTemp = 0;
#define phInterval 30000 //5 minutes is 300000 //Default timer
unsigned long phStateInterval = 10000;
#define phAdjustInterval 15000 //Tighter timer to use during adjustment checks
simpleTimer phTimer(phInterval);
Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99
float ph = 0;

//Stepper motors
simpleStepper phUp(13, 26, 14, 27);
simpleStepper phDown(16, 17, 5, 18);
#define DELAY 2
#define stepsPerRev 512

//RTOS
TaskHandle_t monitorCore;
TaskHandle_t dataLogging;
SemaphoreHandle_t commSemaphore;
SemaphoreHandle_t eepromSemaphore;

//communication
simpleTimer updateDB(10000);
WiFiClient client;
BluetoothSerial SerialBT;
Preferences preferences;
simpleTimer wifiTimer(20000); //Try to connect to WiFi for 20 seconds
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -25200;
const int   daylightOffset_sec = 3600;

String ssid_pass;
String adapterString;
int adapterLength;
String bleMessage;

#define ssidPref "ssid"
#define pwdPref "pwd"
#define ecPref "ecSetting"
#define pHPref "pHSetting"

char *ssidArr;
char *passArr;
char macArr[17];
char * key;

//BLE switch pin
const int SW = 19;
bool bleFlag = false;
simpleTimer changeVar(30000);

Cipher * cipher = new Cipher();
void dataLoggingTask(void *pvParameters);
void monitorTask(void * pvParameters);

//Http client for db
HTTPClient http;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  pinMode(34, INPUT);
  sensors.begin();

  phTimer.initialize();
  ecTimer.initialize();
  changeVar.initialize();
  wifiTimer.initialize();
  setCipherKey();
  //wifiCredentialsTESTING();

  //Start ble during startup
  bleSettings(digitalRead(SW));

  //If wifi has been set before, just reconnect and carry on.
  connectWiFi();

  //Assign tasks to cores
  commSemaphore = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.

}

void dataLoggingTask(void *pvParameters) {
  for (;;) {

    //If ble button has been pressed, restart to enter ble mode
    if (digitalRead(SW) == 0) {
      ESP.restart();
    }
    //bleSettings(digitalRead(SW));

    //delay(50);
    if (updateDB.triggered()) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("dataLoggingTask no wifi");
        connectWiFi();
      }
      else {
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
          dateTimeString = asString;
          Serial.println("Obtained current time");
        }
        else {
          Serial.println("Failed to obtain time");
          dateTimeString = "000000000000"; //default time placeholder
        }

        //Take mutex, write url string, return mutex.
        if (commSemaphore != NULL) {
          if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
            Serial.println("Grabbing mutex in dataLoggingTask");
            String urlFinal = URL + "device=" + device + "&dtg=" + dateTimeString + "&ph=" + ph + "&ec=" + ec + "&temp=" + temperature;
            xSemaphoreGive(commSemaphore);
            //Serial.print("POST data to spreadsheet:");
            //Serial.println(urlFinal);

            http.begin(urlFinal.c_str());
            //delay(50);
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

            int httpCode = http.GET();
            Serial.print("HTTP Status Code: ");
            Serial.println(httpCode);
            //---------------------------------------------------------------------
            //getting response from google sheet
            //String payload;
            //if (httpCode > 0) {
            //payload = http.getString();
            //Serial.println("Payload: " + payload);
            //}
            //---------------------------------------------------------------------
            http.end();

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

void wifiCredentialsTESTING() {
  preferences.begin("WiFiLogin", false);
  preferences.clear();
  String ssidStr = "PrettyFlyForAWIFI-2.4";
  String passStr = "j5zu522xw7";
  preferences.putString("ssid", ssidStr);
  preferences.putString("pwd", passStr);
  preferences.end();
  if (getSettings()) {
    Serial.println("got settings");
  }
}

void connectWiFi() {
  if (getSettings()) {
    wifiTimer.reset();

    WiFi.begin(ssidArr, passArr);
    Serial.print("connectWiFi ssidArr: ");
    Serial.print(ssidArr);
    Serial.println(".");
    Serial.print("connectWiFi passArr: ");
    Serial.print(passArr);
    Serial.println(".");
    //Serial.println("here 1");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(F("!"));
      //Try for 20 seconds
      if (wifiTimer.triggered()) {
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected!");
      // Init and get the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    }
  }
}

bool putSetting(String arg) {

}

bool getSettings() {
  bool ssidSet = false;
  free(ssidArr); //Free allocated memory
  free(passArr); //Free allocated memory

  preferences.begin("WiFiLogin", false);

  if (preferences.isKey(ssidPref)) {
    adapterString = preferences.getString(ssidPref);
    adapterLength = adapterString.length() + 1;
    ssidArr = (char*) malloc (adapterLength);
    adapterString.toCharArray(ssidArr, adapterLength);
    ssidSet = true;
  }
  if (preferences.isKey(pwdPref)) {
    adapterString = cipher->decryptString(preferences.getString(pwdPref));
    adapterLength = adapterString.length() + 1;
    passArr = (char*) malloc (adapterLength);
    adapterString.toCharArray(passArr, adapterLength);
  }
  if (preferences.isKey(pHPref)) {
    phSetting = preferences.getFloat(pHPref);
  }
  if (preferences.isKey(ecPref)) {
    ecSetting = preferences.getFloat(ecPref);
  }
  preferences.end();

  return ssidSet;
}

void setCipherKey() {
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  String mac = WiFi.macAddress();
  mac.toCharArray(macArr, 17);
  key = macArr;

  cipher->setKey(key);
}

void bleSettings(int buttonPressed) {
  bool settingsChanged = false;

  if (buttonPressed == 0) {
    Serial.println("beginning SerialBT");
    if (!SerialBT.begin("ESP32"))
    {
      Serial.println(F("An error occurred initializing Bluetooth"));
    }
    else {
      Serial.println(F("Initialized Bluetooth"));
    }
    bleFlag = true;
    changeVar.reset();
    Serial.println(F("BLE button pressed!"));
  }
  else {
    delay(10);
  }
  while (bleFlag == true) {
    delay(50);
    if (changeVar.triggered()) {
      Serial.println(F("changeVar triggered"));
      bleFlag = false;
    }
    while (SerialBT.available()) {
      preferences.begin("WiFiLogin", false);
      String bleMessage = SerialBT.readString();
      char tempArr[bleMessage.length() + 1];
      bleMessage.toCharArray(tempArr, bleMessage.length() + 1);
      if (bleMessage.length() >= 1) {
        //bleFlag = false;
      }
      ssidArr = strtok(tempArr, ",");
      //Serial.print("ssidArr: ");
      //Serial.println(String(ssidArr));
      if (String(ssidArr) != "?") {
        preferences.putString(ssidPref, String(ssidArr));
        //Serial.print("Saved ssid: ");
        //Serial.println(preferences.getString(ssidPref));
        bleFlag = false;
        settingsChanged  = true;
      }
      passArr = strtok(NULL, ",");
      if (String(passArr) != "?") {
        //Save encrypted password
        String data = String(passArr);
        String cipherString = cipher->encryptString(data);
        preferences.putString(pwdPref, cipherString);

        //Save unencrypted password
        //preferences.putString(pwdPref, String(passArr));
        
        bleFlag = false;
        settingsChanged  = true;
      }
      phTemp = atof(strtok(NULL, ","));
      if (phTemp > 0.00) {
        preferences.putFloat(pHPref, phTemp);
        //Serial.print("Saved pHSetting: ");
        //Serial.println(preferences.getFloat("pHSetting"));
        phSetting = preferences.getFloat(pHPref);
        bleFlag = false;
        settingsChanged  = true;
      }
      ecTemp = atoi(strtok(NULL, ","));
      if (ecTemp > 0.00) {
        preferences.putFloat(ecPref, ecTemp);
        //Serial.print("Saved ecSetting: ");
        //Serial.println(preferences.getFloat(ecPref));
        ecSetting = preferences.getFloat(ecPref);
        bleFlag = false;
        settingsChanged  = true;
      }
      preferences.end();
      //break;

    }
  }
  if (bleFlag == false && settingsChanged) {
    //restart and use preferences to connect to wifi
    ESP.restart();
  }
}

void monitorTask(void * pvParameters) {
  for (;;) {
    delay(100);
    if (phTimer.triggered()) {
      //Serial.println("Checking pH.");
      check_pH();
      //Serial.println("Done checking pH.");
    }
    if (ecTimer.triggered()) {
      //Serial.println("Checking ec.");
      check_ec();
      //Serial.println("Done checking ec.");
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
      Serial.println(F("Grabbing mutex in check_ec"));
      ec = ecValue25;
      temperature = temperatureReading;
      xSemaphoreGive(commSemaphore);
      Serial.println(F("Returned mutex in check_ec"));
    }
  }
  if (ecValue25 >= (ecSetting - 500)) {
    Serial.println(F("turning on nutrient pump"));
    /*
       turn on nutrient pump here
    */
  }

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
      Serial.println(F("Grabbing mutex in check_pH"));
      ph = phReading;
      xSemaphoreGive(commSemaphore);
      Serial.println(F("Returned mutex in check_pH"));
    }
  }

  if (pH.get_error() == Ezo_board::SUCCESS) {
    if (phReading <= (phSetting - 0.25)) {                            //test condition against pH reading
      //Serial.println("PH LEVEL TOO LOW");
      phTimer.setInterval(phAdjustInterval);
      phUp.forward(stepsPerRev, DELAY);
      phUp.off();
    }
    else if (phReading >= (phSetting +  0.25)) {                          //test condition against pH reading
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
    Serial.println(F("pH ERROR"));
  }
}

//empty loop required by ESP32. Loops are handled in tasks.
void loop() {
}
