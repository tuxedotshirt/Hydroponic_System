
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


/*
   Pins required:
   3x4 for peristaltic pumps
   1 for ec probe
   2 - RX/TX for pH
   1 for light relay
   1 for button
   1 for main pump relay

   1 for status led
*/
/*TODO:
    remove temperature probe. Can be assumed to be ~20 degrees.
    send reset request over ble for factory settings
    Save deploymentID for DB from app
    USE EC PROBE FOR WATER LEVEL DETECTION. IF 0, WATER LEVEL IS LOW.

    ph and ec adjustment error - denotes chemicals are low
    1 pin nutrient tank level sensor
    1 pin for light relay
*/

/*
   Available pins:
   
   34
   33 might brick wifi?
   35 input only

*/
#include <Wire.h>
#include <EEPROM.h>
#include "src/Ezo_i2c/Ezo_i2c.h"
#include "src/Ezo_i2c/Ezo_i2c_util.h"
#include "src/DallasTemp/DallasTemperature.h"
#include "src/TDS/GravityTDS.h"
#include "ESP.h"
#include <OneWire.h>
#include <simpleTimer.h>
#include "secret.h"
#include "WiFi.h"
#include <HTTPClient.h>
#include "time.h"
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



//Temperature sensor
#define TdsSensorPin 34
//#define oneWireBus 4
float temperature = 22, tdsValue = 0, kValue = 1.0;
//OneWire oneWire(oneWireBus);
//DallasTemperature sensors(&oneWire);

//EC Probe
float ecSetting = 2000;
float ecTemp = 0;
#define ecInterval 20000
#define ecAdjustInterval 5000
//unsigned long ecStateInterval = 10000;
simpleTimer ecTimer(ecInterval);
float ec = 0;

//pH Probe
float phSetting = 7;
float phTemp = 0;
#define phInterval 30000 //5 minutes is 300000 //Default timer
unsigned long phStateInterval = 10000;
#define phAdjustInterval 15000 //Tighter timer to use during adjustment checks
simpleTimer phTimer(phInterval);
Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99
float ph = 0;

//MOSFETS
//#define circulation 13
#define circulation 15
#define pHUp 16
#define pHDown 17
#define nutrients 18
#define lights 23
#define spare 27

#define button 19

//RTOS
TaskHandle_t monitorCore;
TaskHandle_t dataLogging;
SemaphoreHandle_t commSemaphore;
SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t flashSemaphore;

simpleTimer updateDB(25000);
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

#define pwdPref "pwd"
#define ecPref "ecSetting"
#define pHPref "pHSetting"
#define bleNamePref "bleNameSetting"
#define ssidPref "ssidPref"

char *ssidArr;
char *passArr;
char* bleNameArr;
String bleNameString = "Hydroponic";
String bleNameTemp = "";
char macArr[17];
char * key;

#define SW 19

bool bleFlag = false;
simpleTimer changeVar(90000);

void dataLoggingTask(void *pvParameters);
void monitorTask(void * pvParameters);

//Http client for db
HTTPClient http;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);

  //MOSFETS
  pinMode(pHUp, OUTPUT);
  pinMode(pHDown, OUTPUT);
  pinMode(circulation, OUTPUT);
  pinMode(nutrients, OUTPUT);
  pinMode(lights, OUTPUT);
  pinMode(spare, OUTPUT);
  digitalWrite(pHUp, LOW);
  digitalWrite(pHDown, LOW);
  digitalWrite(circulation, LOW);
  digitalWrite(nutrients, LOW);
  digitalWrite(lights, LOW);
  digitalWrite(spare, LOW);
  delay(50);
  phTimer.initialize();
  ecTimer.initialize();
  changeVar.initialize();
  wifiTimer.initialize();
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  //Start ble during startup
  //In the event that the device has been named already, get whatever settings are available before calling bleSettings
  getSettings();
  bleSettings(digitalRead(SW));
  getSettings();
  writeMessage(bleNameString);
  delay(1000);
  //If wifi has been set before, just reconnect and carry on.
  connectWiFi();

  //Assign tasks to cores
  commSemaphore = xSemaphoreCreateMutex();
  flashSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.
}

//void writeMessage(const __FlashStringHelper * message) {
void writeMessage(String message) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(message);
  display.display();
  delay(500);
}

void dataLoggingTask(void *pvParameters) {
  for (;;) {
    getSettings();

    //If ble button has been pressed, restart to enter ble mode
    if (digitalRead(SW) == 0) {
      ESP.restart();
    }

    if (updateDB.triggered()) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("dataLoggingTask no wifi");
        writeMessage(F("WIFI NOT\nCONNECTED"));
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

            http.begin(urlFinal.c_str());
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

            int httpCode = http.GET();
            Serial.print("HTTP Status Code: ");
            Serial.println(httpCode);

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

void connectWiFi() {
  if (getSettings()) {

    wifiTimer.reset();

    WiFi.begin(ssidArr, passArr);
    writeMessage(F("Connecting WIFI"));
    while (WiFi.status() != WL_CONNECTED)
    {
      
      delay(500);
      //Serial.print(F("."));
      //Try for 20 seconds
      if (wifiTimer.triggered()) {
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected!");
      writeMessage(F("WIFI Connected!"));
      // Init and get the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    }
  }
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
    //Serial.print("SSIDArr: ");
    //Serial.println(ssidArr);
    ssidSet = true;
  }
  if (preferences.isKey(pwdPref)) {
    adapterString = preferences.getString(pwdPref);
    adapterLength = adapterString.length() + 1;
    passArr = (char*) malloc (adapterLength);
    adapterString.toCharArray(passArr, adapterLength);
    //Serial.print("PassArr: ");
    //Serial.println(passArr);
  }
  if (preferences.isKey(pHPref)) {
    phSetting = preferences.getFloat(pHPref);
  }
  if (preferences.isKey(ecPref)) {
    ecSetting = preferences.getFloat(ecPref);
  }
  if (preferences.isKey(bleNamePref)) {
    bleNameString = "";
    bleNameString = preferences.getString(bleNamePref);
    //Serial.print("Got bleNameString here: ");
    //Serial.println(bleNameString);
  }
  preferences.end();

  return ssidSet;
}

//void bleSettings(int buttonPressed) {
void bleSettings(int buttonPressed) {
  bool settingsChanged = false;
  writeMessage(F("Bluetooth"));
  if (buttonPressed == 0) {

    Serial.println("beginning SerialBT");
    Serial.println(F(bleNameString));
    if (!SerialBT.begin(bleNameString))
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
    //xSemaphoreTake( flashSemaphore, ( TickType_t ) 10 );
    delay(50);

    //timer to exit bluetooth mode
    if (changeVar.triggered()) {
      Serial.println(F("changeVar triggered"));
      bleFlag = false;
      xSemaphoreGive(flashSemaphore);
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
        preferences.putString(pwdPref, String(passArr));
        bleFlag = false;
        settingsChanged  = true;
      }
      bleNameTemp = String(strtok(NULL, ","));
      if (bleNameTemp != "?") {
        preferences.putString(bleNamePref, bleNameTemp);
        Serial.print("bleName: ");
        Serial.println(preferences.getString(bleNamePref));
        bleFlag = false;
        settingsChanged = true;
      }
      phTemp = atof(strtok(NULL, ","));
      if (phTemp > 0.00) {
        preferences.putFloat(pHPref, phTemp);
        //Serial.print("Saved pHSetting: ");
        //Serial.println(preferences.getFloat("pHSetting"));
        phSetting = preferences.getFloat(pHPref);
        bleFlag = false;
        settingsChanged = true;
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
      //add deploymentID to preferences

      //add BLE display name to preferences
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
    delay(10);
    mainPumpControl();

    check_pH();

    check_ec();

  }
}

//Puts the temperature compensated ecValue into the global variable
bool check_ec() {
  if (ecTimer.triggered()) {
    writeMessage(F("Checking EC"));
    //sensors.requestTemperatures();
    //float temperatureReading = sensors.getTempCByIndex(0);
    float temperatureReading = temperature;
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
        //temperature = temperatureReading;
        xSemaphoreGive(commSemaphore);
        Serial.println(F("Returned mutex in check_ec"));
      }
    }
    
    ecPumpControl(ecValue25);
    return true;
  }
}

bool check_pH() {
  if (phTimer.triggered()) {
    pH.send_read_cmd();
    display.clearDisplay();
    writeMessage(F("Checking pH"));
    delay(1000);
    receive_and_print_reading(pH);             //get the reading from the PH circuit
    float tempReading = pH.get_last_received_reading();


    //if we got a valid reading
    if (pH.get_error() == Ezo_board::SUCCESS) {
      //access shared resource, but continues autonomously if db value can't be updated
      if (commSemaphore != NULL) {
        if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
          Serial.println(F("Grabbing mutex in check_pH"));
          ph = tempReading;
          xSemaphoreGive(commSemaphore);
          Serial.println(F("Returned mutex in check_pH"));
        }
      }
      //The pump should only try to adjust the pH once, with the most current reading and only if there are no errors in the reading.
      phPumpControl(tempReading);
      return true;
    }
    else {
      Serial.println(F("pH ERROR"));
      return false;
    }
  }
}

//blocking function, do not want to take readings from sensors with pumps running
void ecPumpControl(float reading) {
  if (reading <= (ecSetting - 500)) {
    Serial.println(F("Adjusting nutrients"));
    writeMessage(F("Adjusting \nnutrients"));
    digitalWrite(nutrients, HIGH);
    delay(5000);
    digitalWrite(nutrients, LOW);
  }
}

//blocking function, do not want to take readings from sensors with pumps running
void phPumpControl(float reading) {
  if (reading <= (phSetting - 0.25)) {                            //test condition against pH reading
    //Serial.println("PH LEVEL TOO LOW");
    writeMessage(F("Adjusting pH\nup"));
    phTimer.setInterval(phAdjustInterval);
    digitalWrite(pHUp, HIGH);
    delay(2500);
    digitalWrite(pHUp, LOW);
    display.clearDisplay();
  }
  else if (reading >= (phSetting +  0.25)) {                          //test condition against pH reading
    //Serial.println("PH LEVEL TOO HIGH");
    writeMessage(F("Adjusting pH\ndown"));
    phTimer.setInterval(phAdjustInterval);
    digitalWrite(pHDown, HIGH);
    delay(2500);
    digitalWrite(pHDown, LOW);
    display.clearDisplay();
  }
  else {
    phTimer.setInterval(phInterval);
  }
}

//Removed timer. Converted to blocking call to prevent sensor readings when pump is on.
void mainPumpControl() {
      Serial.println("Circulation pump on");
      writeMessage(F("Circulation\npump on"));
      digitalWrite(circulation, HIGH);
      delay(100000);
      Serial.println("Circulation pump off");
      digitalWrite(circulation, LOW);
      display.clearDisplay(); 
}

/*
void mainPumpControl() {
  if (!pumpOn) {
    //check if it's time to turn on
    if (mainPumpOn.triggered()) {
      Serial.println("mainPumpOn triggered");
      writeMessage(F("Circulation\npump on"));
      mainPumpOff.reset();
      digitalWrite(circulation, HIGH);
      pumpOn = true;
      display.clearDisplay();
    }
  }
  //if the pump is on:
  if (pumpOn) {
    if (mainPumpOff.triggered()) {
      Serial.println("mainPumpOff triggered");
      mainPumpOn.reset();
      digitalWrite(circulation, LOW);
      pumpOn = false;
      display.clearDisplay();
    }
  }
}
*/

//empty loop required by ESP32. Loops are handled in tasks.
void loop() {}
