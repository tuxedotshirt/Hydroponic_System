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
    send reset request over ble for factory settings
    Save deploymentID for DB from app
    use ec probe for low water level detection. 0 means water is low
    Add light on/off times to app
    Add error led to pcb
    ph and ec adjustment error - denotes chemicals are low
    1 pin tank overflow level sensor
    Add manual time entry
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
#include <ESP32Time.h>

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



//Temperature sensor
#define TdsSensorPin 34
#define oneWireBus 4
float temperature = 22, tdsValue = 0, kValue = 1.0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//EC Probe
float ecSetting = 0;
float ecTemp = 0;
#define ecInterval 5000
#define ecAdjustInterval 5000
simpleTimer ecTimer(ecInterval);
float ec = 2000;
float kVal = 1.0;

//pH Probe
float phSetting = 7;
float phTemp = 0;
#define phInterval 5000 //10 minutes is 600000 //Default timer
#define phAdjustInterval 5000 //1 minute timer to use during adjustment checks
simpleTimer phTimer(phInterval);
Ezo_board pH = Ezo_board(99, "pH");  //i2c address of pH EZO board is 99
float ph = 0;

//Pin definitions
#define circulation 15
#define pHUp 16
#define pHDown 17
#define part1 18
#define lights 23
#define part2 27
#define part3 25
#define errorLED 13
#define button 19

float ratio1 = 1500;
float ratio2 = 2300;
float ratio3 = 1700;

//RTOS
TaskHandle_t monitorCore;
TaskHandle_t dataLogging;
SemaphoreHandle_t commSemaphore;
//SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t flashSemaphore;

simpleTimer updateDB(5000); //update every 10 minutes
WiFiClient client;
BluetoothSerial SerialBT;
Preferences preferences;
simpleTimer wifiTimer(20000); //Try to connect to WiFi for 20 seconds
const char* ntpServer = "pool.ntp.org";
//ESP32Time rtc;
ESP32Time rtc(0);

//Lights
bool lightState = false;
int lightOnTime = 1234;
int lightOffTime = 2345;

String ssid_pass;
//String adapterString;
int adapterLength;
String bleMessage;

#define pwdPref "pwd"
#define ecPref "ecSetting"
#define pHPref "pHSetting"
#define bleNamePref "bleNameSetting"
#define ssidPref "ssidPref"
#define ecZero "EC"
#define kValue "kValue"

char *ssidArr;
char *passArr;
char* bleNameArr;
String bleNameString = "Hydroponic";
String bleNameTemp = "";
//char macArr[17];
//char * key;

#define SW 19

bool bleFlag = false;
simpleTimer changeVar(90000);

void dataLoggingTask(void *pvParameters);
void monitorTask(void * pvParameters);

//Http client for db
HTTPClient http;

//Time
String tz = "MST7MDT,M3.2.0,M11.1.0";

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);

  //MOSFETS
  pinMode(pHUp, OUTPUT);
  pinMode(pHDown, OUTPUT);
  pinMode(circulation, OUTPUT);
  pinMode(part1, OUTPUT);
  pinMode(lights, OUTPUT);
  pinMode(part2, OUTPUT);
  pinMode(part3, OUTPUT);
  pinMode(errorLED, OUTPUT);
  
  digitalWrite(pHUp, LOW);
  digitalWrite(pHDown, LOW);
  digitalWrite(circulation, LOW);
  digitalWrite(part1, LOW);
  digitalWrite(lights, LOW);
  digitalWrite(part2, LOW);
  digitalWrite(part3, LOW);
  digitalWrite(errorLED, LOW);
  
  delay(50);
  phTimer.initialize();
  ecTimer.initialize();
  changeVar.initialize();
  wifiTimer.initialize();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  //Start ble during startup
  //In the event that the device has been named already, get whatever settings are available before calling settings
  getSettings();
  settings(digitalRead(SW));
  getSettings();
  //writeMessage(bleNameString);
  //delay(1000);
  //If wifi has been set before, just reconnect and carry on.
  connectWiFi();

  //Assign tasks to cores and create mutexes
  commSemaphore = xSemaphoreCreateMutex();
  flashSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.
}

void setEC() {
  writeMessage("Place probe in\nnutrient solution\nto set desired EC.");
  delay(5000);
  for (int i = 60; i >= 0; i--) {
    writeMessage("Stabilizing reading.\nHold button to exit.\n" + String(i) + " s remaining");
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(5000);
      ESP.restart();
    }
  }
  //take reading, save to preferences.
  sensors.requestTemperatures();
  float temperatureReading = sensors.getTempCByIndex(0);
  //float temperatureReading = temperature;
  float analogValue = analogRead(TdsSensorPin);
  float voltage = analogValue / 4096 * 3.3;
  float ecValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * kVal;
  float ecValue25  =  ecValue / (1.0 + 0.02 * (temperatureReading - 25.0)); //temperature compensation

  preferences.begin("WiFiLogin", false);
  preferences.putFloat(ecZero, ecValue25);
  preferences.end();
  writeMessage("EC Point set: " + String(ecValue25) + " uS");
  delay(5000);

  for (int i = 10; i >= 0; i--) {
    writeMessage("Press button to return\n to SETTINGS\n" + String(i) + "s remaining");
    if (!digitalRead(SW)) {
      settings(digitalRead(SW));
    }
    delay(1000);
  }

  writeMessage("Exiting");
  delay(2500);
  ESP.restart();
}

void calibrateEC() {
  float KValueTemp;
  //rawECsolution is given in ppm
  float rawECsolution = 1000;
  float voltage = 3.3;

  writeMessage("EC Calibration\nRelease button.");
  delay(5000);
  for (int i = 10; i >= 0; i--) {
    writeMessage("Hold to set EC. " + String(i));
    if (!digitalRead(SW)) {
      setEC();
      delay(1000);
    }
  }
  writeMessage("Place probe in \n2000uS buffer\nsolution");
  delay(10000);
  for (int i = 60; i >= 0; i--) {
    writeMessage("Reading. Hold button\nto skip and\nset EC point.\n" + String(i) + " seconds remaining");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(5000);
      ESP.restart();
    }
  }
  //rawECsolution = rawECsolution * (1.0 + 0.02 * (temperature - 25.0));
  rawECsolution = rawECsolution * 2; //convert from tds to EC
  rawECsolution = rawECsolution * (1.0 + 0.02 * (temperature - 25.0));
  KValueTemp = rawECsolution / (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage); //calibrate in the  buffer solution, EC 2000uS, NaCl 1000ppm
  if ((rawECsolution > 0) && (rawECsolution <= 2000) && (KValueTemp > 0.25) && (KValueTemp < 4.0))
  {
    Serial.print(KValueTemp);
    preferences.begin("WiFiLogin", false);
    preferences.putFloat(kValue, KValueTemp);
    preferences.end();
    writeMessage("EC Calibration\nsuccessful.\nPress button to return\nto Settings.");

    for (int i = 10; i >= 0; i--) {
      settings(digitalRead(SW));
      delay(1000);
    }
  }
  else {
    writeMessage("EC Calibration failed.\nPress button to return\nto Settings.");
    for (int i = 10; i >= 0; i--) {
      settings(digitalRead(SW));
      delay(1000);
    }
  }
  ESP.restart();


}

//give user 30s to place probe in solution, 30s to rinse and place in next solution
void calibratePH() {
  writeMessage("pH Calibration\nRelease button");
  delay(5000);

  for (int i = 10; i >= 0; i--) {
    writeMessage("Hold for EC \ncalibration.\npH calibration in " + String(i));
    if (!digitalRead(SW)) {
      calibrateEC();
      delay(1000);
    }
  }

  for (int i = 30; i >= 0; i--) {
    writeMessage("Place probe in\n4 pH solution.\n" + String(i) + " seconds remaining.\nHold to exit.");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(2500);
      ESP.restart();
    }
  }
  //Calibrate pH = 4
  //ph.cal_low();
  writeMessage("pH=4 COMPLETE");
  delay(5000);

  for (int i = 10; i >= 0; i--) {
    writeMessage("Rinse probe.\n" + String(i) + " seconds remaining.\nHold to exit.");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(2500);
      ESP.restart();
    }
  }

  for (int i = 30; i >= 0; i--) {
    writeMessage("Place probe in 7 pH \nsolution.\n" + String(i) + " s remaining.\nHold to exit");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(2500);
      ESP.restart();
    }
  }
  //Calibrate pH = 7
  //ph.cal_mid();
  writeMessage("pH=7 COMPLETE");
  delay(5000);

  writeMessage("Rinse probe.");
  delay(10000);
  writeMessage("pH CALIBRATION \nCOMPLETE");
  delay(5000);
  calibrateEC();
}

void lightControl() {
  int lightTime = rtc.getHour(true) * 100 + rtc.getMinute();
  Serial.print("LightTime: ");
  Serial.println(lightTime);
  if (lightTime >= lightOffTime) {
    if (lightState) {
      //turn lights off
      lightState = false;
    }
  }

  if (lightTime >= lightOnTime) {
    if (lightState) {
      //turn lights off
      lightState = false;
    }
  }
}

bool initTime(String timezone) {
  configTime(0, 0, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
    Serial.println("Obtained current time");
    setTimezone(timezone);
    //writeMessage(rtc.getDateTime(true));
    return true;
  }
  Serial.println("Could not obtain current time");
  return false;
}

void setTimezone(String timezone) {
  setenv("TZ", timezone.c_str(), 1);
  tzset();
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

    //If ble button has been pressed, restart to enter settings mode
    if (digitalRead(SW) == 0) {
      ESP.restart();
    }

    if (updateDB.triggered()) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot update database. WIFI not connected.");
        writeMessage(F("WIFI NOT\nCONNECTED"));
        connectWiFi();
      }
      else {
        initTime(tz);
        //Serial.println(rtc.getDateTime(true));
        //Take mutex, write url string, return mutex.
        if (commSemaphore != NULL) {
          if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
            Serial.println("Grabbing mutex in dataLoggingTask");
            String urlFinal = URL + "device=" + device + "&dtg=" + rtc.getTime("%d%b%y%H%M%S") + "&ph=" + ph + "&ec=" + ec + "&temp=" + temperature;
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
      //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      initTime(tz);
    }
  }
}

bool getSettings() {
  bool ssidSet = false;
  String adapterString;
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
  if (preferences.isKey(ecZero)) {
    ecSetting = preferences.getFloat(ecZero);
  }
  if (preferences.isKey(kValue)) {
    kVal = preferences.getFloat(kValue);
  }
  preferences.end();

  return ssidSet;
}

//void settings(int buttonPressed) {
void settings(int buttonPressed) {

  if (buttonPressed == 0) {

    simpleTimer secondTimer(1000);
    simpleTimer buttonHold(2000);
    simpleTimer exitTimer(10000);
    secondTimer.initialize();
    buttonHold.initialize();
    exitTimer.initialize();
    bool buttonPressed = true;
    bool exitTimerTriggered = false;

settingsMenu:
    writeMessage("SETTINGS\nPress to scroll.\nHold to select.");

    //Allow user to let go of button before moving on
    while (buttonPressed) {
      //if the button has been let go
      if (digitalRead(SW)) {
        buttonPressed = false;
      }
      //debounce button
      delay(200);
      if (!digitalRead(SW)) {
        buttonPressed = true;
      }
      else{
        break;
      }
    }

    //Check to see if user pushes button to begin scrolling
    exitTimer.reset();
    while (!buttonPressed) {
      if (!digitalRead(SW)) {
        buttonPressed = true;
        exitTimer.reset();
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }
    delay(500);
bluetoothMenu:
    writeMessage("BLUETOOTH");
    //delay(500);
    exitTimer.reset();
    while (true) {
      //if the button has been pressed
      if (!digitalRead(SW)) {
        exitTimer.reset();
        delay(500);
        //if the button is still being pressed
        if (!digitalRead(SW)) {
          //ble function
          ble();
          delay(1000);
          goto bluetoothMenu;
        }
        else {
          break;
        }
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }
calibratepHMenu:
    exitTimer.reset();
    writeMessage("Calibrate pH probe");
    while (true) {
      //if the button has been pressed
      if (!digitalRead(SW)) {
        exitTimer.reset();
        delay(500);
        //if the button is still being pressed
        if (!digitalRead(SW)) {
          //calibrate pH probe
          writeMessage("cal pH function");
          delay(1000);
          goto calibratepHMenu;
        }
        else {
          break;
        }
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }

calibrateECMenu:
    exitTimer.reset();
    writeMessage("Calibrate EC probe");
    while (true) {
      //if the button has been pressed
      if (!digitalRead(SW)) {
        exitTimer.reset();
        delay(500);
        //if the button is still being pressed
        if (!digitalRead(SW)) {
          //calibrate EC probe
          writeMessage("cal ec function");
          delay(1000);
          goto calibrateECMenu;
        }
        else {
          break;
        }
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }

setECMenu:
    exitTimer.reset();
    writeMessage("Set EC point");
    while (true) {
      //if the button has been pressed
      if (!digitalRead(SW)) {
        exitTimer.reset();
        delay(500);
        //if the button is still being pressed
        if (!digitalRead(SW)) {
          //set EC point
          writeMessage("set ec function");
          delay(1000);
          goto setECMenu;
        }
        else {
          break;
        }
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }

    exitTimer.reset();
    writeMessage("EXIT SETTINGS");
    while (true) {
      //if the button has been pressed
      if (!digitalRead(SW)) {
        exitTimer.reset();
        delay(500);
        //if the button is still being pressed
        if (!digitalRead(SW)) {
          writeMessage("");
          delay(1000);
          ESP.restart();
        }
        else {
          goto settingsMenu;
        }
      }
      if (exitTimer.triggered()) {
        ESP.restart();
      }
    }
  }
}

void monitorTask(void * pvParameters) {
  for (;;) {
    delay(10);
    mainPumpControl();

    check_pH();

    check_ec();

    lightControl();

  }
}

void ble(){
  bool settingsChanged = false;
        writeMessage(F("Bluetooth\nConnect app."));
        //if (buttonPressed == 0) {
        bleFlag = false;
        Serial.println("beginning SerialBT");
        Serial.println(F(bleNameString));
        if (!SerialBT.begin(bleNameString))
        {
          Serial.println(F("An error occurred initializing Bluetooth"));
          bleFlag = false;
        }
        else {
          Serial.println(F("Initialized Bluetooth"));
          bleFlag = true;
        }
        
        changeVar.reset();
        Serial.println(F("BLE button pressed!"));
        //}
        //else {
        delay(10);
        //}
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
          //ESP.restart();
        }
}

//Puts the temperature compensated ecValue into the global variable
bool check_ec() {
  if (ecTimer.triggered()) {
    writeMessage(F("Checking EC"));
    sensors.requestTemperatures();
    float temperatureReading = sensors.getTempCByIndex(0);
    //float temperatureReading = temperature;
    //Serial.print("temp: ");
    //Serial.println(temperature);
    float analogValue = analogRead(TdsSensorPin);
    float voltage = analogValue / 4096 * 3.3;
    float ecValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * kVal;
    float ecValue25  =  ecValue / (1.0 + 0.02 * (temperatureReading - 25.0)); //temperature compensation

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
  if (reading <= (ecSetting * 0.85)) {
    Serial.println(F("Adjusting nutrients"));
    writeMessage(F("Adjusting \nnutrients"));
    digitalWrite(part1, HIGH);
    delay(ratio1);
    digitalWrite(part1, LOW);
    delay(50);
    digitalWrite(part2, HIGH);
    delay(ratio2);
    digitalWrite(part2, LOW);
    delay(50);
    digitalWrite(part3, HIGH);
    delay(ratio3);
    digitalWrite(part3, LOW);
    delay(50);
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
  delay(1000);
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
