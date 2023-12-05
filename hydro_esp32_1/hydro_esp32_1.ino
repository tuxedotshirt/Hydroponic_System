
/*TODO:
    send reset request over ble for factory settings
    Save deploymentID for DB from app to construct the URL string
    Add nutrient ratios to app
    Add manual nutrient ratio
    Add light on/off times to app
    Add manual time entry and tz compensation to app
    ecPref, phPref, kvalue, p1, p2 and p3 mutex to make sure the global isn't being accessed at the same time.
    check pH reading validity before calling pumps.
*/

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
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
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
//ESP32Time rtc;
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

//#define SW 19

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
  pinMode(waterLevel, INPUT);

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
  mainPumpTimer.initialize();
  timeCheckTimer.initialize();

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
  digitalWrite(lights, HIGH);
  xTaskCreatePinnedToCore(monitorTask, "monitorTask", 10000, NULL, 1, &monitorCore, 1); //Run on core 1, core 0 is for communication.
  xTaskCreatePinnedToCore(dataLoggingTask, "dataLoggingTask", 10000, NULL, 1, &dataLogging, 0); //Run on core 0.

  //recommended by Atlas Scientific
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);
  pH.send_read_cmd();
  delay(1000);
  receive_and_print_reading(pH);
  
}

void lightControl() {
  int lightTime = rtc.getHour(true) * 100 + rtc.getMinute();
  Serial.println(lightTime);
  if (lightTime >= lightOnTime && lightTime <= lightOffTime) {
    //if the light pin is off
    //Serial.println("Turning lights on");
    if (!digitalRead(lights)) {
      //turn lights on
      digitalWrite(lights, HIGH);
    }
  }
  else {
    //if the light pin is on
    //Serial.println("Turning lights off");
    if (digitalRead(lights)) {
      //turn lights off
      digitalWrite(lights, LOW);
    }
  }
}

bool initTime(String timezone) {
  bool success = true;
  configTime(0, 0, ntpServer);
  struct tm timeinfo;
  /////
  Serial.print("Obtaining current time.");
  wifiTimer.reset();

  
  if (!getLocalTime(&timeinfo)) {
    while (!getLocalTime(&timeinfo))
    {
      
      delay(1000);
      Serial.print(".");
      //Try for 20 seconds
      if (wifiTimer.triggered()) {
        success = false;
        break;
      }
    }
  }
  if (success) {
    rtc.setTimeStruct(timeinfo);
    setTimezone(timezone);
    Serial.println("SUCCESS");
  }
  else {
    Serial.println("FAILED");
  }

  return success;
  /////
  /*
    bool success = false;
    configTime(0, 0, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
    Serial.println("Obtained current time");

    setTimezone(timezone);
    Serial.println(rtc.getDateTime(true));
    //writeMessage(rtc.getDateTime(true));
    //return true;
    success = true;
    }
    else {
    Serial.println("Could not obtain current time");
    //return false;
    success = false;
    }
    return success;
  */
}

void setTimezone(String timezone) {
  setenv("TZ", timezone.c_str(), 1);
  tzset();
}

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

void connectWiFi() {
  //if (getSettings()) {

  wifiTimer.reset();
  if (WiFi.status() != WL_CONNECTED) {
    if (getSettings()) {
      WiFi.begin(ssidArr, passArr);
      writeMessage(F("Connecting WIFI"));
      while (WiFi.status() != WL_CONNECTED)
      {

        delay(500);
        Serial.print(F("."));
        //Try for 20 seconds
        if (wifiTimer.triggered()) {
          Serial.println();
          break;
        }
      }
      Serial.println();
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected!");
        writeMessage(F("WIFI Connected!"));
        initTime(tz);
      }
    }
  }
  else {
    Serial.println("WiFi Connection good");
  }
}

bool getSettings() {
  bool gotMutex = false;
  String adapterString;
  free(ssidArr); //Free allocated memory
  free(passArr); //Free allocated memory

  if (commSemaphore != NULL) {
    if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 1000 / portTICK_PERIOD_MS) == pdTRUE ) {
      preferences.begin("WiFiLogin", false);
      gotMutex = true;
      Serial.println(F("Grabbing mutex in getSettings"));

      if (preferences.isKey(ssidPref)) {
        adapterString = preferences.getString(ssidPref);
        adapterLength = adapterString.length() + 1;
        ssidArr = (char*) malloc (adapterLength);
        adapterString.toCharArray(ssidArr, adapterLength);
      }
      if (preferences.isKey(pwdPref)) {
        adapterString = preferences.getString(pwdPref);
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
      if (preferences.isKey(bleNamePref)) {
        bleNameString = "";
        bleNameString = preferences.getString(bleNamePref);
      }
      if (preferences.isKey(kValue)) {
        kVal = preferences.getFloat(kValue);
      }
      if (preferences.isKey(lightOn)) {
        lightOnTime = preferences.getInt(lightOn);
      }
      if (preferences.isKey(lightOff)) {
        lightOffTime = preferences.getInt(lightOff);
      }
      if (preferences.isKey(p1)) {
        ratio1 = preferences.getFloat(p1);
      }
      if (preferences.isKey(p2)) {
        ratio2 = preferences.getFloat(p2);
      }
      if (preferences.isKey(p3)) {
        ratio3 = preferences.getFloat(p3);
      }
      if (preferences.isKey(ecCalibrateTemp)) {
        calibratedTemperature = preferences.getFloat(ecCalibrateTemp);
      }
//      if (preferences.isKey(yIntercept)) {
//        tempYIntercept = preferences.getFloat(yIntercept);
//      }
//      if (preferences.isKey(slope)) {
//        tempSlope = preferences.getFloat(slope);
//      }
      preferences.end();
      xSemaphoreGive(commSemaphore);
      Serial.println(F("Returned mutex in getSettings"));
      settingsChanged = false;
    }
    else {
      Serial.println("Could not get mutex in getSettings");
    }
  }
  return gotMutex;
}

void monitorTask(void * pvParameters) {

  for (;;) {
    delay(10);
    //printDiagnostics();
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

void check_ec() {
  if (ecTimer.triggeredNoReset()) {
    delay(sensorReadDelay);
    float tempSetting = 0.0;
    writeMessage(F("Checking EC"));
    sensors.requestTemperatures();
    float temperatureReading = sensors.getTempCByIndex(0);
    float calTemp = 0.0;
    float analogValue = 0.0;
    float voltage = 0.0;
    float ecValue = 0.0;
    float ecValue25 = 0.0;

    if (temperatureReading == DEVICE_DISCONNECTED_C) {
      temperatureReading = 25.0;
    }
    if (commSemaphore != NULL) {
      if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 1000 / portTICK_PERIOD_MS) == pdTRUE ) {
        analogValue = analogRead(TdsSensorPin);
        voltage = analogValue / 4096 * 3.3;
        ecValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * kVal;
        ecValue25  =  ecValue / (1.0 + 0.02 * (temperatureReading - calibratedTemperature)); //temperature compensation
        Serial.print("EC Reading: ");
        Serial.println(ecValue25);
        //if the EC probe has a valid reading
        if (ecValue25 != 0.0) {
          ec = ecValue25;
          temperature = temperatureReading;
          tempSetting = ecSetting;
          ecTimer.reset();
          waterLow = false;
          xSemaphoreGive(commSemaphore);
          ecPumpControl(ecValue25, tempSetting);
        }
        else {
          xSemaphoreGive(commSemaphore);
          Serial.println("WATER LOW or EC ERROR");
          waterLow = true;
        }

      }
      else {
        Serial.println("Could not get mutex in check_ec()");
      }
    }
  }
}

void check_pH() {
  if (phTimer.triggeredNoReset()) {
    delay(sensorReadDelay);
    //float tempSetting = 0.0;
    //float adjustedReading = 0.0;

    delay(1000);
    pH.send_read_cmd();
    display.clearDisplay();
    writeMessage(F("Checking pH"));
    delay(2000);
    Serial.print("pH reading: ");
    receive_and_print_reading(pH);             //get the reading from the PH circuit
    Serial.println();
    float tempReading = pH.get_last_received_reading();

    //if we got a valid reading
    if (pH.get_error() == Ezo_board::SUCCESS) {
      if (commSemaphore != NULL) {
        if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 1000 / portTICK_PERIOD_MS) == pdTRUE ) {
          Serial.println(F("Grabbing mutex in check_pH"));
          //adjustedReading = (tempReading - tempYIntercept) / tempSlope;
          //Serial.print("Adjusted Reading: ");
          //Serial.println(adjustedReading);
          Serial.print("phSetting: ");
          Serial.println(phSetting);
          ph = tempReading;
          //ph = adjustedReading;
          //tempSetting = phSetting;
          xSemaphoreGive(commSemaphore);
          Serial.println(F("Returned mutex in check_pH"));
          phTimer.reset();
          phPumpControl(tempReading, phSetting);
        }
        else {
          Serial.println(F("Could not grab mutex in check_pH"));
        }
      }
      display.clearDisplay();
    }
    else {
      writeMessage(F("Could not \nread pH"));
      Serial.println(F("ERROR: Could not read pH"));
    }
  }
}

//blocking function, do not want to take readings from sensors with pumps running
void ecPumpControl(float reading, float setting) {
  if (!waterHigh && nutrientCounter <= nutrientCounterLimit) {
    if (reading <= (setting * 0.85)) {
      nutrientCounter++;
      //waterLow = false;
      Serial.println(F("Adjusting nutrients"));
      writeMessage(F("Adjusting \nnutrients"));
      digitalWrite(part1, HIGH);
      delay(ratio1);
      digitalWrite(part1, LOW);

      digitalWrite(circulation, HIGH);
      delay(60000); //1 minute mixing time
      digitalWrite(circulation, LOW);

      digitalWrite(part2, HIGH);
      delay(ratio2);
      digitalWrite(part2, LOW);

      digitalWrite(circulation, HIGH);
      delay(60000); //1 minute mixing time
      digitalWrite(circulation, LOW);

      digitalWrite(part3, HIGH);
      delay(ratio3);
      digitalWrite(part3, LOW);
      delay(50);
      mainPumpControl();
      display.clearDisplay();
    }
    else {
      nutrientCounter = 0;
    }
  }
  else {
    Serial.println("Water is too high to add nutrients");
    display.clearDisplay();
    writeMessage(F("Could not adjust\nnutrients."));
  }
}

//blocking function, do not want to take readings from sensors with pumps running
void phPumpControl(float reading, float setting) {
  if (!waterHigh && phChemCounter <= phChemCounterLimit) {
    if (reading <= (setting - pHTolerance)) {
      phChemCounter++;
      
      Serial.println("Adjusting pH up");
      writeMessage(F("Adjusting pH\nup"));
      //phTimer.setInterval(phAdjustInterval);
      digitalWrite(pHUp, HIGH);
      delay(pHPumpOn);
      digitalWrite(pHUp, LOW);
      display.clearDisplay();
      //mainPumpControl();

      digitalWrite(circulation, HIGH);
      delay(1000*60*10); //10 minute mixing time
      digitalWrite(circulation, LOW);

      display.clearDisplay();
    }
    else if (reading >= (setting +  pHTolerance)) {
      phChemCounter++;
      
      Serial.println("Adjusting pH down");
      writeMessage(F("Adjusting pH\ndown"));
      //phTimer.setInterval(phAdjustInterval);
      digitalWrite(pHDown, HIGH);
      delay(pHPumpOn);
      digitalWrite(pHDown, LOW);
      display.clearDisplay();
      //mainPumpControl();
      digitalWrite(circulation, HIGH);
      delay(1000*60*10); //10 minute mixing time
      digitalWrite(circulation, LOW);

      display.clearDisplay();
    }
    else {
      //phTimer.setInterval(phInterval);
      //phChemCounter = 0;
    }
  }
  else {
    Serial.println("Water is too high or limit exceeded.");
    display.clearDisplay();
    writeMessage(F("Could not\nadjust pH."));
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

void setEC() {
  writeMessage("Place probes in\nnutrient solution\nto set desired EC.");
  delay(5000);
  for (int i = 30; i >= 0; i--) {
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
  preferences.putFloat(ecPref, ecValue25);
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
  float rawECsolution = 2000;
  //float voltage = 3.3;

  writeMessage("EC Calibration\nRelease button.");
  delay(5000);
  writeMessage("Place probes in \n2000uS buffer\nsolution");
  delay(10000);
  for (int i = 30; i >= 0; i--) {
    writeMessage("Reading. Push button\nto exit.\n" + String(i) + " seconds remaining");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(5000);
      ESP.restart();
    }
  }
  sensors.requestTemperatures();
  float temperatureReading = sensors.getTempCByIndex(0);
  Serial.print("temperature: ");
  Serial.println(temperatureReading);
  float analogValue = analogRead(TdsSensorPin);
  float voltage = analogValue / 4096 * 3.3;
  //rawECsolution = rawECsolution * (1.0 + 0.02 * (temperatureReading - 25.0));
  KValueTemp = rawECsolution / (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage); //calibrate in the  buffer solution, EC 2000uS, NaCl 1000ppm
  if ((rawECsolution > 0) && (rawECsolution <= 2000) && (KValueTemp > 0.25) && (KValueTemp < 4.0))
  {
    Serial.print(KValueTemp);
    preferences.begin("WiFiLogin", false);
    preferences.putFloat(kValue, KValueTemp);
    preferences.putFloat(ecCalibrateTemp, temperatureReading);
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

void calibratePH() {


  writeMessage("pH Calibration\nRelease button");
  delay(5000);

  for (int i = 30; i >= 0; i--) {
    writeMessage("Place probe in\n7 pH solution.\n" + String(i) + " seconds remaining.\nHold to exit.");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(2500);
      ESP.restart();
    }
  }
  //Calibrate pH = 7
  writeMessage(F("Calibrating..."));
  delay(1000);
  pH.send_cmd("Cal,mid,7.00");
  Serial.println("Cal,mid,7.00");
  delay(2000);
  receive_reading(pH);
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
    writeMessage("Place probe in 4 pH \nsolution.\n" + String(i) + " s remaining.\nHold to exit");
    delay(1000);
    if (!digitalRead(SW)) {
      writeMessage("Exiting");
      delay(2500);
      ESP.restart();
    }
  }
  //Calibrate pH = 4
  writeMessage(F("Calibrating..."));
  delay(1000);
  pH.send_cmd("Cal,low,4.00");
  Serial.println("Cal,low,4.00");
  delay(2000);
  receive_reading(pH);
  delay(5000);

  writeMessage("Rinse probe.");
  delay(10000);
  writeMessage("pH CALIBRATION \nCOMPLETE");
  delay(5000);
}

void receive_reading(Ezo_board &Sensor)
{
  Sensor.send_read_cmd();
  switch (Sensor.get_error())                          //switch case based on what the response code is.
  {
    case Ezo_board::SUCCESS:
      //Serial.println(Sensor.get_last_received_reading());               //the command was successful, print the reading
      Serial.println("Success");
      writeMessage("Calibration\nSUCCESS");
      break;
 
    case Ezo_board::FAIL:
      Serial.print("Failed ");                          //means the command has failed.
      break;
 
    case Ezo_board::NOT_READY:
      Serial.print("Pending ");                         //the command has not yet been finished calculating.
      break;
 
    case Ezo_board::NO_DATA:
      Serial.print("No Data ");                         //the sensor has no data to send.
      break;
  }
}
//give user 30s to place probe in solution, 30s to rinse and place in next solution
/*void calibratePH() {
  float pH4 = 4.0;
  float pH7 = 7.0;
  float pH4Reading = 0.0;
  float pH7Reading = 0.0;
  float m = 0.0;
  float b = 0.0;

  writeMessage("pH Calibration\nRelease button");
  delay(5000);

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
  pH.send_read_cmd();
  writeMessage(F("Checking pH"));
  delay(1000);
  receive_and_print_reading(pH);             //get the reading from the PH circuit
  Serial.println();
  pH4Reading = pH.get_last_received_reading();
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
  pH.send_read_cmd();
  writeMessage(F("Checking pH"));
  delay(1000);
  receive_and_print_reading(pH);             //get the reading from the PH circuit
  Serial.println();
  pH7Reading = pH.get_last_received_reading();
  writeMessage("pH=7 COMPLETE");
  delay(5000);

  m = (pH7Reading - pH4Reading) / (7.0 - 4.0);

  b = -1 * ((m * 4.0) - pH4Reading);
  preferences.begin("WiFiLogin", false);
  preferences.putFloat(slope, m);
  preferences.putFloat(yIntercept, b);
  preferences.end();

  writeMessage("Rinse probe.");
  delay(10000);
  writeMessage("pH CALIBRATION \nCOMPLETE");
  delay(5000);
}*/

//memory leak when navigation back to settings()? Functions don't return.
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
      else {
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
          calibratePH();
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
          calibrateEC();
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

void ble() {
  writeMessage(F("Bluetooth\nConnect app."));

  bleFlag = false;
  Serial.println("beginning SerialBT");
  Serial.println(bleNameString);
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

  delay(10);

  while (bleFlag == true) {
    delay(50);

    //timer to exit bluetooth mode
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
      }
      ssidArr = strtok(tempArr, ",");
      if (String(ssidArr) != "?") {
        preferences.putString(ssidPref, String(ssidArr));
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
        phSetting = preferences.getFloat(pHPref);
        bleFlag = false;
        settingsChanged = true;
      }
      ecTemp = atoi(strtok(NULL, ","));
      if (ecTemp > 0.00) {
        preferences.putFloat(ecPref, ecTemp);
        ecSetting = preferences.getFloat(ecPref);
        bleFlag = false;
        settingsChanged  = true;
      }
      preferences.end();
    }
  }
}

bool printDiagnostics() {
  bool gotMutex = false;
  String adapterString;
  free(ssidArr); //Free allocated memory
  free(passArr); //Free allocated memory

  if (commSemaphore != NULL) {
    Serial.println("-------------------------------------------------------------------------------");
    if ( xSemaphoreTake( commSemaphore, ( TickType_t ) 1000 / portTICK_PERIOD_MS) == pdTRUE ) {
      preferences.begin("WiFiLogin", false);
      gotMutex = true;
      Serial.println(F("Grabbing mutex in printDiagnostics"));
      if (preferences.isKey(pHPref)) {
        Serial.print("pH Setting: ");
        Serial.println(preferences.getFloat(pHPref));
      }
      Serial.print("Last pH reading: ");
      Serial.println(ph);

      if (preferences.isKey(ecPref)) {
        Serial.print("EC Setting: ");
        Serial.println(preferences.getFloat(ecPref));
      }
      Serial.print("Last EC reading: ");
      Serial.println(ec);

      if (preferences.isKey(lightOn)) {
        Serial.print("Light ON time: ");
        Serial.println(preferences.getInt(lightOn));
      }
      if (preferences.isKey(lightOff)) {
        Serial.print("Light OFF time: ");
        Serial.println(preferences.getInt(lightOff));
      }
      if (preferences.isKey(p1)) {
        Serial.print("Part 1 ratio time: ");
        Serial.println(preferences.getFloat(p1));
      }
      if (preferences.isKey(p2)) {
        Serial.print("Part 2 ratio time: ");
        Serial.println(preferences.getFloat(p2));
      }
      if (preferences.isKey(p3)) {
        Serial.print("Part 3 ratio time: ");
        Serial.println(preferences.getFloat(p3));
      }
      preferences.end();
      xSemaphoreGive(commSemaphore);
      Serial.println(F("Returned mutex in printDiagnostics"));
    }
    else {
      Serial.println("Could not get mutex in printDiagnostics");
    }
    Serial.println("-------------------------------------------------------------------------------");
  }

  return gotMutex;
}
