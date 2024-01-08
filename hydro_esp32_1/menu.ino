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
