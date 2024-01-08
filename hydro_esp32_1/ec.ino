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
