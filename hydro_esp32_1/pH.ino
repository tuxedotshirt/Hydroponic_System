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

void pHSetup(){
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
