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
