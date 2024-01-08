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
