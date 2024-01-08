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
