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
