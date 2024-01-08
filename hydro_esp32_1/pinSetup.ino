void pinSetup(){
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
  digitalWrite(lights, HIGH);
}
