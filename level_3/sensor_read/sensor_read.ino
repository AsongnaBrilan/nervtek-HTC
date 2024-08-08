/*
  Sample code to test the line sensor in the LF-2 robot
*/

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue1 = analogRead(A1);
  int sensorValue2 = analogRead(A2);
  int sensorValue3 = analogRead(A3);
  int sensorValue4 = analogRead(A4);
  int sensorValue5 = analogRead(A5);

  // print out the value you read:

  Serial.print(sensorValue1);
  Serial.print("   ");
  Serial.print(sensorValue2);
  Serial.print("   ");
  Serial.print(sensorValue3);
  Serial.print("   ");
  Serial.print(sensorValue4);
  Serial.print("   ");
  Serial.println(sensorValue5);
  delay(50);
}
