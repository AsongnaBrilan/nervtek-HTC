/*
  Sample code to test your LF-2 Robot motor connections.
  This code will make your robot go forward for 1 second and stop for 1 second.
  In case of a different behaviour you will have to switch the motor connections.
*/

#include <SparkFun_TB6612.h>

#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define PWMA 5
#define PWMB 6

#define ENA 5
#define ENB 6
//#define STBY 5

const int offsetA = 1;
const int offsetB = -1; // - = forward

Motor motor1 = Motor(IN1, IN2, PWMA, offsetA, ENA);
Motor motor2 = Motor(IN3, IN4, PWMB, offsetB, ENB);
void setup() {
  Serial.begin(9600);
}

void loop() {
  /*for (int i = 0; i <= 255; i++) {
    motor1.drive(i);
    motor2.drive(i);
    Serial.println(i);
    delay(50);
  }*/
  
  motor1.drive(120);
  motor2.drive(120);
  delay(1000);

  motor1.drive(0);
  motor2.drive(0);
  delay(2000);
}
