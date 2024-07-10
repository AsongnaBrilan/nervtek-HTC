/*
   This is a code to interface:
   A dual encoder.
   Read Ultrasonic sensor data.
   Drive a motor.

*/

// defining motor pins
#define R_DIR 14
#define R_EN 15
#define L_DIR 6
#define L_EN 7

// defining encoder pins
#define R_encoderA 12
#define R_encoderB 13

#define L_encoderA 4
#define L_encoderB 5

volatile long  R_count{0};
volatile long  L_count{0};

boolean R_direction = true;
boolean L_direction = true;

// defining ultrasonic sensor pin
#define trig_pin 20
#define echo_pin 21
#define MAX_DISTANCE 200

int distance{0};

unsigned long currentMillis{0};
unsigned long prevMillis{0};
const unsigned long period{50};

// variables for PID control
unsigned long previousTimePID = 0;
float ePrevious = 0;
float eIntegral = 0;


void setup() {
  Serial.begin(115200);

  pinMode(R_DIR, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(L_EN, OUTPUT);

  pinMode(R_encoderA, INPUT);
  pinMode(R_encoderB, INPUT);
  pinMode(L_encoderA, INPUT);
  pinMode(L_encoderB, INPUT);

  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(R_encoderA), R_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(L_encoderB), L_interrupt, RISING);

  delay(1000);
}

void loop() {
  /*//us_measure_cm();
  Serial.print("Right Pulse: ");
  Serial.print(R_count);
  Serial.print(" Left Pulse: ");
  Serial.println(L_count);*/

  PID_control();
  
}

void us_measure_cm() {
  currentMillis = millis();
  if (currentMillis > prevMillis + period) {
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    long duration = pulseIn(echo_pin, HIGH, 38000);
    distance = duration * 0.034 / 2;

    if (distance < 2 || distance > MAX_DISTANCE) distance = MAX_DISTANCE; // range of operation of sensor.

    Serial.print("Distance: ");
    Serial.println(distance);


    prevMillis = currentMillis;
  }
}

void R_interrupt() {
  int Val = digitalRead(R_encoderB);
  if(Val == LOW){
    R_direction = false;
  }

  else{
    R_direction = true;
  }

  if (R_direction){
    R_count++;
  }
  else{
    R_count--;
  }
}

void L_interrupt() {
  int Val = digitalRead(L_encoderA);
  if(Val == LOW){
    L_direction = false;
  }

  else{
    L_direction = true;
  }

  if (L_direction){
    L_count++;
  }
  else{
    L_count--;
  }
}

void PID_control(){
  // set point 
  int target = 1000; // in cm

  // PID gains and computation
  float kp = 2.0;
  float kd = 0.1;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);

  moveMotor(R_DIR, R_EN, u);

  //for debugging
  Serial.print(target); 
  Serial.print(", ");
  Serial.println(R_count);
}

float pidController(int target, float kp, float kd, float ki){
  long currentTimePID = micros();
  float deltaT = ((float)(currentTimePID - previousTimePID)) / 1.0e6;

  // compute error, derivative and integral for both motors
  int e = R_count - target;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  previousTimePID = currentTimePID;
  ePrevious = e;

  return u; 
  
}

// Function to move motors
void moveMotor(int dirPin, int pwmPin, float u){
  // maximum motor speed
  float Speed = fabs(u);
  if (Speed > 255){
    Speed = 255;
  }

  // set the direction
  int R_Direction = 1;
  if (u < 0){
    R_Direction = 0;
  }

  //control motor
  digitalWrite(dirPin, R_direction);
  analogWrite(pwmPin, Speed);
}
