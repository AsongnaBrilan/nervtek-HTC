// defining motor pins
int R_DIR = 14;
int R_EN = 15;
int L_DIR = 6;
int L_EN = 7;

//defining the encoder pins
int R_encoderA = 12;
int R_encoderB = 13;

int L_encoderA = 4;
int L_encoderB = 5;

int R_pulseCount = 0;
int L_pulseCount = 0;

boolean R_direction = false; // forward = true, backward = false;
boolean L_direction = false;

// defining pin for UT sensor
int trig_pin = 20;
int echo_pin = 21;

// programming variables
float distance, duration;

const int MAX_DISTANCE = 5; // this is 2cm = 2m

unsigned long currentTime{0};
unsigned long prevTime{0};
const int PERIOD = 50; // this means 50 milliseconds

// PID control variables
unsigned long previousTimePID = 0;
float ePrevious = 0;
float eIntegral = 0;
void setup() {
  Serial.begin(9600);
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

}

void loop() {

  currentTime = millis();

  /*motorTest();
    Serial.print("R_pulse: ");
    Serial.print(R_pulseCount);
    Serial.print(" L_pulse: ");
    Serial.println(L_pulseCount);*/
  if (currentTime > prevTime + PERIOD) {
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(10);

    duration = pulseIn(echo_pin, HIGH);

    distance = duration * 0.034 / 2;

    if (distance < 2) distance = MAX_DISTANCE;
    //if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    Serial.print("distance: ");
    Serial.println( distance);

    prevTime = currentTime;
  }

  //if (distance == MAX_DISTANCE) Drive(100, 100, 0, 1); //Forward
  //if (distance > MAX_DISTANCE) Drive(90, 0, 0, 1); //Right
  //if (distance < MAX_DISTANCE) Drive(0, 90, 0, 1); //LEFT

  PID_control();

}

void R_interrupt() {
  //R_pulseCount++;
  int R_val = digitalRead(R_encoderB);

  if (R_val == LOW) {
    R_direction = false; // backward
  }

  else {
    R_direction = true;
  }

  if (R_direction) {
    R_pulseCount++;
  }

  else {
    R_pulseCount--;
  }
}

void L_interrupt() {
  //L_pulseCount++;
  int L_val = digitalRead(L_encoderA);

  if (L_val == LOW) {
    L_direction = false; // backward
  }

  else {
    L_direction = true;
  }

  if (L_direction) {
    L_pulseCount++;
  }

  else {
    L_pulseCount--;
  }
}

void Drive(int R_speed, int L_speed, int r_dir, int l_dir) {
  digitalWrite(R_DIR, r_dir); // spins forward or backward
  digitalWrite(L_DIR, l_dir);
  analogWrite(R_EN, R_speed);
  analogWrite(L_EN, L_speed);// turns the microcontroller
}

void PID_moveMotor(int R_dir, int L_dir, int R_en, int L_en, float u) {
  // maximum motor speed
  float Speed = fabs(u);
  if (Speed > 255) {
    Speed = 255;
  }

  // set the direction
  int R_direction = 0;
  int L_direction = 0;
  if (u < 0) {
    R_direction = 1;
    L_direction = 0;
  }

  //control motor
  digitalWrite(R_dir, R_direction);
  analogWrite(R_en, Speed);
  digitalWrite(L_dir, L_direction);
  analogWrite(R_en, Speed);
}

void ut_measurment() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(10);

  duration = pulseIn(echo_pin, HIGH);

  distance = duration * 0.034 / 2;

  if (distance < 2) distance = 2;
  if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
  Serial.println(distance);

  //return distance;
}

void PID_control() {
  //PID gains

  float kp = 0;
  float ki = 0;
  float kd = 0;

  float u = pidController(MAX_DISTANCE, kp, ki, kd);

  Drive(80, 80, 0, 1);
}

float pidController(int target, float kp, float ki, float kd) {
  currentTimePID = micros();
  float deltaT = ((float)(currentTimePID - previousTimePID)) / 1.0e6;

  // compute error, derivative and integral for both motors
  int e = distance - MAX_DISTANCE;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  previousTimePID = currentTimePID;
  ePrevious = e;

  return u;

}
