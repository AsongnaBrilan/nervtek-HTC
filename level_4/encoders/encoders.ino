// define pin numbers for motor
#define R_DIR 14
#define R_EN 15

// define pin numbers of encoders
#define R_encoderPinA 12
#define R_encoderPinB 13
volatile long encoderCount = 0;


// PID program variables
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

void setup(){
  Serial.begin(9600);

  pinMode(R_DIR, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_encoderPinA, INPUT);
  pinMode(R_encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(R_encoderPinA), R_handleEncoder, RISING); // interrupt for right encoder.
}

void loop(){
  //setpoint 
  /*int target = 1000;

  //PID gains and computations
  float kp = 2.0;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);

  //control motor
  moveMotor(R_DIR, R_EN, u);

  //print statements for debugging
  Serial.print(target);
  Serial.print(", ");
  Serial.println(encoderCount);*/
  motorTest();
}

// function called during interrupts
void R_handleEncoder(){
  if (digitalRead(R_encoderPinA) > digitalRead(R_encoderPinB)){
    encoderCount++;
  }

  else{
    encoderCount--;
  }
}

void moveMotor(int dirPin, int pwmPin, float u){
  // max motor speed
  float Speed = fabs(u);
  if (Speed > 255){
    Speed = 255;
  }

  //set the direction
  int Direction = 1;
  if (u < 0){
    Direction = 0;
  }

  // control the motor
  digitalWrite(dirPin, Direction);
  analogWrite(pwmPin, Speed);
}

float pidController(int target, float kp, float kd, float ki){
  // measure the time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 0.0e6;

  //compute the error, derivative, and integral
  int e = encoderCount - target;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  //compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  //update variables for the next iteration
  previousTime = currentTime;
  ePrevious = e;

  return u;
}

void motorTest(){
  digitalWrite(R_EN, LOW);
  digitalWrite(R_DIR, LOW);
}
