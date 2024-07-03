// define pin numbers for motor
#define DIR 
#define EN 

// define pin numbers of encoders
#define encoderPinA 
#define encoderPinB
volatile long encoderCount = 0;


// PID program variables
long prevTime = 0;
float ePrev = 0;
float eIntergral = 0;

void setup(){
  Serial.begin(9600);

  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
}

void loop(){
  //setpoint 
  int target = 1000;

  //PID gains and computations
  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);

  //control motor
  moveMotor(DIR, EN, u);

  //print statements for debugging
  Serial.print(target);
  Serial.print(", ");
  Serial.println(encoderCount);
}

// function called during interrupts
void handleEncoder(){
  if (digitalRead(encoderPinA) > digitalRead(encoderPinB)){
    encoderCount++
  }

  else{
    encoderCount--;
  }
}

void moveMotor(int dirPin, int pwmPin, float u){
  // max motor speed
  float speed = fabs(u);
  if (speed > 255){
    speed = 255;
  }

  //set the direction
  int direction = 1;
  if (u < 0){
    direction = 0;
  }

  // control the motor
  digitalWrite(dirPin, direction);
  analogWrite(pwmPin, speed);
}

void pidController(int target, float kp, float kd, float ki){
  // measure the time elapsed since the last iteration
  long currentTine = micros();
  float deltaT = ((float)(currentTime - preTime)) / 0.0e6;

  //compute the error, derivative, and integral
  int e = encoderCount - target;
  float eDerivative = (e - ePrev) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  //compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  //update variables for the next iteration
  preTime = currentTime;
  ePrevious = e;

  return u;
}
