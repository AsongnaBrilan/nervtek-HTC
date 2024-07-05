// declaring motor pins
#define R_DIR 14
#define R_EN 15

// declaring encoder pins
#define R_encoderA 12
#define R_encoderB 13

// variables to hold encoder direction
boolean R_direction = true; // true = forward, false = backward

// variables to hold encoder count
volatile long R_count{0};
volatile long L_count{0};
volatile long R_prevCount{0};
volatile long L_prevCount{0};

// variables to keep track of wheel distance
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F; // 1:48
const float wheel_diameter = 6.0;
const float wheel_cir = 18.8496;

float R_l = 0.0F;

// program variables
long previousMillis = 0;
long currentMillis = 0;
const unsigned long period = 50;

void setup(){
  Serial.begin(9600);

  pinMode(R_encoderA, INPUT_PULLUP);
  pinMode(R_encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(R_encoderA), interrupt_routine, RISING);

}

void loop(){
  currentMillis = millis();
  if (currentMillis > previousMillis + period){
    R_l += ((R_count - R_prevCount) / (CLICKS_PER_ROTATION * GEAR_RATIO) * wheel_cir);
    Serial.print("pulses: ");
    Serial.print(R_count);

    Serial.print("  Right:  ");
    Serial.println(R_l);

    R_prevCount = R_count;
    previousMillis = currentMillis;
  }
  
}

void interrupt_routine(){
  int Val = digitalRead(R_encoderB);

  if ( Val == LOW){
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
