#define R_DIR 14
#define R_EN 15

#define R_encoderA 12
#define R_encoderB 13

boolean R_direction = true; // true = forward, false = backward

volatile long R_count{0};
volatile long L_count{0};

const unsigned long period = 50;

long previousMillis = 0;
long currentMillis = 0;

void setup(){
  Serial.begin(9600);

  pinMode(R_encoderA, INPUT_PULLUP);
  pinMode(R_encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(R_encoderA), interrupt_routine, RISING);

}

void loop(){
  Serial.print("pulses: ");
  Serial.println(R_count);
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
