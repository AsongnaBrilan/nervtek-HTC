/*
 * Encoder puslse counting
*/

 #define R_encoderA 12

 volatile long encoder_count{0};

 void setup(){
  Serial.begin(9600);
  pinMode(R_encoderA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(R_encoderA), interruptHandle, RISING);
  
 }

 void loop(){
  Serial.print(" Pulses: ");
  Serial.println(encoder_count);
}

void interruptHandle(){
  encoder_count++;
}
