#define trig 14
#define echo 15

int duration, distance;

void setup(){
  Serial.begin(115200);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig, LOW);
  delayMicroseconds(10);

  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.034) / 2;
  Serial.println(distance);
}
