const int echo_pin = 21;
const int trig_pin = 20;

const int max_distance = 200; 

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long period = 100;

int distance = 0;

void setup(){
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
}

void loop(){
  usReadCm();
}

void usReadCm(){
  currentMillis = millis();
  if (currentMillis > prevMillis + period){
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    // read the echo pin, returns the sound travel time in microseconds
    // note the duration (38000 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(echo_pin, HIGH, 38000);

    distance  = duration * 0.034 / 2; // time of flight equation...

    //applying limits
    if (distance > max_distance) distance = max_distance;
    if (distance == 0) distance = max_distance;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm ");

    prevMillis = currentMillis;
  }
}
