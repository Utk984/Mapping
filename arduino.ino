//your pin definitions
char fromPi;
int motR_B=8;
int motR_F=9;
int motL_B=10;
int motL_F=11;
int trig = 5;
int echo = 6;
float time;
float dist;
void setup() 
{
  pinMode(motR_F, OUTPUT);
  pinMode(motR_B, OUTPUT);
  pinMode(motL_F, OUTPUT);
  pinMode(motL_B, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  digitalWrite(motR_F, LOW);
  digitalWrite(motR_B, LOW);
  digitalWrite(motL_B, LOW);
  digitalWrite(motL_F, LOW); 
  Serial.begin(9600);
}

void left() {
  digitalWrite(motR_F, HIGH);
  digitalWrite(motR_B, LOW);
  digitalWrite(motL_B, HIGH);
  digitalWrite(motL_F, LOW); 
}

void right() {
  digitalWrite(motR_B, HIGH);
  digitalWrite(motR_F, LOW);
  digitalWrite(motL_F, HIGH);
  digitalWrite(motL_B, LOW); 
}
void forward() {
  digitalWrite(motR_F, HIGH);
  digitalWrite(motR_B, LOW);
  digitalWrite(motL_F, HIGH);
  digitalWrite(motL_B, LOW);
}

void backward() {
  digitalWrite(motR_F, LOW);
  digitalWrite(motR_B, HIGH);
  digitalWrite(motL_F, LOW);
  digitalWrite(motL_B, HIGH); 
}
void stopmotors(){
  digitalWrite(motR_F, LOW);
  digitalWrite(motR_B, LOW);
  digitalWrite(motL_F, LOW);
  digitalWrite(motL_B, LOW); 
}
void loop() {
  float totalDistance=0.0;
  if (Serial.available() > 0) {
    fromPi = Serial.read();
    if  (fromPi == 'T') {
      for (int i = 0; i < 100; ++i) {
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        time = pulseIn(echo, HIGH);
        dist = 0.017 * time;
        totalDistance += dist;
      }
      float avgdist=totalDistance/100;
      if (avgdist<400)
        Serial.println(avgdist);
    }
    else if  (fromPi == 'W') {
      forward();
    }
    else if (fromPi== 'S') {
      backward();
    }
    else if (fromPi== 'D') {
      right();
    }
    else if (fromPi== 'A') {
      left();
    }
    else if (fromPi== 'X') {
      stopmotors();
    }
    else{
      stopmotors();
    }
  }
  Serial.flush();
}
