#define STEP1 18
#define DIR1  21
#define STEP2 25
#define DIR2  26
#define STEP3 13
#define DIR3  22

void stepMotor(int stepPin, int dirPin, bool dir, int steps, int speedUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedUs);
  }
}

int pos1 = 0, pos2 = 0, pos3 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(STEP3, OUTPUT); pinMode(DIR3, OUTPUT);
  digitalWrite(STEP1, LOW); digitalWrite(DIR1, LOW);
  digitalWrite(STEP2, LOW); digitalWrite(DIR2, LOW);
  digitalWrite(STEP3, LOW); digitalWrite(DIR3, LOW);

  Serial.println("=== Motor Test ===");
  Serial.println("1+ 1- = Motor 1 up/down 100 steps");
  Serial.println("2+ 2- = Motor 2 up/down 100 steps");
  Serial.println("3+ 3- = Motor 3 up/down 100 steps");
  Serial.println("R = reset all to 0");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if      (cmd == "1+") { stepMotor(STEP1,DIR1,false,100,500); pos1-=100; }
    else if (cmd == "1-") { stepMotor(STEP1,DIR1,true, 100,500); pos1+=100; }
    else if (cmd == "2+") { stepMotor(STEP2,DIR2,false,100,500); pos2-=100; }
    else if (cmd == "2-") { stepMotor(STEP2,DIR2,true, 100,500); pos2+=100; }
    else if (cmd == "3+") { stepMotor(STEP3,DIR3,false,100,500); pos3-=100; }
    else if (cmd == "3-") { stepMotor(STEP3,DIR3,true, 100,500); pos3+=100; }
    else if (cmd == "R") {
      if (pos1!=0) { stepMotor(STEP1,DIR1,pos1<0,abs(pos1),500); pos1=0; }
      if (pos2!=0) { stepMotor(STEP2,DIR2,pos2<0,abs(pos2),500); pos2=0; }
      if (pos3!=0) { stepMotor(STEP3,DIR3,pos3<0,abs(pos3),500); pos3=0; }
      Serial.println("All reset to 0");
    }

    Serial.printf("M1:%d  M2:%d  M3:%d\n", pos1, pos2, pos3);
  }
}