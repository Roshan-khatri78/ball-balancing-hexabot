#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// LCD Setup
#define SDA_PIN 4
#define SCL_PIN 15
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Button Pins
#define BTN_STOP   27
#define BTN_BACK   13
#define BTN_RIGHT  33
#define BTN_OK     26
#define BTN_LEFT   14

// Servo Pins
#define SERVO1_PIN 21
#define SERVO2_PIN 22
#define SERVO3_PIN 23
Servo servo1, servo2, servo3;

// App States
enum State {
  WELCOME,
  MODE_SELECT,
  STATIC_ADJUST_X,
  STATIC_ADJUST_Y,
  STATIC_MOVE,
  DYNAMIC_SELECT,
  DYNAMIC_RUN,
  THANK_YOU
};
State currentState = WELCOME;

// Debounce
unsigned long lastButtonTime = 0;
const unsigned long debounceDelay = 200;

// Menu
int dynamicIndex = 0;
int modeSelectIndex = 0;
const String shapes[] = {"Circle","Triangle","Square","Infinity","Hexagon","Ellipse"};
const int shapeCount = sizeof(shapes)/sizeof(shapes[0]);

// Ball & PID
float ballX=0, ballY=0;
float targetX=0, targetY=0;
float errorX, errorY, pidX, pidY;
float prevErrorX=0, prevErrorY=0; // Store previous errors
float integral_x=0, integral_y=0, derivative_x=0, derivative_y=0; // PID components

// Timing
unsigned long lastMoveTime=0;
const float adjustDelta=0.1;
const float PID_MULTIPLIER=15.0;

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init(); lcd.backlight();
  pinMode(BTN_STOP,INPUT_PULLUP);
  pinMode(BTN_BACK,INPUT_PULLUP);
  pinMode(BTN_RIGHT,INPUT_PULLUP);
  pinMode(BTN_OK,INPUT_PULLUP);
  pinMode(BTN_LEFT,INPUT_PULLUP);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  smoothBootAnimation();
  showWelcome();
}

void loop(){
  if(millis()-lastButtonTime<debounceDelay) return;

  if(buttonPressed(BTN_STOP)){
    stopServos();
    ballX=ballY=0;
    // Clear trace data and reset PID
    prevErrorX=prevErrorY=0;
    integral_x=integral_y=0;
    derivative_x=derivative_y=0;
    Serial.println("CLEAR_TRACE"); // Clear trace specifically
    Serial.println("STATIC,0,0,90,90"); // Reset to center position
    showThankYou();
    delay(2000);
    currentState=WELCOME;
    showWelcome();
    return;
  }

  switch(currentState){
    case WELCOME:
      if(buttonPressed(BTN_OK)){
        currentState=MODE_SELECT;
        modeSelectIndex=0;
        showModeSelect();
      }
      break;

    case MODE_SELECT:
      if(buttonPressed(BTN_RIGHT)){ modeSelectIndex=1; showModeSelect(); }
      else if(buttonPressed(BTN_LEFT)){ modeSelectIndex=0; showModeSelect(); }
      else if(buttonPressed(BTN_OK)){
        Serial.println("CLEAR");
        if(modeSelectIndex==0){
          currentState=STATIC_ADJUST_X;
          targetX=targetY=0;
          showStaticAdjustX();
        } else {
          currentState=DYNAMIC_SELECT;
          dynamicIndex=0;
          showDynamicSelect();
        }
      }
      break;

    case STATIC_ADJUST_X:
      if(buttonPressed(BTN_RIGHT)){
        targetX=constrain(targetX+adjustDelta,-3,3); showStaticAdjustX();
      } else if(buttonPressed(BTN_LEFT)){
        targetX=constrain(targetX-adjustDelta,-3,3); showStaticAdjustX();
      } else if(buttonPressed(BTN_OK)){
        currentState=STATIC_ADJUST_Y; showStaticAdjustY();
      }
      break;

    case STATIC_ADJUST_Y:
      if(buttonPressed(BTN_RIGHT)){
        targetY=constrain(targetY+adjustDelta,-3,3); showStaticAdjustY();
      } else if(buttonPressed(BTN_LEFT)){
        targetY=constrain(targetY-adjustDelta,-3,3); showStaticAdjustY();
      } else if(buttonPressed(BTN_OK)){
        currentState=STATIC_MOVE;
      }
      break;

    case STATIC_MOVE:
      moveBallToTarget();
      showStaticMove();
      // Send with PID values
      Serial.print("STATIC,"); Serial.print(ballX,2); Serial.print(","); Serial.print(ballY,2);
      Serial.print(","); Serial.print(constrain(90+pidX*PID_MULTIPLIER,0,180));
      Serial.print(","); Serial.println(constrain(90+pidY*PID_MULTIPLIER,0,180));
      delay(200);
      break;

    case DYNAMIC_SELECT:
      if(buttonPressed(BTN_RIGHT)){
        dynamicIndex=(dynamicIndex+1)%shapeCount; showDynamicSelect();
      } else if(buttonPressed(BTN_LEFT)){
        dynamicIndex=(dynamicIndex-1+shapeCount)%shapeCount; showDynamicSelect();
      } else if(buttonPressed(BTN_BACK)){
        currentState=MODE_SELECT; showModeSelect();
      } else if(buttonPressed(BTN_OK)){
        currentState=DYNAMIC_RUN;
        lastMoveTime=millis();
        Serial.print("SHAPE,"); Serial.println(shapes[dynamicIndex]);
      }
      break;

    case DYNAMIC_RUN:
      if(millis()-lastMoveTime>300){
        simulateShape(shapes[dynamicIndex]);
        lastMoveTime=millis();
      }
      break;

    case THANK_YOU:
      break;
  }
}

bool buttonPressed(int pin){
  if(!digitalRead(pin)){
    lastButtonTime=millis();
    return true;
  }
  return false;
}

void smoothBootAnimation(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(" Initializing...");
  lcd.setCursor(0,1);
  for(int i=0;i<=16;i++){
    lcd.setCursor(0,1);
    for(int j=0;j<i;j++) lcd.write(255);
    for(int j=i;j<16;j++) lcd.print(" ");
    delay(100);
  }
  delay(300);
}

void showWelcome(){
  lcd.clear();
  lcd.setCursor(1,0); lcd.print("Welcome to");
  lcd.setCursor(0,1); lcd.print("  Project ASTRA  ");
  currentState=WELCOME;
}

void showModeSelect(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select Mode:");
  lcd.setCursor(0,1);
  lcd.print(modeSelectIndex==0?">Static    Dynamic":" Static   >Dynamic");
}

void showStaticAdjustX(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Set Pos X:"); lcd.print(targetX,1);
  lcd.setCursor(0,1); lcd.print("<OK> Next Y");
}

void showStaticAdjustY(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Set Pos Y:"); lcd.print(targetY,1);
  lcd.setCursor(0,1); lcd.print("<OK> Move");
}

void showStaticMove(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Ball X:"); lcd.print(ballX,1);
  lcd.print(" Y:"); lcd.print(ballY,1);
  lcd.setCursor(0,1);
  lcd.print("Err X:"); lcd.print(errorX,1);
  lcd.print(" Y:"); lcd.print(errorY,1);
}

void moveBallToTarget(){
  const float step=0.1;
  ballX += abs(targetX-ballX)>step?(targetX>ballX?step:-step):0;
  ballY += abs(targetY-ballY)>step?(targetY>ballY?step:-step):0;
  calculatePID();
}

void calculatePID(){
  // Store previous errors
  prevErrorX = errorX;
  prevErrorY = errorY;
  
  // Calculate current errors
  errorX = targetX-ballX; 
  errorY = targetY-ballY;
  
  // Enhanced PID with integral and derivative components
  const float Kp=1.2, Ki=0.05, Kd=0.8;
  
  // Add some realistic noise/disturbance to show error handling
  float noiseX = (random(-50, 51) / 1000.0); // ±0.05 noise
  float noiseY = (random(-50, 51) / 1000.0);
  
  // Integral (accumulate error over time, with windup protection)
  integral_x = constrain(integral_x + errorX, -10, 10);
  integral_y = constrain(integral_y + errorY, -10, 10);
  
  // Derivative (rate of change of error)
  derivative_x = errorX - prevErrorX;
  derivative_y = errorY - prevErrorY;
  
  // PID calculation with noise
  pidX = Kp*(errorX + noiseX) + Ki*integral_x + Kd*derivative_x;
  pidY = Kp*(errorY + noiseY) + Ki*integral_y + Kd*derivative_y;
  
  servo1.write(constrain(90+pidX*PID_MULTIPLIER,0,180));
  servo2.write(constrain(90+pidY*PID_MULTIPLIER,0,180));
  float pidZ=(pidX-pidY)*5.0;
  servo3.write(constrain(90+pidZ,0,180));
}

void showDynamicSelect(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select Shape:");
  lcd.setCursor(0,1); lcd.print("-> "); lcd.print(shapes[dynamicIndex]);
  for(int i=3+shapes[dynamicIndex].length();i<16;i++) lcd.print(" ");
}

void simulateShape(String shape){
  static float t=0;
  static bool firstPoint = true;
  float x=0,y=0;

  if(shape=="Circle"){
    x=2*cos(t); y=2*sin(t);
  }
  else if(shape=="Triangle"){
    // Fixed triangle - proper equilateral triangle
    float segment = fmod(t, 3.0);
    if(segment < 1.0) {
      x = myLerp(-2, 2, segment);
      y = -1.73; // Bottom edge
    } else if(segment < 2.0) {
      x = myLerp(2, 0, segment - 1.0);
      y = myLerp(-1.73, 1.73, segment - 1.0); // Right edge
    } else {
      x = myLerp(0, -2, segment - 2.0);
      y = myLerp(1.73, -1.73, segment - 2.0); // Left edge
    }
  }
  else if(shape=="Square"){
    // Fixed square - proper square shape
    float segment = fmod(t, 4.0);
    if(segment < 1.0) {
      x = myLerp(-2, 2, segment);
      y = -2; // Bottom edge
    } else if(segment < 2.0) {
      x = 2;
      y = myLerp(-2, 2, segment - 1.0); // Right edge
    } else if(segment < 3.0) {
      x = myLerp(2, -2, segment - 2.0);
      y = 2; // Top edge
    } else {
      x = -2;
      y = myLerp(2, -2, segment - 3.0); // Left edge
    }
  }
  else if(shape=="Infinity"){
    x=1.5*sin(t); y=0.75*sin(2*t);
  }
  else if(shape=="Hexagon"){
    float angle=TWO_PI/6;
    float seg=fmod(t,6); int i=int(seg); float f=seg-i;
    x=myLerp(2*cos(i*angle),2*cos((i+1)*angle),f);
    y=myLerp(2*sin(i*angle),2*sin((i+1)*angle),f);
  }
  else if(shape=="Ellipse"){
    x=3*cos(t); y=1.5*sin(t);
  }

  // Send grey trace from 0,0 to first point when starting
  if(firstPoint && t == 0.0){
    Serial.print("GREY_TRACE,0,0,"); Serial.print(x,3); Serial.print(","); Serial.println(y,3);
    firstPoint = false;
  }

  // Update ball position and calculate PID
  ballX = x; ballY = y;
  targetX = x; targetY = y;
  calculatePID();

  // LCD: show dynamic trace data
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Tr X:"); lcd.print(x,1);
  lcd.print(" Y:"); lcd.print(y,1);
  lcd.setCursor(0,1);
  lcd.print("PID:"); lcd.print(pidX,1);
  lcd.print("/"); lcd.print(pidY,1);

  Serial.print("DYNAMIC,");
  Serial.print(x,3); Serial.print(","); Serial.print(y,3); Serial.print(",");
  Serial.print(constrain(90+pidX*PID_MULTIPLIER,0,180)); Serial.print(",");
  Serial.println(constrain(90+pidY*PID_MULTIPLIER,0,180));

  t+=0.1;
  if(t > TWO_PI * 2) { // Reset after 2 full cycles
    t = 0;
    firstPoint = true;
  }
}

float myLerp(float a,float b,float t){return a+t*(b-a);}

void stopServos(){
  servo1.write(90); servo2.write(90); servo3.write(90);
}

void showThankYou(){
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Thank You!!!");
  lcd.setCursor(3,1); lcd.print("Team ASTRA");
  currentState=THANK_YOU;
}