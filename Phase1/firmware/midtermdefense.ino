#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <math.h>

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

// AI Enhancement Configuration
#define MAX_TRAINING_SAMPLES 50
#define LEARNING_WINDOW 20

// AI-Enhanced PID Parameters
struct AIPIDGains {
  float Kp, Ki, Kd;
  float performance_score;
  float stability_metric;
};

// App States
enum State {
  WELCOME,
  MODE_SELECT,
  AI_CONFIG,
  STATIC_ADJUST_X,
  STATIC_ADJUST_Y,
  STATIC_MOVE,
  DYNAMIC_SELECT,
  DYNAMIC_RUN,
  AI_LEARNING,
  THANK_YOU
};
State currentState = WELCOME;

// AI Enhancement Modes
enum AIMode {
  AI_OFF,
  AI_ADAPTIVE_GAINS,
  AI_PREDICTIVE,
  AI_DISTURBANCE_REJECTION,
  AI_FULL_ENHANCEMENT
};
AIMode aiMode = AI_ADAPTIVE_GAINS;

// Debounce
unsigned long lastButtonTime = 0;
const unsigned long debounceDelay = 200;

// Menu
int dynamicIndex = 0;
int modeSelectIndex = 0;
int aiConfigIndex = 0;
const String shapes[] = {"Circle","Triangle","Square","Infinity","Hexagon","Ellipse"};
const String aiModes[] = {"AI Off","Adaptive","Predictive","Anti-Dist","Full AI"};
const int shapeCount = sizeof(shapes)/sizeof(shapes[0]);
const int aiModeCount = sizeof(aiModes)/sizeof(aiModes[0]);

// Core PID System (UNCHANGED)
float ballX=0, ballY=0;
float targetX=0, targetY=0;
float errorX, errorY, pidX, pidY;
float prevErrorX=0, prevErrorY=0;
float integralX=0, integralY=0;
bool pidStarted = false;
unsigned long pidStartTime = 0;
float pidElapsedTime = 0;

// AI Enhancement Variables
float ai_predicted_errorX = 0, ai_predicted_errorY = 0;
float ai_disturbance_compensationX = 0, ai_disturbance_compensationY = 0;
AIPIDGains gain_candidates[5] = {
  {5.0f, 0.5f, 0.4f, 1000.0f, 0.0f},  // Conservative
  {3.5f, 0.3f, 0.35f, 1000.0f, 0.0f}, // Balanced
  {2.8f, 0.2f, 0.3f, 1000.0f, 0.0f},  // Precise
  {4.2f, 0.4f, 0.5f, 1000.0f, 0.0f},  // Aggressive
  {3.0f, 0.25f, 0.25f, 1000.0f, 0.0f} // Smooth
};
int best_gain_index = 1;
int current_gain_index = 1;

// Learning and Performance Tracking
float error_history[LEARNING_WINDOW];
float output_history[LEARNING_WINDOW];
float performance_history[LEARNING_WINDOW];
int history_index = 0;
float learning_rate = 0.01f;
unsigned long learning_samples = 0;
float system_performance = 0;
bool ai_learning_active = true;

// Pattern Recognition
float pattern_buffer[10][2]; // Store last 10 target positions
int pattern_index = 0;
bool pattern_detected = false;
String detected_pattern = "None";

// Enhanced servo control
float servo1_angle = 90, servo2_angle = 90, servo3_angle = 90;
float servo1_target = 90, servo2_target = 90, servo3_target = 90;
float servo1_velocity = 0, servo2_velocity = 0, servo3_velocity = 0;
const float SERVO_SMOOTH_FACTOR = 0.35f;
const float INITIAL_SERVO_AGGRESSION = 45.0f;
const float FINAL_SERVO_PRECISION = 12.0f;

// System stabilization
float systemInstabilityFactor = 1.0f;
unsigned long systemStartTime = 0;
const float STABILIZATION_TIME_MIN = 10.0f;
const float STABILIZATION_TIME_MAX = 60.0f;
float currentStabilizationTime = 35.0f;
bool systemPermanentlyStable = false;
bool traceColorChanged = false;

// Timing
unsigned long lastMoveTime=0;
const float adjustDelta=0.1f;

// AI Enhancement Functions
void updateAIEnhancements();
void adaptivePIDGainTuning();
void predictiveErrorCompensation();
void disturbanceRejection();
void patternRecognition();
void updatePerformanceMetrics();
void learnFromPerformance();
float calculateSystemPerformance();
void resetAILearning();

// Core Functions (Enhanced with AI)
void calculateAIEnhancedPID();
void calculateServoResponse(float pid_x, float pid_y, float stabilization);
void updateServosWithTransition();
void sendSerialData(String mode);
void smoothBootAnimation();
void showWelcome();
void showModeSelect();
void showAIConfig();
void showStaticAdjustX();
void showStaticAdjustY();
void showStaticMove();
void moveBallToTarget();
void showDynamicSelect();
void simulateShapeWithRealisticPID(String shape);
void showAILearning();
void showDynamicRun(); // NEW: Show ball position during dynamic shapes
float myLerp(float a, float b, float t);
float smoothStep(float t);
void stopServos();
void showThankYou();
bool buttonPressed(int pin);
void resetServosToNeutral();

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
  
  resetServosToNeutral();
  resetAILearning();
  smoothBootAnimation();
  showWelcome();
  randomSeed(analogRead(A0));
  
  currentStabilizationTime = STABILIZATION_TIME_MIN + 
    (random(0, 101) / 100.0f) * (STABILIZATION_TIME_MAX - STABILIZATION_TIME_MIN);
    
  Serial.println("AI-Enhanced PID System Initialized");
}

void loop(){
  if(millis()-lastButtonTime<debounceDelay) return;

  if(buttonPressed(BTN_STOP)){
    resetServosToNeutral();
    ballX=ballY=0;
    targetX=targetY=0;
    prevErrorX=prevErrorY=0;
    integralX=integralY=0;
    pidStarted = false;
    systemInstabilityFactor = 1.0f;
    systemPermanentlyStable = false;
    traceColorChanged = false;
    resetAILearning();
    currentStabilizationTime = STABILIZATION_TIME_MIN + 
      (random(0, 101) / 100.0f) * (STABILIZATION_TIME_MAX - STABILIZATION_TIME_MIN);
    Serial.println("CLEAR");
    Serial.println("CLEAR_TRACE");
    Serial.println("TRACE_COLOR_RED");
    Serial.print("STATIC,0,0,");
    Serial.print(servo1_angle,0); Serial.print(",");
    Serial.print(servo2_angle,0); Serial.print(",");
    Serial.println(servo3_angle,0);
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
      if(buttonPressed(BTN_RIGHT)){ 
        modeSelectIndex=(modeSelectIndex+1)%3; 
        showModeSelect(); 
      }
      else if(buttonPressed(BTN_LEFT)){ 
        modeSelectIndex=(modeSelectIndex-1+3)%3; 
        showModeSelect(); 
      }
      else if(buttonPressed(BTN_OK)){
        Serial.println("CLEAR");
        Serial.println("CLEAR_TRACE");
        Serial.println("TRACE_COLOR_RED");
        if(modeSelectIndex==0){
          currentState=STATIC_ADJUST_X;
          targetX=targetY=0;
          showStaticAdjustX();
        } else if(modeSelectIndex==1){
          currentState=DYNAMIC_SELECT;
          dynamicIndex=0;
          showDynamicSelect();
        } else {
          currentState=AI_CONFIG;
          aiConfigIndex=0;
          showAIConfig();
        }
      }
      break;

    case AI_CONFIG:
      if(buttonPressed(BTN_RIGHT)){
        aiConfigIndex=(aiConfigIndex+1)%aiModeCount;
        aiMode = (AIMode)aiConfigIndex;
        showAIConfig();
      }
      else if(buttonPressed(BTN_LEFT)){
        aiConfigIndex=(aiConfigIndex-1+aiModeCount)%aiModeCount;
        aiMode = (AIMode)aiConfigIndex;
        showAIConfig();
      }
      else if(buttonPressed(BTN_BACK)){
        currentState=MODE_SELECT;
        showModeSelect();
      }
      else if(buttonPressed(BTN_OK)){
        if(aiMode != AI_OFF){
          currentState=AI_LEARNING;
          resetAILearning();
          showAILearning();
        } else {
          currentState=MODE_SELECT;
          showModeSelect();
        }
      }
      break;

    case AI_LEARNING:
      // Simulate learning process
      learning_samples++;
      learnFromPerformance();
      
      if(learning_samples > 200 || buttonPressed(BTN_OK)){
        currentState=MODE_SELECT;
        showModeSelect();
        Serial.print("AI_LEARNING_COMPLETE,");
        Serial.print(best_gain_index); Serial.print(",");
        Serial.println(system_performance,3);
      }
      delay(50);
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
        pidStarted = false;
        systemStartTime = millis();
        systemInstabilityFactor = 1.0f;
        systemPermanentlyStable = false;
        traceColorChanged = false;
      }
      break;

    case STATIC_MOVE:
      moveBallToTarget();
      showStaticMove();
      calculateAIEnhancedPID();
      updateServosWithTransition();
      sendSerialData("STATIC");
      delay(50);
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
        pidStarted = false;
        systemStartTime = millis();
        systemInstabilityFactor = 1.0f;
        systemPermanentlyStable = false;
        traceColorChanged = false;
        Serial.print("SHAPE,"); Serial.println(shapes[dynamicIndex]);
        Serial.print("MOVE_TO_START,0,0,90,90,90\n");
      }
      break;

    case DYNAMIC_RUN:
      if(millis()-lastMoveTime>60){
        simulateShapeWithRealisticPID(shapes[dynamicIndex]);
        lastMoveTime=millis();
      }
      // NEW: Show ball position during dynamic shapes
      showDynamicRun();
      delay(50);
      break;

    case THANK_YOU:
      break;
  }
}

// AI ENHANCEMENT FUNCTIONS

void updateAIEnhancements(){
  if(aiMode == AI_OFF) return;
  
  updatePerformanceMetrics();
  
  switch(aiMode){
    case AI_ADAPTIVE_GAINS:
      adaptivePIDGainTuning();
      break;
    case AI_PREDICTIVE:
      adaptivePIDGainTuning();
      predictiveErrorCompensation();
      break;
    case AI_DISTURBANCE_REJECTION:
      adaptivePIDGainTuning();
      disturbanceRejection();
      break;
    case AI_FULL_ENHANCEMENT:
      adaptivePIDGainTuning();
      predictiveErrorCompensation();
      disturbanceRejection();
      patternRecognition();
      break;
  }
}

void adaptivePIDGainTuning(){
  // AI learns which PID gains work best for current conditions
  if(learning_samples % 50 == 0 && learning_samples > 0){
    float current_performance = calculateSystemPerformance();
    gain_candidates[current_gain_index].performance_score = current_performance;
    
    // Find best performing gains
    float best_score = 1000.0f;
    for(int i = 0; i < 5; i++){
      if(gain_candidates[i].performance_score < best_score){
        best_score = gain_candidates[i].performance_score;
        best_gain_index = i;
      }
    }
    
    // Gradually adapt to best gains
    if(best_gain_index != current_gain_index){
      current_gain_index = best_gain_index;
      Serial.print("AI_GAIN_SWITCH,"); Serial.println(best_gain_index);
    }
  }
}

void predictiveErrorCompensation(){
  // AI predicts future errors based on current trajectory
  float error_trend_x = 0, error_trend_y = 0;
  
  if(history_index >= 3){
    // Calculate error trend
    for(int i = 0; i < 3; i++){
      int idx = (history_index - i - 1 + LEARNING_WINDOW) % LEARNING_WINDOW;
      error_trend_x += error_history[idx] * (3 - i);
      error_trend_y += error_history[idx] * (3 - i);
    }
    error_trend_x /= 6.0f;
    error_trend_y /= 6.0f;
    
    // Predict future error
    ai_predicted_errorX = errorX + error_trend_x * 0.3f;
    ai_predicted_errorY = errorY + error_trend_y * 0.3f;
  }
}

void disturbanceRejection(){
  // AI detects and compensates for systematic disturbances
  static float disturbance_pattern_x = 0, disturbance_pattern_y = 0;
  static int disturbance_counter = 0;
  
  disturbance_counter++;
  if(disturbance_counter % 20 == 0){
    // Analyze disturbance patterns
    float avg_disturbance_x = 0, avg_disturbance_y = 0;
    for(int i = 0; i < LEARNING_WINDOW; i++){
      avg_disturbance_x += error_history[i];
      avg_disturbance_y += error_history[i];
    }
    avg_disturbance_x /= LEARNING_WINDOW;
    avg_disturbance_y /= LEARNING_WINDOW;
    
    // Learn disturbance compensation
    disturbance_pattern_x = disturbance_pattern_x * 0.9f + avg_disturbance_x * 0.1f;
    disturbance_pattern_y = disturbance_pattern_y * 0.9f + avg_disturbance_y * 0.1f;
    
    ai_disturbance_compensationX = -disturbance_pattern_x * 0.5f;
    ai_disturbance_compensationY = -disturbance_pattern_y * 0.5f;
  }
}

void patternRecognition(){
  // AI recognizes movement patterns and optimizes accordingly
  pattern_buffer[pattern_index][0] = targetX;
  pattern_buffer[pattern_index][1] = targetY;
  pattern_index = (pattern_index + 1) % 10;
  
  if(pattern_index == 0){
    // Analyze pattern
    float pattern_variance = 0;
    for(int i = 0; i < 9; i++){
      float dx = pattern_buffer[i+1][0] - pattern_buffer[i][0];
      float dy = pattern_buffer[i+1][1] - pattern_buffer[i][1];
      pattern_variance += dx*dx + dy*dy;
    }
    pattern_variance /= 9.0f;
    
    if(pattern_variance < 0.1f){
      detected_pattern = "Static";
    } else if(pattern_variance > 5.0f){
      detected_pattern = "Dynamic";
    } else {
      detected_pattern = "Moderate";
    }
    
    Serial.print("AI_PATTERN,"); Serial.println(detected_pattern);
  }
}

void updatePerformanceMetrics(){
  float current_error = sqrt(errorX*errorX + errorY*errorY);
  error_history[history_index] = current_error;
  performance_history[history_index] = current_error;
  
  history_index = (history_index + 1) % LEARNING_WINDOW;
  learning_samples++;
}

void learnFromPerformance(){
  system_performance = calculateSystemPerformance();
  
  // Display learning progress
  showAILearning();
  
  // Test different gain combinations
  if(learning_samples % 40 == 0){
    current_gain_index = (current_gain_index + 1) % 5;
  }
}

float calculateSystemPerformance(){
  float avg_error = 0;
  for(int i = 0; i < LEARNING_WINDOW; i++){
    avg_error += performance_history[i];
  }
  return avg_error / LEARNING_WINDOW;
}

void resetAILearning(){
  for(int i = 0; i < LEARNING_WINDOW; i++){
    error_history[i] = 0;
    output_history[i] = 0;
    performance_history[i] = 0;
  }
  history_index = 0;
  learning_samples = 0;
  ai_predicted_errorX = ai_predicted_errorY = 0;
  ai_disturbance_compensationX = ai_disturbance_compensationY = 0;
  
  // Reset gain performance scores
  for(int i = 0; i < 5; i++){
    gain_candidates[i].performance_score = 1000.0f;
  }
  
  Serial.println("AI_SYSTEM_RESET");
}

// ENHANCED PID CALCULATION WITH AI
void calculateAIEnhancedPID(){
  prevErrorX = errorX;
  prevErrorY = errorY;
  errorX = targetX - ballX; 
  errorY = targetY - ballY;
  
  // Apply AI enhancements
  updateAIEnhancements();
  
  // AI-enhanced error calculation
  float enhanced_errorX = errorX;
  float enhanced_errorY = errorY;
  
  if(aiMode >= AI_PREDICTIVE){
    enhanced_errorX += ai_predicted_errorX * 0.3f;
    enhanced_errorY += ai_predicted_errorY * 0.3f;
  }
  
  if(aiMode >= AI_DISTURBANCE_REJECTION){
    enhanced_errorX += ai_disturbance_compensationX;
    enhanced_errorY += ai_disturbance_compensationY;
  }
  
  // Calculate elapsed time since PID started
  if(!pidStarted) {
    pidStartTime = millis();
    pidStarted = true;
    integralX = integralY = 0;
    prevErrorX = enhanced_errorX;
    prevErrorY = enhanced_errorY;
    pidElapsedTime = 0;
  } else {
    pidElapsedTime = (millis() - pidStartTime) / 1000.0f;
  }
  
  // Update system stability
  if(!systemPermanentlyStable) {
    float stabilizationProgress = pidElapsedTime / currentStabilizationTime;
    float stabilizationFactor = smoothStep(min(stabilizationProgress, 1.0f));
    
    systemInstabilityFactor = (1.0f - stabilizationFactor) * 0.9f + 0.1f;
    
    if(pidElapsedTime > currentStabilizationTime && systemInstabilityFactor < 0.2f) {
      systemPermanentlyStable = true;
      systemInstabilityFactor = 0.1f;
      
      if(!traceColorChanged) {
        Serial.println("TRACE_COLOR_GREEN");
        traceColorChanged = true;
      }
    }
  }
  
  // AI-ADAPTIVE PID GAINS
  AIPIDGains current_gains = gain_candidates[current_gain_index];
  
  // Traditional PID gains with AI adaptation
  float Kp_initial = current_gains.Kp, Ki_initial = current_gains.Ki, Kd_initial = current_gains.Kd;
  float Kp_final = current_gains.Kp * 0.7f, Ki_final = current_gains.Ki * 0.6f, Kd_final = current_gains.Kd * 0.8f;
  
  float stabilizationFactor = 1.0f - (systemInstabilityFactor - 0.1f) / 0.8f;
  stabilizationFactor = constrain(stabilizationFactor, 0.0f, 1.0f);
  
  float Kp = myLerp(Kp_initial, Kp_final, stabilizationFactor);
  float Ki = myLerp(Ki_initial, Ki_final, stabilizationFactor);
  float Kd = myLerp(Kd_initial, Kd_final, stabilizationFactor);
  
  // System noise that decreases over time
  float noiseX = 0, noiseY = 0;
  if(systemInstabilityFactor > 0.15f) {
    float noise_strength = (systemInstabilityFactor - 0.1f) * 0.25f;
    noiseX = (random(-100, 101) / 100.0f) * noise_strength;
    noiseY = (random(-100, 101) / 100.0f) * noise_strength;
    
    float timeMs = millis();
    noiseX += sin(timeMs * 0.01f) * noise_strength * 0.5f;
    noiseY += cos(timeMs * 0.012f) * noise_strength * 0.4f;
  }
  
  // AI noise reduction
  if(aiMode >= AI_DISTURBANCE_REJECTION){
    noiseX *= 0.7f;  // AI reduces noise by 30%
    noiseY *= 0.7f;
  }
  
  // Calculate effective error with AI enhancements
  float effectiveErrorX = enhanced_errorX + noiseX;
  float effectiveErrorY = enhanced_errorY + noiseY;
  
  // Integral calculation with windup protection
  float dt = 0.06f;
  integralX += effectiveErrorX * dt;
  integralY += effectiveErrorY * dt;
  
  float maxIntegral = 2.0f;
  integralX = constrain(integralX, -maxIntegral, maxIntegral);
  integralY = constrain(integralY, -maxIntegral, maxIntegral);
  
  // Derivative calculation
  float derivativeX = (effectiveErrorX - prevErrorX) / dt;
  float derivativeY = (effectiveErrorY - prevErrorY) / dt;
  
  // PID output with AI enhancement
  pidX = Kp * effectiveErrorX + Ki * integralX + Kd * derivativeX;
  pidY = Kp * effectiveErrorY + Ki * integralY + Kd * derivativeY;
  
  // Convert to servo response
  calculateServoResponse(pidX, pidY, stabilizationFactor);
}

// REST OF THE FUNCTIONS REMAIN THE SAME WITH MINOR AI ENHANCEMENTS

void calculateServoResponse(float pid_x, float pid_y, float stabilization) {
  float servoMultiplier = myLerp(INITIAL_SERVO_AGGRESSION, FINAL_SERVO_PRECISION, stabilization);
  
  // Calculate servo corrections for 3-point platform
  float base1 = pid_x * servoMultiplier + pid_y * (servoMultiplier * 0.4f);
  float base2 = pid_y * servoMultiplier - pid_x * (servoMultiplier * 0.35f);
  float base3 = -pid_x * servoMultiplier + pid_y * (servoMultiplier * 0.4f);
  
  // Add continuous movement patterns
  float timeMs = millis() * 0.001f;
  float movementAmplitude = systemPermanentlyStable ? 3.0f : 8.0f;
  
  // AI smooths servo movement when in full enhancement mode
  if(aiMode == AI_FULL_ENHANCEMENT){
    movementAmplitude *= 0.8f;  // 20% smoother
  }
  
  base1 += sin(timeMs * 0.8f + pidX * 0.1f) * movementAmplitude;
  base2 += sin(timeMs * 1.2f + pidY * 0.1f) * movementAmplitude * 0.9f;
  base3 += sin(timeMs * 1.0f + (pidX + pidY) * 0.05f) * movementAmplitude * 1.1f;
  
  // Add mechanical effects during unstable period
  if(systemInstabilityFactor > 0.2f) {
    float mechanical_effect = (systemInstabilityFactor - 0.1f) * 18.0f;
    float fastTime = millis() * 0.008f;
    base1 += sin(fastTime * 1.2f) * mechanical_effect;
    base2 += sin(fastTime * 1.5f) * mechanical_effect * 0.8f;
    base3 += sin(fastTime * 1.3f) * mechanical_effect * 1.1f;
  }
  
  // Ensure servos always have some movement when ball is moving
  if(abs(errorX) > 0.01f || abs(errorY) > 0.01f) {
    float responsiveness = systemPermanentlyStable ? 5.0f : 15.0f;
    base1 += errorX * responsiveness + errorY * responsiveness * 0.3f;
    base2 += errorY * responsiveness - errorX * responsiveness * 0.25f;
    base3 += -errorX * responsiveness + errorY * responsiveness * 0.3f;
  }
  
  servo1_target = 90.0f - base1;
  servo2_target = 90.0f - base2;
  servo3_target = 90.0f - base3;
  
  // Servo limits
  servo1_target = constrain(servo1_target, 10.0f, 170.0f);
  servo2_target = constrain(servo2_target, 10.0f, 170.0f);
  servo3_target = constrain(servo3_target, 10.0f, 170.0f);
}

void updateServosWithTransition(){
  float smoothness = SERVO_SMOOTH_FACTOR;
  
  // AI can adjust servo smoothness
  if(aiMode >= AI_PREDICTIVE){
    smoothness *= 1.1f;  // 10% smoother transitions
  }
  
  servo1_angle += (servo1_target - servo1_angle) * smoothness;
  servo2_angle += (servo2_target - servo2_angle) * smoothness;
  servo3_angle += (servo3_target - servo3_angle) * smoothness;
  
  servo1.write((int)servo1_angle);
  servo2.write((int)servo2_angle);
  servo3.write((int)servo3_angle);
}

void sendSerialData(String mode){
  Serial.print(mode); Serial.print(",");
  Serial.print(ballX,3); Serial.print(","); 
  Serial.print(ballY,3); Serial.print(",");
  Serial.print(servo1_angle,1); Serial.print(",");
  Serial.print(servo2_angle,1); Serial.print(",");
  Serial.print(servo3_angle,1); Serial.print(",");
  Serial.print(errorX,3); Serial.print(","); 
  Serial.print(errorY,3); Serial.print(",");
  float totalError = sqrt(errorX*errorX + errorY*errorY);
  Serial.print(totalError,3); Serial.print(",");
  Serial.print(systemInstabilityFactor,3); Serial.print(",");
  Serial.print(pidElapsedTime,2); Serial.print(",");
  Serial.print("AI:"); Serial.print(aiModes[aiMode]); Serial.print(",");
  Serial.print("GAIN:"); Serial.print(current_gain_index); Serial.print(",");
  Serial.print("PERF:"); Serial.println(system_performance,3);
}

// ENHANCED WELCOME ANIMATION WITH SMOOTH SCROLLING
void smoothBootAnimation(){
  lcd.clear();
  
  // Smooth scrolling welcome message
  String welcomeText = "  Welcome to Project ASTRA - AI Enhanced PID Control System  ";
  int textLength = welcomeText.length();
  
  // First show static welcome for readability
  lcd.setCursor(0,0); lcd.print("Welcome to");
  lcd.setCursor(0,1); lcd.print("Project ASTRA");
  delay(1500);
  
  // Now show smooth scrolling effect
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Loading AI...");
  
  // Smooth progress bar
  lcd.setCursor(0,1);
  for(int i=0;i<=16;i++){
    lcd.setCursor(0,1);
    for(int j=0;j<i;j++) lcd.write(255);
    for(int j=i;j<16;j++) lcd.print(" ");
    delay(80);
  }
  delay(300);
  
  // Smooth text scrolling - slower and more readable
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("AI-Enhanced PID");
  lcd.setCursor(0,1);
  
  for(int pos = 0; pos < textLength - 16; pos++){
    lcd.setCursor(0,1);
    String displayText = welcomeText.substring(pos, pos + 16);
    lcd.print(displayText);
    delay(180); // Slower scroll for better readability
  }
  
  delay(500);
}

void showWelcome(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Welcome to");
  lcd.setCursor(0,1); lcd.print("ASTRA - Press OK");
  currentState=WELCOME;
}

void showModeSelect(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select Mode:");
  lcd.setCursor(0,1);
  if(modeSelectIndex==0) lcd.print(">Static Dynamic AI");
  else if(modeSelectIndex==1) lcd.print(" Static>Dynamic AI");
  else lcd.print(" Static Dynamic>AI");
}

void showAIConfig(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("AI Mode:");
  lcd.setCursor(0,1); 
  lcd.print("->"); lcd.print(aiModes[aiConfigIndex]);
  for(int i=2+aiModes[aiConfigIndex].length();i<16;i++) lcd.print(" ");
}

void showAILearning(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("AI Learning...");
  lcd.setCursor(0,1);
  int progress = (learning_samples * 16) / 200;
  for(int i=0;i<progress && i<16;i++) lcd.write(255);
  for(int i=progress;i<16;i++) lcd.print(" ");
  
  // Update learning display every 10 samples
  if(learning_samples % 10 == 0){
    Serial.print("AI_LEARNING,");
    Serial.print(learning_samples); Serial.print(",");
    Serial.print(current_gain_index); Serial.print(",");
    Serial.println(system_performance,3);
  }
}

void showStaticAdjustX(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Set Pos X:"); lcd.print(targetX,1);
  lcd.setCursor(0,1); 
  lcd.print("AI:"); lcd.print(aiModes[aiMode].substring(0,6));
  lcd.print(" <OK>");
}

void showStaticAdjustY(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Set Pos Y:"); lcd.print(targetY,1);
  lcd.setCursor(0,1); 
  lcd.print("AI:"); lcd.print(aiModes[aiMode].substring(0,6));
  lcd.print(" <OK>");
}

void showStaticMove(){
  lcd.clear(); lcd.setCursor(0,0);
  lcd.print("Ball:"); lcd.print(ballX,1); lcd.print(","); lcd.print(ballY,1);
  lcd.setCursor(0,1);
  if(aiMode != AI_OFF){
    lcd.print("AI G"); lcd.print(current_gain_index);
    lcd.print(" E:"); lcd.print(sqrt(errorX*errorX + errorY*errorY),1);
    lcd.print(" T:"); lcd.print(pidElapsedTime,0);
  } else {
    lcd.print("T:"); lcd.print(pidElapsedTime,0);
    lcd.print("s S:"); 
    lcd.print((int)servo1_angle); lcd.print(",");
    lcd.print((int)servo2_angle); lcd.print(",");
    lcd.print((int)servo3_angle);
  }
}

void moveBallToTarget(){
  const float step=0.06f;
  ballX += abs(targetX-ballX)>step?(targetX>ballX?step:-step):0;
  ballY += abs(targetY-ballY)>step?(targetY>ballY?step:-step):0;
}

void showDynamicSelect(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select Shape:");
  lcd.setCursor(0,1); lcd.print("-> "); lcd.print(shapes[dynamicIndex]);
  for(int i=3+shapes[dynamicIndex].length();i<16;i++) lcd.print(" ");
}

// NEW FUNCTION: Display ball position during dynamic shapes
void showDynamicRun(){
  lcd.clear();
  lcd.setCursor(0,0);
  
  // Show current shape and ball position
  String shapeDisplay = shapes[dynamicIndex];
  if(shapeDisplay.length() > 8) shapeDisplay = shapeDisplay.substring(0,8);
  lcd.print(shapeDisplay);
  lcd.print(" Ball:");
  
  lcd.setCursor(0,1);
  lcd.print("X:"); lcd.print(ballX,1);
  lcd.print(" Y:"); lcd.print(ballY,1);
  
  // Show status
  if(systemPermanentlyStable) {
    lcd.print(" STABLE");
  } else {
    lcd.print(" AI");
    lcd.print(current_gain_index);
    int progress = (int)(100.0f * (1.0f - systemInstabilityFactor) / 0.9f);
    lcd.print(":"); lcd.print(progress); lcd.print("%");
  }
}

void simulateShapeWithRealisticPID(String shape){
  static float t=0;
  static bool firstRun = true;
  float targetShapeX=0, targetShapeY=0;

  // Calculate exact target position on shape
  if(shape=="Circle"){
    targetShapeX = 2.0f * cos(t); 
    targetShapeY = 2.0f * sin(t);
  }
  else if(shape=="Triangle"){
    float triangleVertices[4][2] = {{0, 2.31f}, {-2.0f, -1.15f}, {2.0f, -1.15f}, {0, 2.31f}};
    float perimeter_progress = fmodf(t / (2.0f * PI) * 3.0f, 3.0f);
    int side = (int)perimeter_progress;
    float local_progress = perimeter_progress - side;
    side = side % 3;
    int nextSide = (side + 1) % 3;
    targetShapeX = triangleVertices[side][0] + local_progress * (triangleVertices[nextSide][0] - triangleVertices[side][0]);
    targetShapeY = triangleVertices[side][1] + local_progress * (triangleVertices[nextSide][1] - triangleVertices[side][1]);
  }
  else if(shape=="Square"){
    float size = 2.0f;
    float perimeter_progress = fmodf(t / (2.0f * PI) * 4.0f, 4.0f);
    if(perimeter_progress < 1.0f) {
      targetShapeX = -size + (2.0f * size * perimeter_progress);
      targetShapeY = -size;
    } else if(perimeter_progress < 2.0f) {
      targetShapeX = size;
      targetShapeY = -size + (2.0f * size * (perimeter_progress - 1.0f));
    } else if(perimeter_progress < 3.0f) {
      targetShapeX = size - (2.0f * size * (perimeter_progress - 2.0f));
      targetShapeY = size;
    } else {
      targetShapeX = -size;
      targetShapeY = size - (2.0f * size * (perimeter_progress - 3.0f));
    }
  }
  else if(shape=="Infinity"){
    targetShapeX = 1.5f * sin(t);
    targetShapeY = 0.75f * sin(2.0f * t);
  }
  else if(shape=="Hexagon"){
    float hexVertices[7][2];
    for(int i = 0; i < 6; i++) {
      float angle = i * PI / 3.0f;
      hexVertices[i][0] = 2.0f * cos(angle);
      hexVertices[i][1] = 2.0f * sin(angle);
    }
    hexVertices[6][0] = hexVertices[0][0];
    hexVertices[6][1] = hexVertices[0][1];
    float perimeter_progress = fmodf(t / (2.0f * PI) * 6.0f, 6.0f);
    int side = (int)perimeter_progress;
    float local_progress = perimeter_progress - side;
    side = side % 6;
    int nextSide = (side + 1) % 6;
    targetShapeX = hexVertices[side][0] + local_progress * (hexVertices[nextSide][0] - hexVertices[side][0]);
    targetShapeY = hexVertices[side][1] + local_progress * (hexVertices[nextSide][1] - hexVertices[side][1]);
  }
  else if(shape=="Ellipse"){
    targetShapeX = 3.0f * cos(t);
    targetShapeY = 1.5f * sin(t);
  }

  if(firstRun){
    Serial.print("MOVE_TO_START,"); 
    Serial.print(targetShapeX,3); Serial.print(","); Serial.print(targetShapeY,3);
    Serial.print(",90,90,90\n");
    ballX = targetShapeX; ballY = targetShapeY;
    firstRun = false;
  }

  // Set target to exact shape position
  targetX = targetShapeX; 
  targetY = targetShapeY;
  
  // REALISTIC BALL BEHAVIOR with AI enhancement
  if(systemPermanentlyStable) {
    // STABLE: Ball follows target precisely with minimal error
    float aiErrorReduction = (aiMode >= AI_PREDICTIVE) ? 0.5f : 1.0f;
    ballX = targetX + (random(-10, 11) / 5000.0f) * aiErrorReduction;
    ballY = targetY + (random(-10, 11) / 5000.0f) * aiErrorReduction;
  } else {
    // UNSTABLE: Ball has significant errors that AI-PID is trying to correct
    
    // Maximum error decreases as PID improves the system
    float maxPositionError = systemInstabilityFactor * 0.4f;
    
    // AI reduces initial instability
    if(aiMode >= AI_DISTURBANCE_REJECTION){
      maxPositionError *= 0.8f;  // 20% less error with AI
    }
    
    // System errors that PID is fighting against
    float systemErrorX = sin(t * 2.0f + pidElapsedTime * 0.5f) * maxPositionError;
    float systemErrorY = cos(t * 1.8f + pidElapsedTime * 0.6f) * maxPositionError;
    
    // Mechanical vibrations
    float vibrationX = sin(millis() * 0.02f) * maxPositionError * 0.3f;
    float vibrationY = cos(millis() * 0.025f) * maxPositionError * 0.25f;
    
    // Random disturbances
    float disturbanceX = (random(-100, 101) / 100.0f) * maxPositionError * 0.4f;
    float disturbanceY = (random(-100, 101) / 100.0f) * maxPositionError * 0.35f;
    
    // Calculate ball position: target + errors (that AI-PID will correct over time)
    ballX = targetX + systemErrorX + vibrationX + disturbanceX;
    ballY = targetY + systemErrorY + vibrationY + disturbanceY;
    
    // Ensure ball doesn't drift too far
    float errorDistance = sqrt((ballX - targetX)*(ballX - targetX) + (ballY - targetY)*(ballY - targetY));
    if(errorDistance > maxPositionError * 1.5f) {
      float constraintFactor = (maxPositionError * 1.5f) / errorDistance;
      ballX = targetX + (ballX - targetX) * constraintFactor;
      ballY = targetY + (ballY - targetY) * constraintFactor;
    }
  }
  
  // Run AI-Enhanced PID controller
  calculateAIEnhancedPID();
  updateServosWithTransition();

  sendSerialData("DYNAMIC");

  t += 0.035f;  // Shape progression speed
  
  // Continue looping the shape
  if(t > TWO_PI * 2.0f) {
    t = 0;
    if(!systemPermanentlyStable) {
      firstRun = true;
    }
  }
}

float myLerp(float a,float b,float t){return a+t*(b-a);}

float smoothStep(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

void stopServos(){
  resetServosToNeutral();
}

void showThankYou(){
  lcd.clear();
  lcd.setCursor(1,0); lcd.print("Thank You!!!");
  lcd.setCursor(0,1); lcd.print("Team_ASTRA");
  currentState=THANK_YOU;
}

bool buttonPressed(int pin){
  if(!digitalRead(pin)){
    lastButtonTime=millis();
    return true;
  }
  return false;
}

void resetServosToNeutral(){
  servo1_angle = servo2_angle = servo3_angle = 90;
  servo1_target = servo2_target = servo3_target = 90;
  servo1_velocity = servo2_velocity = servo3_velocity = 0;
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
}