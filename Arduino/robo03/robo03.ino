#include <M5Atom.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <math.h>
#include "policy_network.h"

WebServer server(80);

const char ssid[] = "robo1";
const char pass[] = "password";

const IPAddress ip(192,168,42,1);
const IPAddress subnet(255,255,255,0);

Servo servo1, servo2;

int ang1 = 0, offset1 = 0, pulse1 = 1000;
int ang2 = 0, offset2 = 0, pulse2 = 1000;

float pitch, roll;
float servo1_target = 0.0f, servo2_target = 0.0f;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta_X = 0.0, theta_Y = 0.0;
float kalAngleX, kalAngleDotX, kalAngleY, kalAngleDotY;
Kalman kalmanX, kalmanY;


unsigned long oldTime = 0, loopTime, nowTime;
float dt;


Preferences preferences;

enum RobotMode {
  MODE_MANUAL,
  MODE_GETUP,
  MODE_GETUP_DONE,
  MODE_GETUP_ABORT,
};

volatile RobotMode robotMode = MODE_MANUAL;
volatile bool autoGetupEnabled = true;

const float TARGET_DELTA_RAD = 0.08f;
const float TARGET_LIMIT_RAD = 1.55f;
const unsigned long GETUP_PERIOD_US = 20000;
const unsigned long GETUP_TIMEOUT_MS = 14000;
const float GETUP_DONE_TILT_RAD = 0.25f;
const unsigned long GETUP_DONE_HOLD_MS = 700;
const float AUTO_FALL_TILT_RAD = 0.75f;
const unsigned long AUTO_FALL_HOLD_MS = 300;
const unsigned long AUTO_RESTART_COOLDOWN_MS = 1500;

unsigned long lastPolicyUs = 0;
unsigned long getupStartMs = 0;
unsigned long uprightStartMs = 0;
unsigned long fallStartMs = 0;
unsigned long lastGetupEndMs = 0;

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void writeServoTargetsRad(float target1, float target2) {
  int deg1 = (int)roundf(target1 * RAD_TO_DEG);
  int deg2 = (int)roundf(target2 * RAD_TO_DEG);
  deg1 = (int)clampf(deg1, -90, 90);
  deg2 = (int)clampf(deg2, -90, 90);

  servo1.write(-deg1 + 90);
  servo2.write(-deg2 + 90);
}

void startGetupMode() {
  noInterrupts();
  autoGetupEnabled = true;
  servo1_target = 0.0f;
  servo2_target = 0.0f;
  robotMode = MODE_GETUP;
  interrupts();

  getupStartMs = millis();
  uprightStartMs = 0;
  lastPolicyUs = 0;
  writeServoTargetsRad(servo1_target, servo2_target);
}

void stopGetupMode(RobotMode nextMode = MODE_MANUAL) {
  noInterrupts();
  robotMode = nextMode;
  interrupts();
  lastGetupEndMs = millis();
  fallStartMs = 0;
}

const char *modeName() {
  switch (robotMode) {
    case MODE_GETUP: return "GETUP";
    case MODE_GETUP_DONE: return "DONE";
    case MODE_GETUP_ABORT: return "ABORT";
    default: return "MANUAL";
  }
}

void updateAutoGetup() {
  if (!autoGetupEnabled || robotMode == MODE_GETUP) {
    fallStartMs = 0;
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastGetupEndMs < AUTO_RESTART_COOLDOWN_MS) {
    fallStartMs = 0;
    return;
  }

  bool fallen = fabsf(roll) > AUTO_FALL_TILT_RAD || fabsf(pitch) > AUTO_FALL_TILT_RAD;
  if (!fallen) {
    fallStartMs = 0;
    if (robotMode == MODE_GETUP_DONE || robotMode == MODE_GETUP_ABORT) {
      robotMode = MODE_MANUAL;
    }
    return;
  }

  if (fallStartMs == 0) fallStartMs = nowMs;
  if (nowMs - fallStartMs > AUTO_FALL_HOLD_MS) {
    startGetupMode();
  }
}

void runGetupPolicy() {
  unsigned long nowUs = micros();
  if (lastPolicyUs != 0 && nowUs - lastPolicyUs < GETUP_PERIOD_US) {
    return;
  }
  lastPolicyUs = nowUs;

  float obs[OBS_DIM] = {
    roll,
    pitch,
    servo1_target,
    servo2_target,
  };
  float action[ACTION_DIM];
  forward_policy(obs, action);

  action[0] = clampf(action[0], -1.0f, 1.0f);
  action[1] = clampf(action[1], -1.0f, 1.0f);

  servo1_target = clampf(servo1_target + action[0] * TARGET_DELTA_RAD,
                         -TARGET_LIMIT_RAD, TARGET_LIMIT_RAD);
  servo2_target = clampf(servo2_target + action[1] * TARGET_DELTA_RAD,
                         -TARGET_LIMIT_RAD, TARGET_LIMIT_RAD);

  writeServoTargetsRad(servo1_target, servo2_target);

  unsigned long nowMs = millis();
  bool upright = fabsf(roll) < GETUP_DONE_TILT_RAD && fabsf(pitch) < GETUP_DONE_TILT_RAD;
  if (upright) {
    if (uprightStartMs == 0) uprightStartMs = nowMs;
    if (nowMs - uprightStartMs > GETUP_DONE_HOLD_MS) {
      stopGetupMode(MODE_GETUP_DONE);
    }
  } else {
    uprightStartMs = 0;
  }

  if (nowMs - getupStartMs > GETUP_TIMEOUT_MS) {
    stopGetupMode(MODE_GETUP_ABORT);
  }
}


// =================================
//   IMU
// =================================
void get_theta(){
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  theta_X  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  theta_Y  = -atan2(-accX, -accZ) * RAD_TO_DEG;
}

void get_gyro(){
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  gyroX = -gyroX;
}


//ブラウザ表示
void handleRoot() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>XL330-pend</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{";
  temp +="  max-width: 500px;";
  temp +="  margin: auto;";
  temp +="  text-align: center;";
  temp +="  font-size: 1.2rem;";
  temp +="}";
  temp +="span,.pm{";
  temp +="  display: inline-block;";
  temp +="  border: 1px solid #ccc;";
  temp +="  width: 50px;";
  temp +="  height: 30px;";
  temp +="  vertical-align: middle;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="span{";
  temp +="  width: 120px;";
  temp +="}";
  temp +="button{";
  temp +="  width: 100px;";
  temp +="  height: 40px;";
  temp +="  font-weight: bold;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="button.on{ background:lime; color:white; }";
  temp +=".column-3{ max-width:330px; margin:auto; text-align:center; display:flex; justify-content:space-between; flex-wrap:wrap; }";
  temp +="</style>";
  temp +="</head>";
  
  temp +="<body>";
  temp +="<div class=\"container\">";

  temp +="mode<br>";
  temp +="<span>" + String(modeName()) + "</span><br>";
  temp +="<a href=\"/getupStart\"><button class=\"on\">GETUP</button></a>";
  temp +="<a href=\"/getupStop\"><button>STOP</button></a><br>";
  temp +="auto getup<br>";
  temp +="<span>" + String(autoGetupEnabled ? "ON" : "OFF") + "</span><br>";
  temp +="<a href=\"/autoOn\"><button class=\"on\">AUTO ON</button></a>";
  temp +="<a href=\"/autoOff\"><button>AUTO OFF</button></a><br>";
  temp +="roll/pitch rad<br>";
  temp +="<span>" + String(roll, 3) + "</span>";
  temp +="<span>" + String(pitch, 3) + "</span><br>";
  temp +="target rad<br>";
  temp +="<span>" + String(servo1_target, 3) + "</span>";
  temp +="<span>" + String(servo2_target, 3) + "</span><br>";

  
  //ang1
  temp +="ang1<br>";
  temp +="<a class=\"pm\" href=\"/ang1M\">-</a>";
  temp +="<span>" + String(ang1) + "</span>";
  temp +="<a class=\"pm\" href=\"/ang1P\">+</a><br>";

  //offset1
  temp +="offset1<br>";
  temp +="<a class=\"pm\" href=\"/offset1M\">-</a>";
  temp +="<span>" + String(offset1) + "</span>";
  temp +="<a class=\"pm\" href=\"/offset1P\">+</a><br>";

  //pulse1
  temp +="pulse1<br>";
  temp +="<a class=\"pm\" href=\"/pulse1M\">-</a>";
  temp +="<span>" + String(pulse1) + "</span>";
  temp +="<a class=\"pm\" href=\"/pulse1P\">+</a><br>";

  //ang2
  temp +="ang2<br>";
  temp +="<a class=\"pm\" href=\"/ang2M\">-</a>";
  temp +="<span>" + String(ang2) + "</span>";
  temp +="<a class=\"pm\" href=\"/ang2P\">+</a><br>";

  //offset2
  temp +="offset2<br>";
  temp +="<a class=\"pm\" href=\"/offset2M\">-</a>";
  temp +="<span>" + String(offset2) + "</span>";
  temp +="<a class=\"pm\" href=\"/offset2P\">+</a><br>";

  //pulse2
  temp +="pulse2<br>";
  temp +="<a class=\"pm\" href=\"/pulse2M\">-</a>";
  temp +="<span>" + String(pulse2) + "</span>";
  temp +="<a class=\"pm\" href=\"/pulse2P\">+</a><br>";

  temp +="</div>";
  temp +="</body>";
  server.send(200, "text/HTML", temp);
}


void ang1M() {
  if(ang1 >= -90){
    ang1 -= 5;
  }
  handleRoot();
}
void ang1P() {
  if(ang1 <= 90){
    ang1 += 5;
  }
  handleRoot();
}

void offset1M() {
  if(offset1 >= -300){
    offset1 -= 2;
    preferences.putInt("offset1", offset1);
    servo1.detach();
    servo1.attach(26, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  }
  handleRoot();
}
void offset1P() {
  if(offset1 <= 300){
    offset1 += 2;
    preferences.putInt("offset1", offset1);
    servo1.detach();
    servo1.attach(26, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  }
  handleRoot();
}

void pulse1M() {
  if(pulse1 > 0){
    pulse1 -= 20;
    preferences.putInt("pulse1", pulse1);
    servo1.detach();
    servo1.attach(26, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  }
  handleRoot();
}
void pulse1P() {
  if(pulse1 <= 1200){
    pulse1 += 20;
    preferences.putInt("pulse1", pulse1);
    servo1.detach();
    servo1.attach(26, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  }
  handleRoot();
}



void ang2M() {
  if(ang2 >= -90){
    ang2 -= 5;
  }
  handleRoot();
}
void ang2P() {
  if(ang2 <= 90){
    ang2 += 5;
  }
  handleRoot();
}

void offset2M() {
  if(offset2 >= -300){
    offset2 -= 2;
    preferences.putInt("offset2", offset2);
    servo2.detach();
    servo2.attach(32, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);
  }
  handleRoot();
}
void offset2P() {
  if(offset2 <= 300){
    offset2 += 2;
    preferences.putInt("offset2", offset2);
    servo2.detach();
    servo2.attach(32, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);
  }
  handleRoot();
}

void pulse2M() {
  if(pulse2 > 0){
    pulse2 -= 20;
    preferences.putInt("pulse2", pulse2);
    servo2.detach();
    servo2.attach(32, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);
  }
  handleRoot();
}
void pulse2P() {
  if(pulse2 <= 1200){
    pulse2 += 20;
    preferences.putInt("pulse2", pulse2);
    servo2.detach();
    servo2.attach(32, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);
  }
  handleRoot();
}

void getupStart() {
  startGetupMode();
  handleRoot();
}

void getupStop() {
  autoGetupEnabled = false;
  stopGetupMode(MODE_MANUAL);
  handleRoot();
}

void autoOn() {
  autoGetupEnabled = true;
  stopGetupMode(MODE_MANUAL);
  handleRoot();
}

void autoOff() {
  autoGetupEnabled = false;
  stopGetupMode(MODE_MANUAL);
  handleRoot();
}


// =================================
// Core0
// =================================
void core0(void *){
  WiFi.softAP(ssid,pass);
  delay(100);
  WiFi.softAPConfig(ip,ip,subnet);

  server.on("/", handleRoot); 

  server.on("/ang1P", ang1P);
  server.on("/ang1M", ang1M);
  server.on("/offset1P", offset1P);
  server.on("/offset1M", offset1M);
  server.on("/pulse1P", pulse1P);
  server.on("/pulse1M", pulse1M);

  server.on("/ang2P", ang2P);
  server.on("/ang2M", ang2M);
  server.on("/offset2P", offset2P);
  server.on("/offset2M", offset2M);
  server.on("/pulse2P", pulse2P);
  server.on("/pulse2M", pulse2M);

  server.on("/getupStart", getupStart);
  server.on("/getupStop", getupStop);
  server.on("/autoOn", autoOn);
  server.on("/autoOff", autoOff);
  
  server.begin();

  for(;;){
    server.handleClient();

    
    disableCore0WDT();
  }
}

// =================================
// Setup
// =================================
void setup(){
  M5.begin(true, false, true); //SerialEnable, bool I2CEnable, DisplayEnable
  

  preferences.begin("parameter",false);

  offset1 = preferences.getInt("offset1", offset1);
  pulse1 = preferences.getInt("pulse1", pulse1);

  offset2 = preferences.getInt("offset2", offset2);
  pulse2 = preferences.getInt("pulse2", pulse2);


  M5.IMU.Init();

  //フルスケールレンジ
  M5.IMU.SetAccelFsr(M5.IMU.AFS_2G);
  M5.IMU.SetGyroFsr(M5.IMU.GFS_250DPS);

  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);

  servo1.setPeriodHertz(50);
  servo1.attach(26, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  servo2.setPeriodHertz(50);
  servo2.attach(32, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);

  // Core0 start
  xTaskCreatePinnedToCore(core0,"core0",4096,NULL,1,NULL,0);

  delay(500);
  oldTime = micros();
}

// =================================
// LOOP (制御)
// =================================
void loop(){
  M5.update();

  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = (float)loopTime / 1000000.0; //sec

  get_theta();
  get_gyro();
    
  kalAngleX = kalmanX.getAngle(theta_X, gyroX, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, gyroY, dt);
  
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();

  pitch = kalAngleX * DEG_TO_RAD;
  roll = -kalAngleY * DEG_TO_RAD;

  if (M5.Btn.wasPressed()) {
    if (robotMode == MODE_GETUP) {
      autoGetupEnabled = false;
      stopGetupMode(MODE_MANUAL);
    } else {
      startGetupMode();
    }
  }

  updateAutoGetup();

  /*
  Serial.print(modeName());
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(servo1_target);
  Serial.print(", ");
  Serial.println(servo2_target);
  */

  if (robotMode == MODE_GETUP) {
    runGetupPolicy();
    delay(1);
  } else {
    servo1.write(-ang1 + 90);
    servo2.write(-ang2 + 90);
    delay(10);
  }

}
