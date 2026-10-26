// 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
// Original robot project: HomeMadeGarbage / SelfRisingRobot; upstream credit retained.
#pragma once
// Verified V3.1 recovery core. Functions below copied verbatim from the working sketch.
// Integration binds its targets/calibration/servos to the shared hardware; Walk never writes its filters.
#pragma push_macro("OBS_DIM")
#pragma push_macro("ACTION_DIM")
#undef OBS_DIM
#undef ACTION_DIM
namespace v31 {
using recovery_policy::forward_policy;
constexpr int OBS_DIM=4,ACTION_DIM=2;
float &servo1_target=::targets[0], &servo2_target=::targets[1];
int &offset1=::offsets[0], &offset2=::offsets[1], &pulse1=::spans[0], &pulse2=::spans[1];
Servo &servo1=::lowerServo, &servo2=::upperServo;
float accX=0,accY=0,accZ=0,gyroX=0,gyroY=0,gyroZ=0;
float theta_X=0,theta_Y=0,pitch=0,roll=0,kalAngleX=0,kalAngleY=0,kalAngleDotX=0,kalAngleDotY=0,dt=0;
Kalman kalmanX,kalmanY;
unsigned long oldTime=0,nowTime=0,loopTime=0,lastPolicyUs=0,uprightStartMs=0;
enum RobotMode {MODE_GETUP,MODE_GETUP_DONE};RobotMode robotMode=MODE_GETUP;
const float TARGET_DELTA_RAD=.08f,TARGET1_LIMIT_RAD=PI/2.0f,TARGET2_LIMIT_RAD=2.0f*PI/3.0f;
const unsigned long GETUP_PERIOD_US=20000,GETUP_DONE_HOLD_MS=700;
const float GETUP_DONE_TILT_RAD=.25f;
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
void get_theta(){
  M5.Imu.getAccelData(&accX,&accY,&accZ);
  theta_X  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  theta_Y  = -atan2(-accX, -accZ) * RAD_TO_DEG;
}
void get_gyro(){
  M5.Imu.getGyroData(&gyroX,&gyroY,&gyroZ);
  gyroX = -gyroX;
}
void writeServoTargetsRad(float target1, float target2) {
  target1 = clampf(target1, -TARGET1_LIMIT_RAD, TARGET1_LIMIT_RAD);
  target2 = clampf(target2, -TARGET2_LIMIT_RAD, TARGET2_LIMIT_RAD);
  int us1 = (int)roundf(1500.0f + offset1 - target1 / TARGET1_LIMIT_RAD * pulse1);
  int us2 = (int)roundf(1500.0f + offset2 - target2 / TARGET2_LIMIT_RAD * pulse2);
  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
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
                         -TARGET1_LIMIT_RAD, TARGET1_LIMIT_RAD);
  servo2_target = clampf(servo2_target + action[1] * TARGET_DELTA_RAD,
                         -TARGET2_LIMIT_RAD, TARGET2_LIMIT_RAD);

  writeServoTargetsRad(servo1_target, servo2_target);

  unsigned long nowMs = millis();
  bool upright = fabsf(roll) < GETUP_DONE_TILT_RAD && fabsf(pitch) < GETUP_DONE_TILT_RAD;
  if (upright) {
    if (uprightStartMs == 0) uprightStartMs = nowMs;
    if (nowMs - uprightStartMs > GETUP_DONE_HOLD_MS) {
      robotMode = MODE_GETUP_DONE;
    }
  } else {
    uprightStartMs = 0;
    robotMode = MODE_GETUP;
  }
}
void updateObservation(){
  // IMU 读取
  get_theta();
  get_gyro();

  // 时间计算
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = loopTime / 1000000.0f;

  // Kalman 滤波
  kalAngleX = kalmanX.getAngle(theta_X, gyroX, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, gyroY, dt);
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();
  pitch = kalAngleX * DEG_TO_RAD;
  roll = -kalAngleY * DEG_TO_RAD;

}
void begin(){
 M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg,m5::IMU_Class::axis_y_pos,m5::IMU_Class::axis_z_neg);
 get_theta();kalmanX.setAngle(theta_X);kalmanY.setAngle(theta_Y);
 pitch=theta_X*DEG_TO_RAD;roll=-theta_Y*DEG_TO_RAD;oldTime=micros();
}
} // namespace v31

#pragma pop_macro("ACTION_DIM")
#pragma pop_macro("OBS_DIM")
