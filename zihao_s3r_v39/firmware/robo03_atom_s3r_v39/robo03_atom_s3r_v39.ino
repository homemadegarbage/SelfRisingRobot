// 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
// Original robot project: HomeMadeGarbage / SelfRisingRobot; upstream credit retained.
#include <M5Unified.h>
#include <Kalman.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <math.h>
#include "policy_network.h"
#include "walk_config.h"
#include "control_modes.h"
#include "v32_motion.h"
#include "roll_gaits.h"
#include "roll_controller.h"
#include "web_ui.h"
#include "icon_assets.h"
uint8_t iconTheme=0;
#include "recovery_network.h"

// Four modes. After IMU startup, Rise is the default autonomous mode.
const char* RELEASE_AUTHOR="同济子豪兄 2026-10-26";
WebServer server(80); DNSServer dns; Servo lowerServo, upperServo;
float targets[2]={0,0}, previousAction[2]={0,0}, commandXY[2]={0,0};
float commandYaw=0;
WiFiUDP liveUdp;
bool liveEnabled=false; // Deliberately not persisted: OFF on every boot.
uint32_t liveLastMs=0, liveSequence=0, liveBoot=0;
IPAddress livePeer;uint16_t livePort=4242;uint32_t livePeerMs=0;
bool livePeerKnown=false;
uint32_t liveSendErrors=0,liveSendMaxUs=0,loopMaxUs=0,commandGapMaxMs=0,commandTimeouts=0;
const char* stopReason="boot";
void receiveLiveSubscription(){
 int n=liveUdp.parsePacket();if(n<=0)return;
 char msg[32]={0};int count=liveUdp.read((uint8_t*)msg,sizeof(msg)-1);
 IPAddress peer=liveUdp.remoteIP();
 if(count==19&&!memcmp(msg,"ROBOBO_SUBSCRIBE_V1",19)&&peer[0]==192&&peer[1]==168&&peer[2]==42&&peer[3]>1&&peer[3]<255){
  livePeer=peer;livePort=liveUdp.remotePort();livePeerMs=millis();livePeerKnown=true;
 }
}
constexpr uint16_t liveHz=200; // Fixed pose telemetry target rate for V3.9.

bool screenPaused=false, returningHome=false;
uint32_t motionTick=0, actionStart=0;
int actionId=-1, rollDirection=0, rollPose=0;
float rollProgress=0;
float rollGoal[3]={0,0,1};
int rollStable=0;
bool struggleTriggered=false, rollFinished=false;
M5Canvas lcd(&M5.Display);
const float limits[2]={PI/2,2*PI/3};
int offsets[2]={0,0}, spans[2]={1000,1000};
float q[4]={1,0,0,0}, bodyGyro[3]={0,0,0};
ControlModes control;
#include "v31_recovery_core.h"
float &recoveryRoll=v31::roll, &recoveryPitch=v31::pitch;
float measuredUp=1;
bool recovering=false, servosAttached=false, imuReady=false;
uint32_t policyCount=0,snapCount=0,controlEpoch=1;
int recoveryStable=0;
uint32_t sessionToken=0,lastSequence=0,lastCommandMs=0,lastPolicyUs=0,lastImuUs=0;
uint32_t phaseStartMs=0,lastDisplayMs=0;
const uint32_t COMMAND_TIMEOUT_MS=400;
void sendLivePose(bool force=false){
 if(!liveEnabled&&!force)return;
 uint32_t now=micros();if(!force&&uint32_t(now-liveLastMs)<1000000UL/liveHz)return;
 liveLastMs=now;
 char packet[512];
 snprintf(packet,sizeof(packet),"{\"schema\":\"robobo.pose.v1\",\"boot\":%lu,\"seq\":%lu,\"ms\":%lu,\"enabled\":%s,\"valid\":%s,\"q\":[%.7f,%.7f,%.7f,%.7f],\"targets\":[%.6f,%.6f]}",
 (unsigned long)liveBoot,(unsigned long)liveSequence++,(unsigned long)millis(),
 liveEnabled?"true":"false",imuReady?"true":"false",q[0],q[1],q[2],q[3],targets[0],targets[1]);
 bool unicast=livePeerKnown&&uint32_t(millis()-livePeerMs)<3500;
 uint32_t started=micros();
 if(liveUdp.beginPacket(unicast?livePeer:IPAddress(192,168,42,255),unicast?livePort:4242)){
  liveUdp.write((const uint8_t*)packet,strlen(packet));if(!liveUdp.endPacket())liveSendErrors++;
 }else liveSendErrors++;
 liveSendMaxUs=max(liveSendMaxUs,uint32_t(micros()-started));
}

float clipf(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
void normalizeQ(){float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);if(n>1e-6f)for(int i=0;i<4;i++)q[i]/=n;}
void matrix(float R[9]){
 float w=q[0],x=q[1],y=q[2],z=q[3];
 R[0]=1-2*(y*y+z*z);R[1]=2*(x*y-w*z);R[2]=2*(x*z+w*y);
 R[3]=2*(x*y+w*z);R[4]=1-2*(x*x+z*z);R[5]=2*(y*z-w*x);
 R[6]=2*(x*z-w*y);R[7]=2*(y*z+w*x);R[8]=1-2*(x*x+y*y);
}
void writeTargets(){
 if(servosAttached)v31::writeServoTargetsRad(targets[0],targets[1]);
}
void stopMotion(){commandYaw=0;commandXY[0]=commandXY[1]=0;previousAction[0]=previousAction[1]=0;}
void emergencyStop(const char* reason="manual"){stopReason=reason;control.stop();stopMotion();sessionToken=0;controlEpoch++;returningHome=false;actionId=-1;rollDirection=0;rollFinished=false;}
void returnHome(){emergencyStop();returningHome=true;motionTick=micros();}
void imuFault(){imuReady=false;emergencyStop("imu_fault");}
// Walk orientation is an observer of V3.1 samples, never a second IMU reader.
// Its quaternion state cannot change V3.1 roll/pitch or Kalman state.
void initializeWalkOrientation(){
 float roll=recoveryRoll,pitch=recoveryPitch;
 float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
 q[0]=cr*cp;q[1]=sr*cp;q[2]=cr*sp;q[3]=-sr*sp;normalizeQ();lastImuUs=micros();
}
void updateWalkOrientation(){
 uint32_t now=micros();if(uint32_t(now-lastImuUs)<4000)return;
 float dt=uint32_t(now-lastImuUs)*1e-6f;lastImuUs=now;
 if(dt>.1f)return; // Do not integrate a long UI stall into Walk orientation.
 float ax=v31::accX,ay=v31::accY,az=v31::accZ;
 bodyGyro[0]=-v31::gyroY*DEG_TO_RAD;
 bodyGyro[1]=v31::gyroX*DEG_TO_RAD; // V3.1 already negates gyroX
 bodyGyro[2]=-v31::gyroZ*DEG_TO_RAD;
 float g[3]={-ay,-ax,-az},n=sqrtf(ax*ax+ay*ay+az*az),R[9];matrix(R);
 float w[3]={bodyGyro[0],bodyGyro[1],bodyGyro[2]};
 if(n>.8f && n<1.2f){
  for(int i=0;i<3;i++)g[i]/=n;
  w[0]+=.7f*(g[1]*R[8]-g[2]*R[7]);
  w[1]+=.7f*(g[2]*R[6]-g[0]*R[8]);
  w[2]+=.7f*(g[0]*R[7]-g[1]*R[6]);
 }
 float a=q[0],b=q[1],c=q[2],d=q[3],h=.5f*dt;
 q[0]+=h*(-b*w[0]-c*w[1]-d*w[2]);q[1]+=h*(a*w[0]+c*w[2]-d*w[1]);
 q[2]+=h*(a*w[1]+d*w[0]-b*w[2]);q[3]+=h*(a*w[2]+b*w[1]-c*w[0]);normalizeQ();
 for(int i=0;i<4;i++)if(!isfinite(q[i])){return;}
}
void establishStartupForward(){
 // IMU Y+ = policy body -X. Set its horizontal projection to world +X once at boot.
 // If that axis is vertical, use the perpendicular horizontal axis as a fallback.
 float R[9];matrix(R);float fx=-R[0],fy=-R[3];
 if(hypotf(fx,fy)<.05f){fx=-R[4];fy=R[1];}
 float yaw=atan2f(fy,fx),c=cosf(-yaw/2),s=sinf(-yaw/2);
 float a=q[0],b=q[1],d=q[2],e=q[3];q[0]=c*a-s*e;q[1]=c*b-s*d;q[2]=c*d+s*b;q[3]=c*e+s*a;normalizeQ();
}
void attachServos(){
 if(servosAttached)return;
 // Preserve the exact V3.1 attach sequence (effective 10-bit in this library).
 lowerServo.setTimerWidth(14);upperServo.setTimerWidth(14);
 lowerServo.attach(1,500,2500);upperServo.attach(2,500,2500);
 servosAttached=lowerServo.attached()&&upperServo.attached();
 if(!servosAttached){emergencyStop();return;}
 writeTargets();
}
const char* modeId(){return control.mode==ControlMode::Rise?"rise":control.mode==ControlMode::Balance?"balance":control.mode==ControlMode::Action?"action":"walk";}
const char* stateId(){
 if(!imuReady)return "imu_error";
 if(returningHome)return "returning_home";
 if(screenPaused)return "screen_paused";
 if(!control.enabled)return "stopped";
 if(control.mode==ControlMode::Action)return actionId==3&&!struggleTriggered?"touch_wait":actionId<0?"action_idle":"acting";
 if(rollFinished)return "roll_done";
 if(rollDirection)return "rolling";
 if(control.mode==ControlMode::Balance)return "balancing";
 if(control.mode==ControlMode::Walk)return (hypotf(commandXY[0],commandXY[1])<.05f&&fabsf(commandYaw)<.05f)?"walk_idle":recovering?"recovering":"crawling";
 return control.phase==RisePhase::Waiting?"charging":control.phase==RisePhase::Acting?"rising":control.phase==RisePhase::Cooldown?(control.completed&&measuredUp>.8f&&fabsf(recoveryRoll)<.25f&&fabsf(recoveryPitch)<.25f?"cooldown":"retry"):"watching";
}
void selectMode(ControlMode mode){
 stopReason="none";returningHome=false;actionId=-1;rollDirection=0;rollFinished=false;stopMotion();recovering=false;recoveryStable=0;controlEpoch++;control.select(mode,millis());attachServos();
 if(!servosAttached)return;
 sessionToken=esp_random();if(!sessionToken)sessionToken=1;lastSequence=0;
 lastCommandMs=millis();phaseStartMs=millis();lastPolicyUs=0;v31::lastPolicyUs=0;
}
void recoveryAction(const float R[9],float action[2]){
 float obs[4]={recoveryRoll,recoveryPitch,targets[0],targets[1]};
 recovery_policy::forward_policy(obs,action);policyCount++;
}
bool applyAction(const float action[2],float delta){
 if(!isfinite(action[0])||!isfinite(action[1])){emergencyStop();return false;}
 for(int i=0;i<2;i++){previousAction[i]=clipf(action[i],-1,1);targets[i]=clipf(targets[i]+delta*previousAction[i],-limits[i],limits[i]);}
 writeTargets();return true;
}
void runWalk(const float R[9]){
 float up[3]={R[6],R[7],R[8]},n=sqrtf(up[0]*up[0]+up[1]*up[1]+up[2]*up[2]);
 if(!isfinite(n)||n<.1f){emergencyStop();return;}
 for(int i=0;i<3;i++)up[i]/=n;
 // Screen normal / zero-position rod = body +Z. Project onto the ground.
 float f[3]={-up[0]*up[2],-up[1]*up[2],1-up[2]*up[2]};
 n=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
 if(n<.15f){f[0]=1-up[0]*up[0];f[1]=-up[1]*up[0];f[2]=-up[2]*up[0];n=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);}
 for(int i=0;i<3;i++)f[i]/=n;
 float left[3]={up[1]*f[2]-up[2]*f[1],up[2]*f[0]-up[0]*f[2],up[0]*f[1]-up[1]*f[0]};
 float obs[OBS_DIM]={0};
 for(int i=0;i<3;i++){obs[i]=up[i];obs[3+i]=f[i]*commandXY[0]+left[i]*commandXY[1];obs[6+i]=clipf(bodyGyro[i]/10,-3,3);}
 obs[9]=targets[0]/limits[0];obs[10]=targets[1]/limits[1];obs[11]=previousAction[0];obs[12]=previousAction[1];
 int gaitIndex=0;
#if WALK_GUIDED
 if(fabsf(commandYaw)>.4f)gaitIndex=commandYaw>0?4:5;
 else if(fabsf(commandXY[0])>=fabsf(commandXY[1]))gaitIndex=commandXY[0]>=0?0:1;
 else gaitIndex=commandXY[1]>=0?2:3;
#if WALK_GUIDED_MULTIPOSE
 int poseIndex=fabsf(up[0])>=fabsf(up[1])?(up[0]>=0?0:1):(up[1]>=0?2:3);
 gaitIndex+=6*poseIndex;
#endif
 double period=WALK_GAITS[gaitIndex][5];
#else
 double period=1.6;
#endif
 float phase=2*PI*fmod(uint32_t(millis()-phaseStartMs)*.001,period)/period;obs[13]=sinf(phase);obs[14]=cosf(phase);obs[15]=commandYaw;
 float action[2];forward_policy(obs,action);policyCount++;
 if(!isfinite(action[0])||!isfinite(action[1])){emergencyStop();return;}
 for(int i=0;i<2;i++)action[i]=clipf(action[i],-1,1);
#if WALK_GUIDED
 float residualScale=(gaitIndex%6==0)?WALK_FORWARD_RESIDUAL_SCALE:WALK_RESIDUAL_SCALE;
#if WALK_RESIDUAL_PITCH_ONLY
 if(gaitIndex>=6)residualScale=0;
#endif
 for(int i=0;i<2;i++){
  float desired=limits[i]*clipf(WALK_GAITS[gaitIndex][i]+WALK_GAITS[gaitIndex][2+i]*sinf(phase+(i?WALK_GAITS[gaitIndex][4]:0))+residualScale*action[i],-1,1);
  action[i]=clipf((desired-targets[i])/WALK_DELTA_RAD,-1,1);
 }
#elif WALK_CPG
 for(int i=0;i<2;i++){
  float desired=limits[i]*clipf(.45f*sinf(phase+(i?PI/2:0))+.55f*action[i],-1,1);
  action[i]=clipf((desired-targets[i])/WALK_DELTA_RAD,-1,1);
 }
#endif
 applyAction(action,WALK_DELTA_RAD);
}
void runV32Motion(){
 uint32_t now=micros();if(uint32_t(now-motionTick)<20000)return;motionTick=now;
 if(returningHome){
  bool done=true;for(int i=0;i<2;i++){targets[i]=v32Approach(targets[i],0,.012f);done=done&&fabsf(targets[i])<.0001f;}
  writeTargets();if(done)returningHome=false;return;
 }
 if(!control.enabled||!imuReady)return;
 if(control.mode==ControlMode::Action){
  if(uint32_t(millis()-lastCommandMs)>COMMAND_TIMEOUT_MS){commandTimeouts++;emergencyStop("command_timeout");return;}
  if(actionId<0)return;
  float t=uint32_t(millis()-actionStart)*.001f;
  if(actionId==3&&!struggleTriggered){
   if(t>1&&(sqrtf(bodyGyro[0]*bodyGyro[0]+bodyGyro[1]*bodyGyro[1]+bodyGyro[2]*bodyGyro[2])>1.2f||fabsf(sqrtf(v31::accX*v31::accX+v31::accY*v31::accY+v31::accZ*v31::accZ)-1.f)>.25f)){struggleTriggered=true;actionStart=millis();}
   return;
  }
  float desired[2];bool running=v32Pose(actionId,t,desired);
  for(int i=0;i<2;i++)targets[i]=v32Approach(targets[i],clipf(desired[i],-limits[i],limits[i]),actionId==5?.045f:.025f);
  writeTargets();if(!running&&fabsf(targets[0])<.001f&&fabsf(targets[1])<.001f)actionId=-1;
 }else if(control.mode==ControlMode::Walk&&rollDirection){
  if(uint32_t(millis()-lastCommandMs)>COMMAND_TIMEOUT_MS){commandTimeouts++;emergencyStop("command_timeout");return;}
  float R[9];matrix(R);float up[3]={R[6],R[7],R[8]};
  float n=sqrtf(up[0]*up[0]+up[1]*up[1]+up[2]*up[2]);if(!isfinite(n)||n<.1f){emergencyStop();return;}
  for(int i=0;i<3;i++)up[i]/=n;
  float f[3]={-up[0]*up[2],-up[1]*up[2],1-up[2]*up[2]};float fn=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
  if(fn>.15f)rollProgress+=rollDirection*(bodyGyro[0]*f[0]+bodyGyro[1]*f[1]+bodyGyro[2]*f[2])*.02f/fn;
  float score=up[0]*rollGoal[0]+up[1]*rollGoal[1]+up[2]*rollGoal[2];
  rollStable=score>.92f&&fabsf(up[2])<.7f&&rollProgress>1.f?rollStable+1:0;
  float t=uint32_t(millis()-actionStart)*.001f;
  if(rollStable>=5||t>=6){rollDirection=0;rollFinished=true;stopMotion();return;}
  int index=rollPose*2+(rollDirection>0?0:1);float obs[13]={0},action[2]={0};
  for(int i=0;i<3;i++){obs[i]=up[i];obs[3+i]=rollGoal[i];obs[6+i]=clipf(bodyGyro[i]/10,-3,3);}
  for(int i=0;i<2;i++){obs[9+i]=targets[i]/limits[i];obs[11+i]=previousAction[i];}
  if(rollNetwork(index,obs,action)){
   if(!isfinite(action[0])||!isfinite(action[1])){emergencyStop();return;}
   applyAction(action,.1f);
  }else{
   const float *g=ROLL_GAITS[index];
   for(int i=0;i<2;i++){float desired=limits[i]*clipf(g[i]+g[i+2]*sinf(2*PI*t/g[5]+(i?g[4]:0)),-1,1);float next=v32Approach(targets[i],desired,.1f);previousAction[i]=clipf((next-targets[i])/.1f,-1,1);targets[i]=next;}
  }
  writeTargets();
 }
}
void bootScreen(){
 M5.Display.setTextWrap(false);
 for(int y=0;y<128;y++)M5.Display.drawFastHLine(0,y,128,M5.Display.color565(255,210+y/4,170+y/3));
 M5.Display.fillCircle(12,16,25,0xFE90);M5.Display.fillCircle(119,73,20,0xFBD2);
 M5.Display.setTextColor(0x8208);M5.Display.setTextSize(3);M5.Display.setCursor(10,25);M5.Display.print("RoBoBo");
 M5.Display.setTextSize(2);M5.Display.setCursor(40,60);M5.Display.print("V3.9");
 M5.Display.setTextSize(1);M5.Display.setCursor(19,93);M5.Display.print("HomeMadeGarbage");
 M5.Display.setCursor(28,106);M5.Display.print("&TommyZihao");M5.Display.setCursor(43,118);M5.Display.print("&Feisuo");
}
// Display classification only; never changes policy observations or triggers.
const char* fallDirection(){
 if(!imuReady)return "imu_error";
 float x=-sinf(recoveryPitch),y=sinf(recoveryRoll)*cosf(recoveryPitch),z=cosf(recoveryRoll)*cosf(recoveryPitch);
 if(z>.75f)return "upright";
 if(z<-.75f)return "inverted";
 if(fabsf(x)>=fabsf(y))return x<0?"pitch_pos":"pitch_neg";
 return y>0?"roll_pos":"roll_neg";
}
void showTelemetry(){
 const char* pose=fallDirection();
 int r=0,g=215,b=120; // upright: emerald
 if(!strcmp(pose,"pitch_pos")){r=255;g=205;b=35;}
 else if(!strcmp(pose,"pitch_neg")){r=65;g=185;b=255;}
 else if(!strcmp(pose,"roll_pos")){r=255;g=135;b=150;}
 else if(!strcmp(pose,"roll_neg")){r=190;g=145;b=255;}
 else if(!strcmp(pose,"inverted")){r=255;g=90;b=60;}
 else if(!strcmp(pose,"imu_error")){r=g=b=130;}
 if(screenPaused){r=255;g=30;b=30;}
 uint16_t color=((r>>3)<<11)|((g>>2)<<5)|(b>>3);
 lcd.fillSprite(color);lcd.setTextColor(0x0000);lcd.setTextWrap(false);
 const char* label=screenPaused?"PAUSED":modeId();
 float size=3.5f;lcd.setTextSize(size);
 while(lcd.textWidth(label)>120&&size>1){size-=.1f;lcd.setTextSize(size);}
 lcd.setCursor((128-lcd.textWidth(label))/2,(128-lcd.fontHeight())/2);lcd.print(label);lcd.pushSprite(0,0);
}
void runPolicy(){
 if(!imuReady||!control.enabled||returningHome||screenPaused||control.mode==ControlMode::Action)return;
 if(control.mode==ControlMode::Walk){
  if(uint32_t(millis()-lastCommandMs)>COMMAND_TIMEOUT_MS){commandTimeouts++;emergencyStop("command_timeout");return;}
  if(rollDirection||rollFinished)return;
  uint32_t now=micros();if(lastPolicyUs&&uint32_t(now-lastPolicyUs)<20000)return;lastPolicyUs=now;
  if((hypotf(commandXY[0],commandXY[1])>=.05f||fabsf(commandYaw)>=.05f)){float R[9];matrix(R);runWalk(R);}
  return;
 }
 // Mode wrapper only: both autonomous modes execute the original V3.1 controller.
 ControlStep step=control.tick(millis(),recoveryRoll,recoveryPitch,false,measuredUp);
 if(step==ControlStep::Snap){targets[0]=targets[1]=0;previousAction[0]=previousAction[1]=0;writeTargets();snapCount++;}
 else if(step==ControlStep::Recovery){
  unsigned long before=v31::lastPolicyUs;v31::runGetupPolicy();
  if(before!=v31::lastPolicyUs)policyCount++;
 }
}
void sendState(bool includeSession=false){
 float R[9];matrix(R);
 String j="{\"mode\":\""+String(modeId())+"\",\"state\":\""+stateId()+"\",\"enabled\":"+(control.enabled?"true":"false")+",\"imuReady\":"+(imuReady?"true":"false")+",\"tilt\":"+String(acosf(clipf(cosf(recoveryRoll)*cosf(recoveryPitch),-1,1))*RAD_TO_DEG,1)+",\"policyCount\":"+String(policyCount)+",\"snapCount\":"+String(snapCount)+",\"target1\":"+String(targets[0],4)+",\"target2\":"+String(targets[1],4);
 j+=",\"screenPaused\":"+String(screenPaused?"true":"false");
 j+=",\"pwm1\":"+String(constrain(int(roundf(1500+offsets[0]-targets[0]/limits[0]*spans[0])),500,2500));
 j+=",\"pwm2\":"+String(constrain(int(roundf(1500+offsets[1]-targets[1]/limits[1]*spans[1])),500,2500));
 j+=",\"liveHz\":"+String(liveHz);
 j+=",\"liveEnabled\":"+String(liveEnabled?"true":"false");
 j+=",\"transport\":\""+String(livePeerKnown&&uint32_t(millis()-livePeerMs)<3500?"unicast":"broadcast")+"\",\"stopReason\":\""+String(stopReason)+"\"";
 j+=",\"commandTimeouts\":"+String(commandTimeouts)+",\"commandGapMaxMs\":"+String(commandGapMaxMs)+",\"loopMaxUs\":"+String(loopMaxUs)+",\"liveSendMaxUs\":"+String(liveSendMaxUs)+",\"liveSendErrors\":"+String(liveSendErrors);
 j+=",\"epoch\":"+String(controlEpoch);
 j+=",\"fallDirection\":\""+String(fallDirection())+"\",\"yaw\":"+String(atan2f(R[3],R[0]),4);
 j+=",\"roll\":"+String(recoveryRoll,4)+",\"pitch\":"+String(recoveryPitch,4)+",\"measuredUp\":"+String(measuredUp,4);
 if(includeSession)j+=",\"token\":"+String(sessionToken);
 server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",j+"}");
}
void setup(){
 auto cfg=M5.config();cfg.serial_baudrate=115200;cfg.internal_imu=true;cfg.fallback_board=m5::board_t::board_M5AtomS3R;M5.begin(cfg);
 liveBoot=esp_random();iconTheme=esp_random()%4;
 bootScreen();lcd.createSprite(128,128);
 Preferences prefs;prefs.begin("robo",true);offsets[0]=prefs.getInt("offset1",0);offsets[1]=prefs.getInt("offset2",0);spans[0]=prefs.getInt("pulse1",1000);spans[1]=prefs.getInt("pulse2",1000);prefs.end();
 delay(2000);imuReady=M5.Imu.isEnabled();
 IPAddress ip(192,168,42,1);WiFi.softAPConfig(ip,ip,IPAddress(255,255,255,0));WiFi.softAP("RoBoBo");dns.start(53,"*",ip);liveUdp.begin(4242);
 server.on("/icons.jpg",HTTP_GET,[]{server.sendHeader("Cache-Control","no-store");server.send_P(200,"image/jpeg",(const char*)iconImages[iconTheme],iconSizes[iconTheme]);});
 server.on("/",HTTP_GET,[]{server.sendHeader("Cache-Control","no-store");server.send(200,"text/html",PAGE);});
 server.on("/mode",HTTP_POST,[]{
  if(screenPaused){server.send(409,"text/plain","请按屏幕恢复");return;}
  if(!imuReady){server.send(409,"text/plain","IMU 未就绪，请保持静止并重启");return;}
  if(!server.hasArg("epoch")||strtoul(server.arg("epoch").c_str(),nullptr,10)!=controlEpoch){server.send(409,"text/plain","控制状态已变化，请重试");return;}
  String id=server.arg("mode");ControlMode next;
  if(id=="rise")next=ControlMode::Rise;else if(id=="balance")next=ControlMode::Balance;else if(id=="walk")next=ControlMode::Walk;else if(id=="action")next=ControlMode::Action;else{server.send(400,"text/plain","未知模式");return;}
  selectMode(next);sendState(true);
 });
 server.on("/stop",HTTP_POST,[]{emergencyStop();sendState();});
 server.on("/home",HTTP_POST,[]{returnHome();sendState();});
 server.on("/cmd",HTTP_POST,[]{
  if(!control.enabled||screenPaused||!(control.mode==ControlMode::Walk||control.mode==ControlMode::Action)||!sessionToken||strtoul(server.arg("token").c_str(),nullptr,10)!=sessionToken){server.send(409,"text/plain","inactive");return;}
  uint32_t seq=strtoul(server.arg("seq").c_str(),nullptr,10);if(seq<=lastSequence){server.send(200,"text/plain","stale");return;}
  String cmd=server.arg("cmd");
  if(control.mode==ControlMode::Action){
   if(cmd!="heartbeat"){
    const char* ids[]={"nod","bow","look","struggle","crazy","hop"};int id=-1;for(int i=0;i<6;i++)if(cmd==ids[i])id=i;
    if(id<0){server.send(400,"text/plain","unknown action");return;}actionId=id;actionStart=millis();struggleTriggered=false;
   }
  }else{
   if(cmd!="forward"&&cmd!="backward"&&cmd!="roll_left"&&cmd!="roll_right"&&cmd!="yaw_left"&&cmd!="yaw_right"&&cmd!="idle"){server.send(400,"text/plain","unknown command");return;}
   int nextRoll=cmd=="roll_left"?-1:cmd=="roll_right"?1:0;
   if(nextRoll&&nextRoll!=rollDirection&&!rollFinished){float R[9];matrix(R);rollPose=fabsf(R[6])>=fabsf(R[7])?(R[6]>=0?0:1):(R[7]>=0?2:3);actionStart=millis();rollGoal[0]=nextRoll*R[7];rollGoal[1]=-nextRoll*R[6];rollGoal[2]=R[8];rollStable=0;rollProgress=0;previousAction[0]=previousAction[1]=0;}
   if(!nextRoll)rollFinished=false;
   rollDirection=rollFinished?0:nextRoll;
   commandXY[0]=cmd=="forward"?1:cmd=="backward"?-1:0;commandXY[1]=0;commandYaw=cmd=="yaw_left"?1:cmd=="yaw_right"?-1:0;
   if(cmd=="idle")previousAction[0]=previousAction[1]=0;
  }
  commandGapMaxMs=max(commandGapMaxMs,uint32_t(millis()-lastCommandMs));lastSequence=seq;lastCommandMs=millis();server.send(200,"text/plain","OK");
 });
 server.on("/live",HTTP_POST,[]{
  String value=server.arg("enabled");
  if(value!="0"&&value!="1"){server.send(400,"text/plain","enabled must be 0 or 1");return;}
  // Rate is fixed; legacy hz arguments cannot change it.
  liveEnabled=value=="1";sendLivePose(true);sendState();
 });
 server.on("/data",HTTP_GET,[]{sendState();});
 server.onNotFound([]{server.sendHeader("Location","http://192.168.42.1/");server.send(302,"text/plain","");});server.begin();
 if(imuReady){v31::begin();initializeWalkOrientation();establishStartupForward();selectMode(ControlMode::Rise);}
 lastImuUs=micros();phaseStartMs=millis();
 Serial.printf("ROBOBO_V3_9 author=同济子豪兄_2026-10-26 ready imu=%d default=rise front=boot_IMU_Y+ GPIO1=lower GPIO2=upper IP=192.168.42.1\n",imuReady);
}
void loop(){
 uint32_t loopStart=micros();
 if(Serial.available()&&Serial.read()=='?')Serial.printf("IMU_STATE roll=%.2f pitch=%.2f up=%.3f state=%s\n",recoveryRoll*RAD_TO_DEG,recoveryPitch*RAD_TO_DEG,measuredUp,stateId());
 M5.update();
 static bool longHandled=false;
 if(M5.BtnA.wasPressed())longHandled=false;
 if(M5.BtnA.pressedFor(800)&&!longHandled){
  longHandled=true;
  if(imuReady){screenPaused=false;selectMode(control.mode==ControlMode::Rise?ControlMode::Balance:ControlMode::Rise);}
 }
 if(M5.BtnA.wasReleased()&&!longHandled){
  if(screenPaused){screenPaused=false;if(imuReady)selectMode(ControlMode::Rise);}
  else{screenPaused=true;returnHome();}
 }
 if(imuReady){
  v31::updateObservation();
  if(!isfinite(recoveryRoll)||!isfinite(recoveryPitch)){imuFault();}
  float n=sqrtf(v31::accX*v31::accX+v31::accY*v31::accY+v31::accZ*v31::accZ);
  if(n>.1f)measuredUp=-v31::accZ/n;
 }
 runPolicy();updateWalkOrientation();runV32Motion();server.handleClient();dns.processNextRequest();receiveLiveSubscription();sendLivePose();
 if(millis()-lastDisplayMs>200){lastDisplayMs=millis();showTelemetry();}
 loopMaxUs=max(loopMaxUs,uint32_t(micros()-loopStart));
}
