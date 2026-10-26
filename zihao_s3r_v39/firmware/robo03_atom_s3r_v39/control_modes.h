// 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
// Original robot project: HomeMadeGarbage / SelfRisingRobot; upstream credit retained.
#pragma once
#include <stdint.h>
#include <math.h>
enum class ControlMode { Rise, Balance, Walk, Action };
enum class ControlStep { Hold, Recovery, Walk, Snap };
enum class RisePhase { Watching, Waiting, Acting, Cooldown };
struct ControlModes {
 ControlMode mode=ControlMode::Rise;
 RisePhase phase=RisePhase::Watching;
 bool enabled=false, uprightTiming=false, completed=false;
 uint32_t phaseSince=0, uprightSince=0;
 static constexpr float TRIGGER=.75f, UPRIGHT=.25f;
 static constexpr uint32_t WAIT_MS=300, UPRIGHT_MS=700, TIMEOUT_MS=14000, COOLDOWN_MS=1500;
 void select(ControlMode next,uint32_t now){mode=next;enabled=true;phase=RisePhase::Watching;phaseSince=now;uprightTiming=false;completed=false;}
 void stop(){completed=false;enabled=false;phase=RisePhase::Watching;uprightTiming=false;}
 ControlStep tick(uint32_t now,float roll,float pitch,bool walkCommand,float measuredUp=1.f){
  if(!enabled)return ControlStep::Hold;
  if(mode==ControlMode::Action)return ControlStep::Hold;
  if(mode==ControlMode::Balance)return ControlStep::Recovery;
  if(mode==ControlMode::Walk)return walkCommand?ControlStep::Walk:ControlStep::Hold;
  bool fallen=fabsf(roll)>TRIGGER||fabsf(pitch)>TRIGGER||measuredUp<0.f;
  if(phase==RisePhase::Cooldown){if(uint32_t(now-phaseSince)<COOLDOWN_MS)return ControlStep::Hold;phase=RisePhase::Watching;}
  if(phase==RisePhase::Watching){if(fallen){phase=RisePhase::Waiting;phaseSince=now;}return ControlStep::Hold;}
  if(phase==RisePhase::Waiting){
   if(!fallen){phase=RisePhase::Watching;return ControlStep::Hold;}
   if(uint32_t(now-phaseSince)<WAIT_MS)return ControlStep::Hold;
   phase=RisePhase::Acting;phaseSince=now;uprightTiming=false;
  }
  if(measuredUp>.8f&&fabsf(roll)<UPRIGHT&&fabsf(pitch)<UPRIGHT){
   if(!uprightTiming){uprightTiming=true;uprightSince=now;}
   if(uint32_t(now-uprightSince)>=UPRIGHT_MS){phase=RisePhase::Cooldown;phaseSince=now;uprightTiming=false;completed=true;return ControlStep::Snap;}
  }else uprightTiming=false;
  if(uint32_t(now-phaseSince)>=TIMEOUT_MS){phase=RisePhase::Cooldown;phaseSince=now;uprightTiming=false;completed=false;return ControlStep::Hold;}
  return ControlStep::Recovery;
 }
};
