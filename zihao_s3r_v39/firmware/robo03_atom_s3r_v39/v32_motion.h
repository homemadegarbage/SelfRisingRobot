// 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
// Original robot project: HomeMadeGarbage / SelfRisingRobot; upstream credit retained.
#pragma once
#include <math.h>
inline float v32Approach(float current,float goal,float step){return current<goal?fminf(current+step,goal):fmaxf(current-step,goal);}
inline bool v32Pose(int id,float t,float out[2]){
 out[0]=out[1]=0;float duration=id==1?4.f:id==3?3.f:5.f;if(t>=duration)return false;
 float envelope=fminf(1.f,fminf(t/.4f,(duration-t)/.6f));
 switch(id){
 case 0:out[1]=.4f*sinf(2*3.14159265f*t/1.4f);break;
 case 1:out[1]=.95f*sinf(3.14159265f*t/duration);out[0]=.15f*sinf(3.14159265f*t/duration);break;
 case 2:out[0]=.65f*sinf(2*3.14159265f*t/2.4f);break;
 case 3:out[0]=.5f*sinf(12*t);out[1]=.55f*sinf(15*t);break;
 case 4:out[0]=.6f*sinf(16*t);out[1]=.75f*sinf(19*t+.5f);break;
 case 5:out[0]=.22f*sinf(9*t);out[1]=.65f*sinf(9*t);break;
 }
 out[0]*=envelope;out[1]*=envelope;return true;
}
