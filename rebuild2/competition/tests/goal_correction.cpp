#include <cassert>
#include <cstring>
#include <iostream>
#include "../main/goal_correction.h"
using namespace robot;
bool equal(const Decision& a,const Decision& b) { return a.state==b.state && a.reason==b.reason && !memcmp(a.pwm,b.pwm,sizeof(a.pwm)); }
int main() {
  Camera c; c.flags=Ready; c.frameMs=50; c.version=3; Pwm pwm={175,175,175,175};
  auto baseline=decide(false,true,c,true,false,false,true,pwm,0);
  auto d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline));
  c.yellow={Unknown,90,0}; d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline));
  c.yellow={60,69,0}; d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline));
  c.yellow={60,90,25}; d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline));
  c.yellow={60,90,0}; d=baseline; correctGoal(d,c,0,0); assert(d.pwm[0]==155 && d.pwm[2]==195);
  std::cout<<"baseline 175,175,175,175; goal+60 155,155,195,195\n";
  c.yellow={300,90,0}; d=baseline; correctGoal(d,c,0,0); assert(d.pwm[0]==195 && d.pwm[2]==155);
  d=baseline; correctGoal(d,c,0,3); assert(equal(d,baseline));
  d=baseline; correctGoal(d,c,0,5); assert(equal(d,baseline));
  c.yellow={60,90,0}; c.flags=Ready|TargetBlue; d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline));
  c.blue={60,90,0}; d=baseline; correctGoal(d,c,0,0); assert(d.pwm[0]==155);
  c.flags=Ready|Escape|LineHazard; c.line=0; c.escape=Rear;
  baseline=decide(false,true,c,true,false,false,true,pwm,0); d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline) && d.state==State::Escape);
  c.flags=Ready; baseline=decide(false,true,c,true,false,false,false,pwm,0); d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline) && d.reason==StopReason::BallLost);
  baseline=decide(false,false,c,true,false,false,true,pwm,0); d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline) && d.reason==StopReason::CameraTimeout);
  baseline=decide(true,true,c,true,false,false,true,pwm,0); d=baseline; correctGoal(d,c,0,0); assert(equal(d,baseline) && d.reason==StopReason::Manual);
  c.flags=Ready; for(int left=140; left<=210; ++left) for(int right=140; right<=210; ++right) {
    Decision x; x.state=State::Chase; x.reason=StopReason::None; x.pwm[0]=x.pwm[1]=left; x.pwm[2]=x.pwm[3]=right;
    for(int angle=0;angle<360;angle+=15) { auto before=x; c.yellow={uint16_t(angle),90,0}; correctGoal(x,c,0,0);
      assert(x.state==before.state && x.reason==before.reason);
      for(int i=0;i<4;i++) { assert(x.pwm[i]>=140 && x.pwm[i]<=210); assert(abs(x.pwm[i]-before.pwm[i])<=20); }
      x=before;
    }
  }
  std::cout<<"goal fallback, selection, line, stops and bounds OK\n";
}
