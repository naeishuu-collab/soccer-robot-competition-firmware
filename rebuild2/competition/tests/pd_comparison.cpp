#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include "../main/control.h"
struct Sample { int angle, ms; };
struct Stats { int flips=0, wrong=0, turnVariation=0, pwmVariation=0, responseMs=-1; };
Stats run(const std::vector<Sample>& samples, bool legacy, bool step=false) {
  ChasePD pd; pd.legacy=legacy; Stats out; int prevTurn=0, prevPwm[4]={}, prevSign=0, stepMs=-1;
  for (const auto& s : samples) {
    if (s.angle < 0) { pd.reset(); prevTurn=prevSign=0; for (auto &v:prevPwm) v=0; continue; }
    pd.update(uint16_t(s.angle), 100, uint32_t(s.ms));
    int sign=(pd.turn>0)-(pd.turn<0);
    if (sign && prevSign && sign!=prevSign) ++out.flips;
    if (sign) prevSign=sign;
    int signedAngle=s.angle>180?s.angle-360:s.angle;
    if (signedAngle && sign*signedAngle<0) ++out.wrong;
    out.turnVariation+=std::abs(pd.turn-prevTurn); prevTurn=pd.turn;
    for(int i=0;i<4;++i) { out.pwmVariation+=std::abs(pd.pwm[i]-prevPwm[i]); prevPwm[i]=pd.pwm[i]; assert(std::abs(pd.pwm[i])<=210); }
    if (step && signedAngle==60 && stepMs<0) stepMs=s.ms;
    if (step && stepMs>=0 && out.responseMs<0 && pd.turn>=54) out.responseMs=s.ms-stepMs;
    assert(pd.pwm[0]==pd.pwm[1] && pd.pwm[2]==pd.pwm[3]);
  }
  return out;
}
void compare(const char* name, const std::vector<Sample>& data, bool step=false) {
  auto a=run(data,true,step), b=run(data,false,step);
  std::cout<<name<<" old/new flips="<<a.flips<<"/"<<b.flips<<" wrong="<<a.wrong<<"/"<<b.wrong
   <<" turn_delta="<<a.turnVariation<<"/"<<b.turnVariation<<" pwm_delta="<<a.pwmVariation<<"/"<<b.pwmVariation
   <<" response_ms="<<a.responseMs<<"/"<<b.responseMs<<"\n";
}
int main() {
  std::vector<Sample> stable, moving, noise, step, wrap, reacquire, irregular;
  int t=0; const int jitter[]={20,45,25,38,22,60};
  for(int i=0;i<80;++i) {
    t+=30; stable.push_back({20,t});
    int position = i%40<20 ? -60+(i%40)*6 : 60-(i%40-20)*6;
    moving.push_back({position<0 ? 360+position : position,t});
    noise.push_back({(i%4==0?2:i%4==1?358:i%4==2?3:357),t});
    step.push_back({i<10?0:60,t});
    wrap.push_back({i%4==0?358:i%4==1?359:i%4==2?0:1,t});
    reacquire.push_back({i==30?-1:(i<30?60:300),t});
    int angle=(i%6==0?40:i%6==1?30:i%6==2?45:i%6==3?35:i%6==4?42:33);
    irregular.push_back({angle, i==0?20:irregular.back().ms+jitter[i%6]});
  }
  assert(run(noise,false).turnVariation < run(noise,true).turnVariation);
  assert(run(irregular,false).turnVariation < run(irregular,true).turnVariation);
  assert(run(step,false,true).responseMs >= 0 && run(step,false,true).responseMs <= 30);
  compare("stationary",stable); compare("moving",moving); compare("front_noise",noise);
  compare("step_0_to_60",step,true); compare("wrap_359_0",wrap);
  compare("loss_reacquire",reacquire); compare("irregular",irregular);
  ChasePD pd; pd.update(359,100,10); pd.update(0,100,40); assert(std::fabs(pd.dRaw)<3);
  pd.update(90,100,70); assert(std::fabs(pd.d)<=PdConfig::DLimit); pd.reset(); pd.update(270,100,100); assert(pd.d==0);
  pd.update(0,100,130); robot::Camera cam; cam.flags=robot::Ready; cam.yellow={90,90,0};
  auto goal=trialDecision(true,false,true,cam,true,false,false,true,pd.pwm,RunMode::GoalYellow,0,0);
  assert(goal.state==State::Chase && goal.pwm[0]<goal.pwm[2]);
  ChasePD oldGoal, newGoal; oldGoal.legacy=true;
  oldGoal.update(10,100,130); newGoal.update(10,100,130);
  auto oldOut=trialDecision(true,false,true,cam,true,false,false,true,oldGoal.pwm,RunMode::GoalYellow,10,0);
  auto newOut=trialDecision(true,false,true,cam,true,false,false,true,newGoal.pwm,RunMode::GoalYellow,10,0);
  assert(oldOut.state==State::Chase && newOut.state==State::Chase);
  std::cout<<"goal_10deg old/new PWM="<<oldOut.pwm[0]<<","<<oldOut.pwm[2]<<"/"<<newOut.pwm[0]<<","<<newOut.pwm[2]<<"\n";
  std::cout<<"goal correction and derivative reset: OK\n";
}
