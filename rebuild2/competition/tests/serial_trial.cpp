#define RCJ_START_WAIT_MS 0 // Existing controller regression; start wait is tested separately.
#include <cassert>
#include <iostream>
#include "../main/main.ino"
using namespace robot;
void inject(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c), 1); }
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=pollMs=t; waiting=false; loop(); }
void ball(uint32_t t) { haveBall=true; pd.update(0,100,t); }
int main() {
  Camera cam; cam.flags=Ready | Escape | LineHazard; cam.yellow={90,90,0}; cam.blue={270,90,0};
  Pwm pwm{175,175,175,175};
  auto d=trialDecision(true,false,false,cam,true,false,false,true,pwm,RunMode::PdChase,0,0);
  assert(d.state==State::Chase && d.pwm[0]==175 && d.pwm[2]==175);
  d=trialDecision(true,false,true,cam,true,false,false,true,pwm,RunMode::GoalYellow,0,0);
  assert(d.pwm[0]==135 && d.pwm[2]==210);
  d=trialDecision(true,false,true,cam,true,false,false,true,pwm,RunMode::GoalBlue,0,0);
  assert(d.pwm[0]==210 && d.pwm[2]==135);
  d=trialDecision(true,false,true,cam,true,false,false,true,pwm,RunMode::GoalYellow,0,300);
  assert(d.pwm[0]==175 && d.pwm[2]==175);
  assert(trialDecision(true,false,false,cam,true,false,false,true,pwm,RunMode::GoalBlue,0,0).reason==StopReason::CameraTimeout);
  assert(trialDecision(true,false,false,cam,false,false,false,true,pwm,RunMode::PdChase,0,0).reason==StopReason::IrTimeout);
  assert(trialDecision(true,false,false,cam,true,false,true,true,pwm,RunMode::PdChase,0,0).reason==StopReason::IrNoReply);
  assert(trialDecision(true,false,false,cam,true,false,false,false,pwm,RunMode::PdChase,0,0).reason==StopReason::BallLost);
  Serial.connected=true; clockMs=0; clockUs=0; setup();
  haveIr=true; irStarted=true; camera=cam;
  tick(4000); assert(manual && decision.state==State::Stop);
  inject('p'); tick(4001); assert(manual && runMode==RunMode::PdChase && !trialArmed);
  ball(4002); inject('g'); tick(4002);
  assert(!manual && trialArmed && decision.state==State::Chase && decision.pwm[0]==175);
  tick(8000); assert(trialArmed && decision.state==State::Chase);
  inject('g'); tick(9001); assert(trialArmed && decision.state==State::Chase);
  tick(9002); assert(trialArmed && decision.state==State::Chase);
  tick(9004); assert(trialArmed && decision.state==State::Chase);
  haveBall=false; tick(9005); assert(!manual && trialArmed && decision.reason==StopReason::BallLost && decision.state==State::Stop);
  ball(9006); tick(9006); assert(!manual && decision.state==State::Chase);
  irInvalid=true; tick(9008); assert(manual && !trialArmed && decision.state==State::Stop);
  irInvalid=false; ball(9009); tick(9009); assert(manual);
  inject('g'); tick(9010); assert(trialArmed);
  Serial.connected=false; tick(9011); assert(manual && !trialArmed && decision.state==State::Stop);
  Serial.connected=true; tick(9012); assert(manual);
  inject('y'); tick(9013); assert(manual && runMode==RunMode::GoalYellow);
  ball(9014); inject('g'); tick(9014); assert(manual && !trialArmed); // camera missing
  haveCamera=cameraLive=true; cameraMs=9015; ball(9015); inject('g'); tick(9015);
  assert(trialArmed && decision.state==State::Chase && decision.pwm[0]==135);
  tick(9215); assert(manual && !trialArmed && decision.state==State::Stop); // camera stale
  cameraLive=true; cameraMs=9216; ball(9216); inject('g'); tick(9216); assert(trialArmed);
  inject('s'); tick(9217); assert(manual && !trialArmed && decision.state==State::Stop);
  inject('b'); tick(9218); assert(manual && runMode==RunMode::GoalBlue);
  inject('c'); tick(9219); assert(manual && runMode==RunMode::ManagedCompetition);
  std::cout << "Serial trial: single g, no time limit, loss latches, camera and goals OK\n";
}
