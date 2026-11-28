#include <cassert>
#include <iostream>
#include "../main/main.ino"
#include "../motor/motor_output.h"
using namespace robot;
void send(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c), 1); }
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
int main() {
  Camera c; c.flags=Ready; c.version=3; Pwm p={150,150,190,190};
  auto d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Chase && d.pwm[0]==150);
  d=decide(false,true,c,true,false,false,false,p); assert(d.reason==StopReason::BallLost && d.pwm[0]==0);
  d=decide(false,false,c,true,false,false,true,p); assert(d.reason==StopReason::CameraTimeout && !d.pwm[0]);
  d=decide(false,true,c,false,false,false,true,p); assert(d.reason==StopReason::IrTimeout && !d.pwm[0]);
  d=decide(true,true,c,true,false,false,true,p); assert(d.reason==StopReason::Manual && !d.pwm[0]);
  c.flags=Ready|Escape|LineHazard; c.line=0; c.lineCm=255; c.escape=Rear;
  d=decide(false,true,c,true,false,false,false,p); assert(d.state==State::Escape && d.pwm[0]==-175);
  c.line=180; c.escape=Front; d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Escape && d.pwm[0]==175);
  c.escape=0; d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Escape && d.pwm[0]==175);
  c.line=Unknown; d=decide(false,true,c,true,false,false,true,p); assert(d.reason==StopReason::NoEscape && !d.pwm[0]);
  c.line=90; d=decide(false,true,c,true,false,false,true,p); assert(d.reason==StopReason::NoEscape && !d.pwm[0]);
  c.line=Unknown; c.escape=Rear; d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Escape && d.pwm[0]==-175);
  c.flags=Ready|Slow; c.escape=0; d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Chase && d.pwm[0]==140 && d.pwm[2]==140);
  c.flags=Ready; d=decide(false,true,c,true,false,false,true,p); assert(d.state==State::Chase);
  MotorOutput m; m.accept(p,0); m.update(0); assert(m.applied[0]==150); m.update(100); assert(m.watchdog && !m.applied[0]);
  Serial.connected=true; clockMs=0; clockUs=0; setup(); assert(manual && !transmitted[0]);
  haveIr=true; irStarted=true; cameraLive=haveCamera=true; camera.flags=0; send('g'); tick(4000); tick(6000);
  assert(startPending && manual && !transmitted[0]);
  camera.flags=Ready; tick(6001); assert(!startPending && !manual && decision.reason==StopReason::BallLost);
  haveBall=true; pd.update(0,100,6002); tick(6002); assert(decision.state==State::Chase);
  haveBall=false; tick(6003); assert(decision.reason==StopReason::BallLost && !decision.pwm[0]);
  send('s'); tick(6004); assert(manual && !decision.pwm[0]);
  std::cout << "rebuild2 control, startup and watchdog OK\n";
}
