#include <cassert>
#include <iostream>
#include <string>
#include "../main/main.ino"
using namespace robot;
void send(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c),1); }
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
void ball(uint32_t t) { haveIr=haveBall=true; irInvalid=missedIr=false; pd.update(0,100,t); }
void flush(uint32_t t) { for(int i=0;i<1500;i++) tick(t); }
int main() {
  Serial.connected=true; clockMs=0; clockUs=0; setup();
  haveIr=haveBall=true; irStarted=true; ball(4000); tick(4000);
  send('p'); tick(4001); ball(4002); send('g'); tick(4002);
  assert(startPending && manual && decision.state==State::Stop && transmitted[0]==0);
  // The normal bus poll continues while waiting.
  clockMs=4033; clockUs=4033000; irMs=cameraMs=4033; pollMs=4002; waiting=false; loop();
  if (!waiting) { delayMicroseconds(BusGuardUs + 1); loop(); }
  assert(waiting && Serial1.tx.size()>=sizeof(Poll));
  assert(!memcmp(Serial1.tx.data()+Serial1.tx.size()-sizeof(Poll),Poll,sizeof(Poll)));
  send('g'); tick(5000); assert(startPending && startRequestMs==4002);
  tick(6001); assert(startPending && manual && decision.pwm[0]==0);
  tick(6002); assert(!startPending && !manual && trialArmed && decision.state==State::Chase);
  send('s'); tick(6003); assert(manual && decision.state==State::Stop);
  ball(6004); send('g'); tick(6004); assert(startPending);
  send('s'); tick(6100); assert(!startPending && manual);
  ball(6101); send('g'); tick(6101); assert(startPending);
  send('b'); tick(6102); assert(!startPending && manual && runMode==RunMode::GoalBlue);
  haveBall=false; irStrength=0; irAngle=Unknown; pd.reset();
  camera.flags=Ready; camera.version=3; camera.safe=0xffff; haveCamera=cameraLive=true;
  send('g'); tick(6103); assert(startPending && manual);
  tick(8103); assert(!startPending && !manual && trialArmed && decision.reason==StopReason::BallLost);
  ball(8104); tick(8104); assert(!manual && trialArmed && decision.state==State::Chase);
  send('s'); tick(8105); ball(8106); send('y'); tick(8106); haveBall=false; pd.reset(); send('g'); tick(8107);
  tick(10107); assert(!manual && trialArmed && decision.reason==StopReason::BallLost);
  irInvalid=true; tick(10108); assert(manual && !trialArmed); irInvalid=false;
  ball(10109); send('c'); tick(10109); ball(10110); send('g'); tick(10110);
  tick(12110); assert(!startPending && !manual && runMode==RunMode::ManagedCompetition && decision.state==State::Chase);
  send('s'); tick(10105); ball(10106); send('y'); tick(10106); ball(10107); send('g'); tick(10107);
  irInvalid=true; tick(12107); assert(!startPending && manual); irInvalid=false;
  ball(12108); send('c'); tick(12108); ball(12109); send('g'); tick(12109);
  tick(14109); assert(!startPending && !manual && runMode==RunMode::ManagedCompetition && decision.state==State::Chase);
  send('s'); tick(14110); ball(14111); send('g'); tick(14111); Serial.connected=false; tick(14112);
  assert(!startPending && manual); Serial.connected=true;
  flush(14113);
  const std::string log(Serial.tx.begin(),Serial.tx.end());
  assert(log.find("START_PENDING: 2000ms\n")!=std::string::npos);
  assert(log.find("STARTED\n")!=std::string::npos);
  assert(log.find("START_CANCELLED")!=std::string::npos);
  std::cout<<"Start wait: zero PWM, poll, repeat g, cancel, all modes, faults and messages OK\n";
}

