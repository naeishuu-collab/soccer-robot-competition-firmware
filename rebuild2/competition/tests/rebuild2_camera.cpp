#include <cassert>
#include <iostream>
#include "../main/main.ino"
using namespace robot;
void frame(uint8_t seq, uint8_t flags, uint16_t bearing, uint8_t ms, uint16_t escape=0, uint16_t goal=Unknown) {
  uint8_t b[CameraSize]{}; b[0]=0xc6; b[1]=0x6c; b[2]=seq; b[3]=flags;
  put16(b+4,escape); put16(b+6,escape); put16(b+8,bearing); b[10]=255; b[11]=ms;
  put16(b+12,Unknown); b[15]=255; put16(b+16,goal); b[18]=goal == Unknown ? 0 : 90; b[19]=goal == Unknown ? 255 : 0; b[20]=3;
  seal(b,sizeof(b)); cameraSerial.inject(b,sizeof(b));
}
void tick(uint32_t now) { clockMs=now; clockUs=now*1000; irMs=pollMs=now; waiting=false; loop(); }
int main() {
 Serial.connected=true; setup(); haveIr=irStarted=true; haveBall=true; pd.update(15,100,4000);
 frame(40,0,Unknown,10); tick(1000); assert(startPending && manual);
 frame(41,Ready|Slow,Unknown,120); tick(4000); assert(!startPending && !manual);
 frame(1,Ready|Slow,Unknown,120); tick(4000); assert(decision.state==State::Chase && decision.pwm[0]<=140 && decision.pwm[2]<=140);
 frame(2,Ready|Slow|Escape|LineHazard,0,120); tick(4020); assert(decision.state==State::Escape && decision.pwm[0]==-175);
 frame(3,Ready|Escape|LineHazard,0,10); tick(4040); assert(decision.state==State::Escape);
 frame(4,Ready|Escape|LineHazard,Unknown,10); tick(4070); assert(decision.state==State::Escape && decision.pwm[0]==-175);
 frame(5,Ready|Escape|LineHazard,Unknown,10); tick(4200); assert(decision.reason==StopReason::NoEscape && !decision.pwm[0]);
 frame(6,Ready,Unknown,10); tick(4210); assert(decision.state==State::Chase);
 tick(4410); assert(decision.reason==StopReason::CameraTimeout && !decision.pwm[0]);
 frame(7,Ready,Unknown,10); tick(4420); assert(decision.state==State::Chase);
 frame(8,Ready|Slow,Unknown,199); tick(4430); assert(decision.state==State::Chase);
 tick(4431); assert(decision.reason==StopReason::CameraTimeout);
 frame(9,Ready|Slow,Unknown,255); tick(4440); assert(decision.reason==StopReason::CameraTimeout);
 frame(10,Ready,Unknown,10); tick(4450); assert(decision.state==State::Chase);
 pd.update(0,100,4451); haveBall=true;
 frame(12,Ready,Unknown,10,0,60); tick(4451); assert(decision.state==State::Chase && decision.pwm[0]==155 && decision.pwm[2]==195);
 haveBall=false; tick(4452); assert(decision.reason==StopReason::BallLost && !decision.pwm[0]);
 haveBall=true; pd.update(0,100,4453); tick(4453); assert(decision.state==State::Chase && decision.pwm[0]==155);
 char stop='s'; Serial.inject(reinterpret_cast<const uint8_t*>(&stop),1); tick(4460); assert(manual && decision.reason==StopReason::Manual);
 frame(11,Ready,Unknown,10); tick(4470); assert(manual && decision.reason==StopReason::Manual);
 std::cout << "freshness, slow, bearing memory, reconnect and manual stop OK\n";
}
