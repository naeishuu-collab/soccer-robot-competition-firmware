#define RCJ_PHASE3_ORBIT 1
#include <cassert>
#include <iostream>
#include "../main/main.ino"
using namespace robot;
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
int main() {
 Serial.connected=true; setup(); haveCamera=cameraLive=haveIr=irStarted=haveBall=true; manual=false;
 camera.flags=Ready; camera.frameMs=10; camera.yellow={0,90,0}; camera.blue={90,90,0}; pd.update(0,100,4000); tick(4000);
 assert(orbit.active && decision.pwm[0]==-140 && decision.pwm[1]==140);
 camera.flags=Ready|Escape|LineHazard; camera.line=0; camera.escape=Rear; tick(4001); assert(!orbit.active && decision.state==State::Escape && decision.pwm[0]==-175);
 camera.flags=Ready; tick(4002); assert(!orbit.active && decision.pwm[0]==-140 && decision.pwm[1]==-140); // existing pivot during cooldown
 tick(6001); assert(orbit.active); camera.blue={0,90,0}; tick(6002); assert(!orbit.active && decision.pwm[0]==175 && decision.pwm[1]==175);
 tick(6003); assert(!orbit.active && decision.pwm[0]==175 && decision.pwm[1]==175);
 tick(8002); camera.blue={90,90,0}; tick(8003); assert(orbit.active); manual=true; tick(8004); assert(!orbit.active && decision.reason==StopReason::Manual && !decision.pwm[0]);
 manual=false; tick(10004); assert(orbit.active); cameraLive=false; tick(10005); assert(!orbit.active && decision.reason==StopReason::CameraTimeout && !decision.pwm[0]);
 cameraLive=true; camera.blue={90,90,0}; tick(12005); assert(orbit.active); haveIr=false; tick(12006); assert(!orbit.active && decision.reason==StopReason::IrTimeout && !decision.pwm[0]);
 haveIr=true; haveBall=true; tick(14006); assert(orbit.active); haveBall=false; tick(14007); assert(!orbit.active && decision.reason==StopReason::BallLost && !decision.pwm[0]);
 haveBall=true; tick(16007); assert(orbit.active); camera.blue={Unknown,0,255}; tick(16008); assert(!orbit.active && decision.pwm[0]==-140 && decision.pwm[1]==-140);
 std::cout << "orbit integration priority OK\n";
}
