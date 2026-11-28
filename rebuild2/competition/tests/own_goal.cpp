#include <cassert>
#include <cstring>
#include <iostream>
#include "../main/main.ino"
using namespace robot;
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
void send(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c),1); }
bool equal(const Decision& a,const Decision& b) { return a.state==b.state && a.reason==b.reason && !memcmp(a.pwm,b.pwm,sizeof(a.pwm)); }
int main() {
 Serial.connected=true; setup(); assert(targetBlue); haveCamera=cameraLive=haveIr=irStarted=haveBall=true;
 camera.flags=Ready; camera.frameMs=10; pd.update(0,100,4000); manual=false;
 camera.yellow={0,90,0}; camera.blue={60,90,0}; tick(4000);
 assert(decision.state==State::Chase && decision.pwm[0]==-140 && decision.pwm[2]==140);
 camera.yellow={1,90,0}; camera.blue={300,90,0}; tick(4001); assert(decision.pwm[0]==-140 && decision.pwm[2]==140);
 camera.yellow={180,90,0}; tick(4002); assert(decision.pwm[0]==195 && decision.pwm[2]==155);
 camera.blue={Unknown,0,255}; pd.update(30,100,4003); camera.yellow={0,90,0}; tick(4003);
 auto baseline=decide(false,true,camera,true,false,false,true,pd.pwm,pd.error);
 assert(equal(decision,baseline));
 camera.yellow={Unknown,0,255}; tick(4004); baseline=decide(false,true,camera,true,false,false,true,pd.pwm,pd.error); assert(equal(decision,baseline));
 send('y'); tick(4005); assert(!targetBlue); camera.yellow={60,90,0}; camera.blue={Unknown,0,255}; pd.update(0,100,4006); tick(4006); assert(decision.pwm[0]==155 && decision.pwm[2]==195); camera.blue={0,90,0}; tick(4006); assert(decision.pwm[0]==-140 && decision.pwm[2]==140);
 send('b'); tick(4007); assert(targetBlue); camera.yellow={0,69,0}; camera.blue={Unknown,0,255}; tick(4008); baseline=decide(false,true,camera,true,false,false,true,pd.pwm,pd.error); assert(equal(decision,baseline));
 camera.yellow={0,90,30}; tick(4009); baseline=decide(false,true,camera,true,false,false,true,pd.pwm,pd.error); assert(equal(decision,baseline));
 camera.yellow={0,90,0}; camera.flags=Ready|Escape|LineHazard; camera.line=0; camera.escape=Rear; tick(4010); assert(decision.state==State::Escape && decision.pwm[0]==-175);
 camera.flags=Ready; cameraLive=false; tick(4011); assert(decision.reason==StopReason::CameraTimeout && decision.pwm[0]==0);
 std::cout << "target selection, own goal, hysteresis, line and stop OK\n";
}
