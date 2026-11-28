#include <cassert>
#include <iostream>
#include "../main/main.ino"
using namespace robot;
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
void send(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c),1); }
int main() {
  Serial.connected=false; setup(); assert(manual && startPending);
  haveIr=irStarted=true; haveCamera=cameraLive=true; camera.flags=0;
  tick(1000); tick(3000); assert(startPending && manual && !decision.pwm[0]);
  camera.flags=Ready; haveBall=true; pd.update(0,100,3001); tick(3001);
  assert(!manual && !startPending && decision.state==State::Chase);
  Serial.connected=true; send('s'); tick(3002); assert(manual && !startPending && !decision.pwm[0]);
  tick(6000); assert(manual && !startPending);
  send('g'); tick(6001); assert(manual && startPending);
  tick(8000); assert(manual && startPending);
  haveBall=true; pd.update(0,100,8001); tick(8001); assert(!manual && !startPending && decision.state==State::Chase);
  send('s'); tick(8002); send('g'); tick(8003); assert(startPending);
  Serial.connected=false; tick(8004); assert(manual && !startPending);
  setup(); assert(startPending && manual);
  Serial.connected=true; send('s'); tick(8100); assert(manual && !startPending);
  tick(12000); assert(manual && !startPending && !decision.pwm[0]);
  std::cout << "auto start, camera readiness, manual stop and delayed rearm OK\n";
}
