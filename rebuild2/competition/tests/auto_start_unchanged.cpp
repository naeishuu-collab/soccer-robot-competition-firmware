#include <cassert>
#include <iostream>
#define RCJ_SERIAL_TRIAL 0
#include "../main/main.ino"
using namespace robot;
int main() {
  clockMs=1000; clockUs=1000000; Serial.connected=false; setup();
  assert(!manual);
  haveCamera=cameraLive=haveIr=haveBall=true; camera.flags=Ready; camera.version=2; camera.safe=Front;
  irStarted=true; cameraMs=irMs=pollMs=clockMs; pd.update(0,100,clockMs);
  delayMicroseconds(BusGuardUs+1); loop(); assert(decision.reason==StopReason::Startup);
  clockMs=4000; clockUs=4000000; cameraMs=irMs=pollMs=clockMs; waiting=false; loop();
  assert(!manual && decision.state==State::Chase);
  std::cout<<"Existing auto start without g unchanged OK\n";
}
