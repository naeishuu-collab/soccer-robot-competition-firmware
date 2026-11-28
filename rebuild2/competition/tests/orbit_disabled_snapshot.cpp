#include <iostream>
#include "../main/main.ino"
using namespace robot;
int main() {
 Serial.connected=true; setup(); haveCamera=cameraLive=haveIr=irStarted=haveBall=true; manual=false; camera.flags=Ready; camera.frameMs=10;
 for (int i=0;i<180;i++) {
  uint32_t t=4000+i; clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false;
  camera.yellow={uint16_t((i*7)%360),uint8_t(i%5?90:20),uint8_t(i%4)}; camera.blue={uint16_t((i*13)%360),90,0};
  pd.update(uint16_t((i*11)%360),100,t); targetBlue=(i%2)==0; loop();
  std::cout<<int(decision.state)<<':'<<int(decision.reason)<<':'<<decision.pwm[0]<<','<<decision.pwm[1]<<','<<decision.pwm[2]<<','<<decision.pwm[3]<<'\n';
 }
}
