#define RCJ_SERIAL_TRIAL 0
#define RCJ_WHEELS_LIFTED_DIAGNOSTICS 1
#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include "../main/main.ino"
using namespace robot;
void send(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c), 1); }
void tick(uint32_t t) { clockMs=t; clockUs=t*1000; irMs=cameraMs=pollMs=t; waiting=false; loop(); }
void prepare(char pattern, uint32_t t) { send('s'); tick(t); send('M'); tick(t+1); send(pattern); tick(t+2); send('g'); tick(t+3); send('R'); tick(t+4); }
int main() {
 Serial.connected=true; setup(); send('y'); tick(1); assert(!targetBlue); send('b'); tick(2); assert(targetBlue); haveCamera=cameraLive=haveIr=irStarted=true; camera.flags=Ready; camera.frameMs=10;
 const int expected[6][4]={{-120,120,-120,120},{120,-120,120,-120},{120,120,120,120},{-120,-120,-120,-120},{-120,-120,120,120},{120,120,-120,-120}};
 for (int i=0;i<6;i++) {
  uint32_t t=4000+i*1000; prepare(char('5'+i%5),t); // 0 selects pattern 10
  if (i==5) { send('s'); tick(t+5); send('M'); tick(t+6); send('0'); tick(t+7); send('g'); tick(t+8); send('R'); tick(t+9); }
  assert(benchArmed && decision.state==State::Chase);
  for (int j=0;j<4;j++) assert(decision.pwm[j]==expected[i][j]);
  send('R'); tick(t+50); assert(benchMs==(i==5?t+9:t+4));
  tick(t+200); assert(manual && !benchArmed && decision.state==State::Stop && !decision.pwm[0]);
  send('R'); tick(t+201); assert(!benchArmed && !decision.pwm[0]);
 }
 prepare('1',11000); assert(decision.pwm[0]==120 && !decision.pwm[1]); send('s'); tick(11005); assert(manual && !decision.pwm[0]);
 send('M'); tick(12000); send('2'); tick(12001); send('-'); tick(12002); send('g'); tick(12003); send('R'); tick(12004); assert(decision.pwm[1]==-120);
 Serial.connected=false; tick(12005); assert(manual && !decision.pwm[1]); Serial.connected=true;
 send('s'); tick(13000); send('M'); tick(13001); send('3'); tick(13002); send('g'); tick(13003); send('R'); tick(13004); camera.flags=Ready|LineHazard; tick(13005); assert(manual && !decision.pwm[2]);
 camera.flags=Ready; prepare('4',14000); haveIr=false; tick(14005); assert(manual && !decision.pwm[3]);
 haveIr=true; prepare('4',15000); cameraLive=false; tick(15005); assert(manual && !decision.pwm[3]);
 cameraLive=true; send('s'); tick(16000); send('M'); tick(16001); send('1'); tick(16002); send('g'); tick(16003); tick(18003); assert(manual && !benchArmed);
 requestedReport=true; for (int i=0;i<1500;i++) tick(18004);
 const std::string log(Serial.tx.begin(),Serial.tx.end());
 assert(log.find("BENCH_LAST_PWM=")!=std::string::npos && log.find("BENCH_ELAPSED_MS=")!=std::string::npos);
 std::cout << "lifted bench pulses, limits and interlocks OK\n";
}
