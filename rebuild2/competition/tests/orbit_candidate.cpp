#include <cassert>
#include <cstring>
#include <iostream>
#include "../main/orbit_candidate.h"
using namespace robot;
Decision chase() { Decision d; d.state=State::Chase; d.reason=StopReason::None; for (auto &p:d.pwm) p=175; return d; }
Camera goals(int own=0, int attack=90) { Camera c; c.flags=Ready|TargetBlue; c.frameMs=10; c.yellow={uint16_t(own),90,0}; c.blue={uint16_t(attack),90,0}; return c; }
int main() {
  Camera c=goals(); OrbitCandidate o; Decision d=chase();
  assert(o.update(d,c,0,0,100)); assert(d.pwm[0]==-140 && d.pwm[1]==140 && d.pwm[2]==-140 && d.pwm[3]==140);
  c.blue.angle=270; d=chase(); assert(o.update(d,c,0,0,200)); assert(d.pwm[0]==-140); // held direction
  d=chase(); assert(!o.update(d,c,0,0,700)); assert(d.pwm[0]==175);
  d=chase(); assert(!o.update(d,c,0,0,2699)); assert(o.update(d,c,0,0,2700)); assert(d.pwm[0]==140);
  c.blue.angle=0; d=chase(); assert(o.update(d,c,0,0,2701)); assert(d.pwm[0]==175);
  o=OrbitCandidate{}; c=goals(25,90); d=chase(); assert(o.update(d,c,0,10,0));
  o=OrbitCandidate{}; c=goals(26,90); d=chase(); assert(!o.update(d,c,0,10,0));
  o=OrbitCandidate{}; c=goals(); d=chase(); assert(o.update(d,c,0,0,0));
  c.flags|=LineHazard; d=chase(); d.state=State::Escape; d.pwm[0]=-175; assert(!o.update(d,c,0,0,1)); assert(d.pwm[0]==-175);
  c.flags=Ready|TargetBlue; d=chase(); assert(!o.update(d,c,0,0,2));
  o=OrbitCandidate{}; d=chase(); assert(o.update(d,c,0,0,0)); d=Decision{}; d.reason=StopReason::Manual; assert(!o.update(d,c,0,0,1)); assert(d.pwm[0]==0);
  o=OrbitCandidate{}; d=chase(); assert(o.update(d,c,0,0,0)); d=Decision{}; d.reason=StopReason::BallLost; assert(!o.update(d,c,0,0,1));
  o=OrbitCandidate{}; d=chase(); assert(o.update(d,c,0,0,0)); c.blue.age=29; d=chase(); assert(!o.update(d,c,0,0,1)); assert(d.pwm[0]==175);
  o=OrbitCandidate{}; c=goals(); c.flags=Ready; c.blue={0,90,0}; c.yellow={270,90,0}; d=chase(); assert(o.update(d,c,0,0,0)); assert(d.pwm[0]==140);
  std::cout << "orbit candidate simulation OK\n";
}
