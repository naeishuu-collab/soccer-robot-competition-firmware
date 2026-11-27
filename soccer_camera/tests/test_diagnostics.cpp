#include <Diagnostics.h>
#include <assert.h>
#include <stdio.h>

int main() {
  robotdiag::EventRing<3> events;
  assert(events.size() == 0);
  assert(robotdiag::name(robotdiag::StopReason::CAMERA_TIMEOUT)[0] == 'C');
  events.push(10, robotdiag::EventKind::STATE, 1, 2);
  events.push(20, robotdiag::EventKind::STOP, 3);
  events.push(30, robotdiag::EventKind::PWM, 4, -175);
  assert(events.size() == 3 && events.overwritten() == 0);
  events.push(40, robotdiag::EventKind::COMM_ERROR, 5);
  assert(events.size() == 3 && events.overwritten() == 1);
  assert(events.oldest(0).atMs == 20);
  assert(events.oldest(2).code == 5);
  puts("diagnostic reason names and bounded event history passed");
}
