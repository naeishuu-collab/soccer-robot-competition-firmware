#include <cassert>
#include <iostream>
#include "../main/control.h"

#include "old_path.h"

struct Case {
  const char* name;
  robot::Pwm pwm;
  uint16_t mask;
  bool fresh;
  bool physicallyAllowed;  // Expected for the synthetic hazard placement only.
};

int main() {
  constexpr SweepConfig model{8, 9, 14, 28, 1, 35};
  constexpr uint16_t front = 1, right = 1 << 4, back = 1 << 8, left = 1 << 12;
  const Case cases[] = {
    {"fully observed straight", {175,175,175,175}, 0xffff, true, true},
    {"front line ahead", {175,175,175,175}, uint16_t(~front), true, false},
    {"rear line while moving ahead", {175,175,175,175}, uint16_t(~back), true, true},
    {"right line near swept flank", {175,175,175,175}, uint16_t(~right), true, false},
    {"left line near swept flank", {175,175,175,175}, uint16_t(~left), true, false},
    {"rear line during shallow curve", {175,175,210,210}, uint16_t(~back), true, true},
    {"rear line during hard curve", {-140,-140,210,210}, uint16_t(~back), true, false},
    {"rotation with line on right", {175,175,-175,-175}, uint16_t(~right), true, false},
    {"rotation in fully observed space", {175,175,-175,-175}, 0xffff, true, true},
    {"front and rear lines", {175,175,210,210}, uint16_t(~(front | back)), true, false},
    {"front unobserved", {175,175,210,210}, uint16_t(~front), true, false},
    {"rear unobserved during curve", {175,175,210,210}, uint16_t(~back), true, true},
    {"rear safe, camera stale", {175,175,210,210}, 0xffff, false, false},
    {"backward motion and front line", {-175,-175,-175,-175}, uint16_t(~front), true, true},
    {"right translation in clear space", {-160,160,-160,160}, 0xffff, true, true},
    {"forward right diagonal facing line", {0,160,0,160}, uint16_t(~(1 << 2)), true, false},
    {"stationary", {0,0,0,0}, 0, true, true},
  };
  unsigned oldStops = 0, newStops = 0, oldUnsafe = 0, newUnsafe = 0;
  unsigned oldNeedless = 0, newNeedless = 0;
  for (const auto& row : cases) {
    const Sweep sweep = candidateRequired(row.pwm, model);
    const bool oldOk = row.fresh && zipPathClear(row.mask, row.pwm);
    const bool newOk = row.fresh && sweep.withinRange && !(sweep.required & ~row.mask);
    oldStops += !oldOk;
    newStops += !newOk;
    oldUnsafe += oldOk && !row.physicallyAllowed;
    newUnsafe += newOk && !row.physicallyAllowed;
    oldNeedless += !oldOk && row.physicallyAllowed;
    newNeedless += !newOk && row.physicallyAllowed;
    std::cout << row.name << ": old=" << oldOk << " candidate=" << newOk
              << " expected=" << row.physicallyAllowed << " required=0x" << std::hex
              << sweep.required << std::dec << '\n';
  }
  assert(newUnsafe == 0 && newNeedless < oldNeedless);
  unsigned legacySameSignStops = 0, defaultSameSignStops = 0;
  for (const auto& row : cases) {
    if (!row.fresh || !row.physicallyAllowed || row.pwm[0] <= 0 || row.pwm[2] <= 0) continue;
    if (row.pwm[0] != row.pwm[1] || row.pwm[2] != row.pwm[3]) continue;
    robot::Camera cam;
    cam.flags = robot::Ready;
    cam.safe = row.mask;
    legacySameSignStops += !zipPathClear(row.mask, row.pwm);
    const auto decision = decide(false, true, cam, true, false, false, true, row.pwm, 20);
    defaultSameSignStops += decision.state == State::Stop;
    assert(decision.alternate == 0 && !decision.blocked);
  }
  assert(legacySameSignStops == 2 && defaultSameSignStops == 0);
  const robot::Pwm curve{175,175,210,210}, hard{-140,-140,210,210};
  robot::Camera cam;
  cam.flags = robot::Ready;
  cam.safe = uint16_t(~back);
  const auto commissioned = decide(false, true, cam, true, false, false, true, curve, 20, model);
  assert(commissioned.path == PathMode::Sweep && commissioned.pwm[0] == curve[0]);
  const auto softened = decide(false, true, cam, true, false, false, true, hard, 20, model);
  assert(softened.path == PathMode::ReducedTurn && softened.pwm[0] > 0 &&
         !(softened.required & ~cam.safe));
  assert(decide(false, true, cam, true, false, false, true, hard, 20).state == State::Stop);
  assert(decide(false, false, cam, true, false, false, true, curve, 20, model).state == State::Stop);
  cam.safe = 0xffff;
  assert(decide(false, true, cam, true, false, false, true, hard, 20, model, true).alternate == 160);
  cam.safe = uint16_t(~front);
  assert(decide(false, true, cam, true, false, false, true, curve, 20).state == State::Stop);
  cam.flags = robot::Ready | robot::Escape;
  cam.escape = robot::Rear;
  assert(decide(false, true, cam, false, false, true, false, curve, 20, model, true).path == PathMode::Escape);
  std::cout << "Integrated default, eligible forward cases: unnecessary stops 2 -> 0\n";
  assert(!candidateRequired(cases[0].pwm, {8, 9, 14, 28, 1, 20}).withinRange);
  assert(!candidateRequired(cases[0].pwm, {}).withinRange);
  std::cout << "Synthetic comparison: total stops " << oldStops << " -> " << newStops
            << ", unnecessary " << oldNeedless << " -> " << newNeedless
            << ", unsafe permissions " << oldUnsafe << " -> " << newUnsafe << '\n';
}
