# Control algorithm and comparison boundary

## Reference chase

`ZipChasePD` in `soccer_camera/main/zip_chase_pd.h` is the reference implementation. The fixed 10,000-input
comparison in `test_zip_chase_pd.cpp` independently transcribes the ZIP equations and checks four signed
wheel PWM outputs. The reference uses P=`2.0 * error`, D=`0.2 * angular_rate`, nominal output 175, non-zero
floor 140 and maximum magnitude 210. The algorithm and its output limits were preserved.

## Isolated filtered candidate

`FilteredZipChasePD` is selected only by `ENABLE_EXPERIMENTAL_PD_FILTER=1`, which defaults to 0. It uses the
same P/D gains and PWM range but low-pass filters angular rate, wraps the 359°→1° boundary, and suppresses
the derivative for sample intervals outside 5–100 ms. A missing ball resets both controller states and
outputs zero. It is a candidate for host comparison, not a competition default; its timing and steering
response have not been measured on the robot.

| Case | ZIP reference | Filtered candidate | Verification |
|---|---|---|---|
| Ordinary input sequence | Preserved formula | Same nominal P/D and PWM envelope, filtered D | Reference path: 10,000 exact comparisons; candidate: boundary/gap/reset unit cases |
| 359° then 1° | Wrapped shortest difference | Wrapped shortest difference | Host test |
| Invalid/lost ball | Reset, four zeros | Reset including filter, four zeros | Host test |
| Very long scheduling gap | Raw elapsed-time D | D suppressed for that sample | Host test; hardware period distribution pending |

The MAIN `chaseLoop()` diagnostic labels which controller was compiled. `ROBOT_TEST_MAX_PWM` remains 210;
there is no added global cap. MOTOR validates the protocol's signed PWM envelope and retains its reverse
pause, ramp, and communication-timeout stop.

Search, orbit and attack branches in the expanded `brain::Brain` are separately compiled with the matching
`ENABLE_BALL_SEARCH`, `ENABLE_ORBIT`, and `ENABLE_ATTACK` flags. They default to disabled. With orbit off,
the expanded controller uses direct ball approach/alignment; with attack off, it does not enter ATTACK. A
missing ball stops instead of sweeping the field when search is disabled. These defaults reduce exposure
to unvalidated motion; they are not performance claims.

## Safety and tactics

The existing `brain::Brain::step()` is the expanded route's single decision point. It checks camera and IR
freshness, line-mask availability, line escape, ball validity, goal-relative tactics, and own-goal vetoes
before producing one `Output`. `upgradeLoop()` is responsible for calling it and sending one command.
That expanded route and its wheel model are still PENDING_HARDWARE. Do not infer that its search/orbit/
attack logic has been validated by host tests alone.
