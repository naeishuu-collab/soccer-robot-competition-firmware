# Software verification record

State labels: `VERIFIED` means the named software check ran successfully; it does not mean hardware was
tested. Hardware-only items stay `PENDING_HARDWARE`.

## Phase status

| Phase | State | Current result |
|---|---|---|
| 0 | VERIFIED | Baseline SHA, source routes, protocol and restore point recorded |
| 1 | PARTIAL | Per-board diagnostics expanded; OpenMV UART counters/CSV added; some shared structured reporting and MAIN/MOTOR feedback remain absent |
| 2 | PARTIAL | Build dependency checks, feature gates and host ModeGate exist; USB mode router and full runtime test suite are absent |
| 3 | PARTIAL | ZIP reference unchanged; filtered PD is optional and default off; physical comparison pending |
| 4 | PARTIAL | Existing fast/reference vision pipeline and timing reports covered by host tests; no OpenMV device measurements |
| 5 | PARTIAL | All-direction line guard and host cases exist; chassis/field stopping geometry unverified |
| 6 | PARTIAL | Existing search state is compile-gated default off; no field movement verification |
| 7 | PARTIAL | Existing yellow/blue recognition tested with host fakes; yellow issue needs recorded camera images |
| 8 | PARTIAL | Existing orbit/attack code is gated default off; physical scoring/heading behavior unverified |
| 9 | PARTIAL | Existing frame validation/reverse/watchdog code and host tests; bus/wheel behavior unverified |
| 10 | PARTIAL | Expanded Brain returns one decision and loop sends one route output; runtime mode router and hardware arbitration validation absent |

| Item | State | Evidence / limit |
|---|---|---|
| Source baseline | VERIFIED | `44cd80b3fd2a5137721e3f97bdd7603d21474557` |
| Arduino compile | VERIFIED | MAIN RP2354B, IR RP2354A, LINE compile-only RP2354B, MOTOR RP2354A; no upload |
| Host C++ | VERIFIED | 21 executables; tactical regression suite built with search/orbit/attack explicitly enabled; default disabled profile checked separately; ZIP PD has 10,000 exact reference comparisons; core 65 cases / 5,165 checks; integration 14 scenarios / 107 checks |
| Optional filtered-PD firmware build | VERIFIED | MAIN compiles with `ENABLE_EXPERIMENTAL_PD_FILTER=1`; candidate still requires hardware response comparison |
| Python unit tests | VERIFIED | 110 tests, including line/goal pipeline, UART writer fault injection and CSV schema |
| OpenMV Python syntax | VERIFIED | `compileall` on OpenMV and test sources; host Python is not OpenMV runtime |
| Protocol round trips and fault cases | VERIFIED in host model | Existing tests cover parser, CRC, timeouts and invalid inputs; no physical bus timing evidence |
| Synthetic vision | VERIFIED in host model | Existing tests use generated/fake blobs; not recorded camera frames |
| Build identity injection | VERIFIED | compile script adds current short SHA; OpenMV SHA is manual |
| Physical sensors, wheel outputs, safety distances, FPS | PENDING_HARDWARE | No robot/camera hardware operated |
| Protocol version auto-negotiation | NOT_IMPLEMENTED | Formats are paired by compile/config settings, not negotiated |
| Runtime USB test-mode router | PARTIAL | Enum/gate is host tested; no MAIN serial mode-switch command; per-board test actions remain separate |
| Feature profiles | PARTIAL | Default profile and full-tactics host profile are tested. Search/orbit/attack and filtered PD compile gates exist; no complete flag matrix or USB toggles |
| MAIN/MOTOR state correlation | PARTIAL | Two USB consoles only; no MOTOR reply protocol |
| Competition acceptance | NO-GO | Active expanded path has not been verified on hardware |

See `PHASE1_STATUS_2026-11-27_JA.md` for the preceding baseline and current limitations. Re-run all test
commands after subsequent source changes; this record only applies to the listed tested commit.
