# AV Safety Supervisor — Autonomous Vehicle Safety Supervisor on QNX

QNX eHACK 2026 · Problem statement #17 · Team NEXGEN INNOVATORS

An independent, highest-priority QNX program that checks every command of the autonomous
navigation software against physics and its own sensors, corrects or blocks unsafe commands, and
brings the vehicle to a safe state within a guaranteed deadline. A separate STM32 safety chip
stops the vehicle if the supervisor itself fails.

This is **Stage 0**: the complete QNX software running against a simulated vehicle, so it can be
developed and demonstrated **without hardware**. The supervisor code does not change when the real
STM32 + robot car replace `vehicle_simulator`.

## The programs

| Program | Priority | What it does |
|---|---|---|
| `safety_supervisor` | 50 (highest) | Checks every navigation command: PASS / CLAMP (made safer) / REJECT (blocked). Only it may drive the car |
| `vehicle_simulator` | 40 / 50 | Simulated car + wheel sensors + ultrasonic + IMU + camera + STM32 safety chip |
| `recovery_manager` | 45 | Smooth controlled stop; restarts navigation after a crash or freeze |
| `navigation` | 10 (low) | Stand-in for the AI driver: sends speed + steering 20× per second; can be told to misbehave |
| `event_logger` | 8 | Writes the easy-to-read safety log, CSV files and black-box incident files |
| `operator_cli` | 5 | Operator console: start, status, inject faults, acknowledge |
| `cpu_stress` | 9 | Loads the CPU to prove the supervisor keeps its timing |

```
 navigation (prio 10) ──MsgSend(command)──► SAFETY SUPERVISOR (prio 50, fixed-priority channel)
   the untrusted AI   ◄──MsgReply(PASS / CLAMP / REJECT)──┘   │
   heartbeat pulse ──────────────────────────────────────────►│
 vehicle_simulator ──sensor values (100 Hz) + camera (33 Hz)──►│
 recovery_manager ──stop commands (only accepted in MRM)──────►│
                  ◄──pulses: start stop / stop / restart nav───┤
 vehicle_simulator ◄──safe command every 10 ms (supervisor only)┤
 event_logger (prio 8) ◄──shared-memory buffer + pulse─────────┘
 operator_cli ──status / ack / inject faults──► supervisor, navigation, simulator
```

Deadlock rule: the supervisor only `MsgSend`s downwards (to the vehicle). Programs that send to
it are only notified with pulses, so no waiting cycle can form.

## Source layout

| Path | Contents |
|---|---|
| `shared/messages.h` | every message, pulse code and list (states, problems, faults) |
| `shared/settings.*` | periods, timeouts, priorities, GPIO pins, `robot` and `car` profiles, plain-English names |
| `shared/vehicle_physics.*` | turning model, stopping distance, safe steering limit |
| `shared/sensor_health.*` | detects DEAD / FROZEN / OUT-OF-RANGE / JUMPING sensors |
| `shared/qnx_helpers.*` | QNX helpers: priority, memory lock, channels, pulse timers, send with timeout |
| `shared/event_buffer.*` | lock-free shared-memory event buffer (supervisor → logger) |
| `shared/access_control.*` | who may send which message (only navigation may command, only the supervisor may drive) |
| `safety_supervisor/supervisor_main.c` | QNX message loop: receives commands, sensor values, timer ticks |
| `safety_supervisor/decision_core.*` | runs all checks every cycle and builds the decision |
| `safety_supervisor/check_sensors.c` | sensor health, real speed, 2-out-of-3 turn vote, brake check |
| `safety_supervisor/check_nav_command.c` | valid value, steering too fast, too much steering for speed, collision limit |
| `safety_supervisor/check_obstacle_camera.c` | stopping distance vs obstacle, camera vs ultrasonic, camera flicker |
| `safety_supervisor/watchdog.c` | sensor link, navigation heartbeat / freeze / crash, camera too late |
| `safety_supervisor/safety_states.*` | NORMAL → CAUTION → DEGRADED → MRM → ESTOP → SAFE → RECOVERY |
| `safety_supervisor/final_command.*` | picks the one safe command sent to the car |
| `safety_supervisor/send_output.*` | connections to vehicle, recovery manager, logger |
| `safety_supervisor/leds_buzzer.*` | red/green LED + buzzer (simulated now; Pi 5 GPIO later) |
| `safety_supervisor/timing_stats.*` | cycle time, timer delay, reaction time |
| `vehicle_simulator/` | simulated car, sensors, camera and STM32 safety chip |
| `navigation/` | navigation stand-in with fault injection |
| `recovery_manager/` | controlled stop, restarts navigation |
| `event_logger/` | safety log, CSV files, black box |
| `operator_cli/` | operator console (mandatory CLI) |
| `tools/` | `start_all.sh`, `stop_all.sh`, `cpu_stress.c` |
| `tests/unit_tests.c` | 42 tests of the decision logic (run on the PC) |

## Build

Windows (Git Bash) or Linux, with QNX SDP 8.0 installed:

```sh
source ~/qnx800/qnxsdp-env.sh      # Windows Git Bash: source /c/Users/<you>/qnx800/qnxsdp-env.sh
make                               # Raspberry Pi 5 / Pi 4  -> build/aarch64le/
make ARCH=x86_64                   # QNX x86_64 VM          -> build/x86_64/
make test                          # PC unit tests (needs gcc in PATH)
```

## Run

Copy everything in `build/x86_64/` (VM) or `build/aarch64le/` (Pi) to `/tmp/sen` on the target, then:

```sh
cd /tmp/sen && chmod +x *
./start_all.sh robot vm       # on the QNX VM   (vm = relaxed timeouts, the PC can pause the VM)
./start_all.sh robot          # on the Raspberry Pi
./start_all.sh car vm         # full-size car numbers (80 km/h)
```

Second window — the easy-to-read safety log with values:

```sh
tail -f /tmp/av_logs/safety_log.txt
```

Third window — the operator console:

```sh
/tmp/sen/operator_cli
```

Example of the safety log:

```
    1.00 s  LIVE       state NORMAL | who drives: NAV | nav asks 0.50 m/s, steer 0.0 deg | problems: -
                       speed 0.50 m/s | obstacle none | turning 0.3 deg/s | car gets 0.50 m/s, steer 0.0 deg
   12.30 s  FAULT      operator injected fault "slew"
   12.35 s  BLOCKED    nav asked 0.50 m/s, steer 30.0 deg  ->  car gets 0.50 m/s, steer 0.0 deg   why: steering changes faster than the car can steer (STEERING_TOO_FAST)
   12.35 s  REACTION   supervisor reacted to "slew" after 48.2 ms
```

Files in `/tmp/av_logs` (copy to Windows with the Momentics Target File System Navigator, open in Excel):

| File | Contents |
|---|---|
| `safety_log.txt` | the readable log above |
| `safety_events.csv` | every safety event with all values |
| `live_values.csv` | state, speed, obstacle distance, nav command, car command every 10 ms |
| `incident_001.csv` … | black box: 0.5 s before + 1 s after each emergency or blocked command |
| `supervisor.out`, `navigation.out`, `simulator.out`, `recovery.out` | console output of each program |

## Demo script (in `operator_cli`, prompt `safety>`)

| Step | Command | What happens |
|---|---|---|
| 1 | `start`, `ack`, `watch 5` | car cruises at 0.5 m/s, state NORMAL |
| 2 | `inject slew` | AI hallucination 0→30° → **REJECT**, car keeps driving straight |
| 3 | `inject envelope` | too much steering for the speed → **CLAMP** to 21.8° |
| 4 | `inject flicker` … `clear` | camera flicker → **CAUTION**, swerves blocked, speed halved |
| 5 | `obstacle 2 unknown` | camera cannot see the box → CAMERA_MISSED_OBSTACLE → **MRM** → stops ~10 cm before it |
| 6 | `obstacle off`, wait 2 s, `ack` | SAFE → RECOVERY → operator ack → NORMAL |
| 7 | `inject imu-frozen` | sensor isolated → **DEGRADED**, car continues slowly |
| 8 | `inject crash` | navigation dies → QNX kernel tells the supervisor → **MRM** + automatic restart |
| 9 | `inject hang` | navigation alive but silent → heartbeat timeout → **MRM**, killed + restarted |
| 10 | `inject link-drop` | pulled wire → simulated STM32 stops the car on its own |
| 11 | `rogue` | an unregistered program tries to drive → **refused (EPERM)** |
| 12 | `./cpu_stress -t 4 &` (shell), `status` | CPU saturated, supervisor timing unchanged |
| — | `status` | all values, decisions, timing and reaction times |

Run `./start_all.sh car vm` and `obstacle 30 unknown` at cruise to show the full-scale case:
80 km/h with an obstacle at 30 m → ESTOP at ~0.9 g.

## QNX concepts used

| Requirement | Mechanism |
|---|---|
| Supervisory control | supervisor is a message-passing server; navigation's `MsgSend` waits for the verdict |
| Priority override | SCHED_FIFO prio 50 vs nav 10; `_NTO_CHF_FIXED_PRIORITY` channel; control switches to recovery |
| Watchdog | 10 ms timer pulse; nav heartbeat pulse; `_PULSE_CODE_DISCONNECT` on crash; STM32 timeout |
| Fault tolerance | separate processes (memory isolation), 2-out-of-3 vote, graded states, `posix_spawn` restart |
| Timers | `timer_create` + `SIGEV_PULSE` (10 ms tick, 20 ms stop ramp, 50 ms nav, 100 ms logger) |
| Determinism | `mlockall`, no memory allocation after start, optional CPU pinning (`-c`), `ClockCycles` timing |
| Logging | lock-free shared-memory buffer (`shm_open` + `mmap`) read by a low-priority logger |
| Access control | only the navigation connection may send commands, only the supervisor may drive |

## Next stages (hardware)

- `safety_supervisor/leds_buzzer.c`: Pi 5 GPIO back-end (build with `EXTRA_CFLAGS=-DAV_GPIO_RPI`).
- `vehicle_link` (UART + CRC-8/alive-counter frames to the STM32) and `sensor_monitor` (MPU6050 on I²C) replace `vehicle_simulator`, using the same messages.
- `perception_gw` (UDP from the camera AI) replaces the simulated camera.
- STM32 firmware: safety watchdog, motors, encoders, HC-SR04, hardware stopwatch.

## Status

- Decision logic: 42/42 unit tests pass (`make test`).
- All QNX programs compile without warnings for aarch64le and x86_64 (QNX SDP 8.0).
- Runs on the QNX 8 x86_64 VM.

Declare this code and all open-source components as pre-built material, as required by the
hackathon rules.
