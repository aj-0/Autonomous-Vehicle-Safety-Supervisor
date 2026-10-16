# AV Safety Supervisor — Autonomous Vehicle Safety Supervisor on QNX
QNX eHACK 2026 · Problem statement #17  Team NEXGEN INNOVATORS

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
| `safety_supervisor` | 50 (highest) | Checks every navigation command: PASS / CLAMP (made safer) / REJECT (blocked). Only it may drive the car. Owns the safe-state deadline timer and the GPIO safety outputs |
| `recovery_manager` | 45 | Smooth controlled stop; restarts navigation or the sensor monitor after a crash or freeze |
| `vehicle_simulator` | 40 | Simulated car + raw wheel sensors + ultrasonic + IMU + camera + motor watchdog (VM / demo) |
| `vehicle_link` | 48 | **Real rover** instead of the simulator: motors, encoders, LiDAR, buttons, relay (see "Real rover") |
| `perception_gateway` | 35 | **Real rover**: Jetson camera detections over UDP, checked, passed on |
| `sensor_monitor` | 30 | **Sensor Monitor task**: judges every sensor every 10 ms (DEAD / FROZEN / OUT OF RANGE / JUMP) and forwards readings + health to the supervisor |
| `navigation` | 10 (low) | Stand-in for the AI driver: sends speed + steering 20× per second; can be told to misbehave |
| `event_logger` | 8 | Writes the easy-to-read safety log, CSV files and black-box incident files |
| `operator_cli` | 5 | Operator console: start, status, inject faults, acknowledge |
| `cpu_stress` | 9 | Loads the CPU to prove the supervisor keeps its timing |

```
 navigation (prio 10) ──MsgSend(command)──► SAFETY SUPERVISOR (prio 50, fixed-priority channel)
   the untrusted AI   ◄──MsgReply(PASS / CLAMP / REJECT)──┘   │
   heartbeat pulse ──────────────────────────────────────────►│
 vehicle_simulator ──raw sensors──► sensor_monitor (prio 30) ──readings + health (100 Hz)──►│
 vehicle_simulator ──camera results (33 Hz)────────────────────►│
 recovery_manager ──stop commands (only accepted in MRM)──────►│
                  ◄──pulses: start stop / stop / restart nav / restart sensor monitor┤
 vehicle_simulator ◄──safe command every 10 ms (supervisor only)┤
 event_logger (prio 8) ◄──shared-memory buffer + pulse─────────┤
 GPIO: red/green LED, buzzer, inject marker, motor relay ◄──────┘
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
| `safety_supervisor/safety_deadline.*` | safe-state deadline timer: every stop must end in time (PASS / FAIL) |
| `safety_supervisor/final_command.*` | picks the one safe command sent to the car |
| `safety_supervisor/send_output.*` | connections to vehicle, recovery manager, logger |
| `safety_supervisor/leds_buzzer.*` | GPIO safety outputs: in memory on the VM, real pins on the Pi (`/dev/gpio/msg`) |
| `safety_supervisor/timing_stats.*` | cycle time, timer delay, reaction time |
| `sensor_monitor/` | Sensor Monitor task: sensor health checks between the vehicle and the supervisor; real MPU6050 over I2C (`imu_mpu6050.c`) |
| `vehicle_simulator/` | simulated car, raw sensors, camera and motor watchdog |
| `vehicle_link/` | real rover: L298N motors, encoders, TF-Luna, buttons, relay (`pi_gpio.c` = QNX GPIO messages) |
| `perception_gateway/` | real rover: Jetson UDP detections -> supervisor + navigation |
| `shared/hw_protocols.*` | TF-Luna frame parser, encoder speed, Jetson packet check (unit-tested) |
| `navigation/` | navigation stand-in with fault injection |
| `recovery_manager/` | controlled stop, restarts navigation and the sensor monitor |
| `event_logger/` | safety log, CSV files, black box |
| `operator_cli/` | operator console (mandatory CLI) |
| `tools/` | `start_all.sh` (simulation), `start_rover.sh` (real rover), `stop_all.sh`, `cpu_stress.c` |
| `tests/unit_tests.c` | 61 tests of the decision logic and rover drivers (run on the PC) |

## Build

Windows (Git Bash) or Linux, with QNX SDP 8.0 installed:

```sh
source ~/qnx800/qnxsdp-env.sh      # Windows Git Bash: source /c/Users/<you>/qnx800/qnxsdp-env.sh
make                               # Raspberry Pi 5 / Pi 4  -> build/aarch64le/
make ARCH=x86_64                   # QNX x86_64 VM          -> build/x86_64/
make GPIO=rpi                      # Pi with real GPIO pins -> build/aarch64le-gpio/
make test                          # PC unit tests (needs gcc in PATH)
```

`make GPIO=rpi` needs QNX's `sys/rpi_gpio.h` (the message interface of the Raspberry Pi GPIO
resource manager `/dev/gpio/msg`). It is published in QNX's samples repository
`gitlab.com/qnx/projects/hardware-component-samples`, file `common/system/gpio/sys/rpi_gpio.h`
(QNX licence, free for non-commercial use - do not commit it to this repository). Copy it to
`qnx800/target/qnx/usr/include/sys/`. The plain `make` build also runs on the Pi; it only keeps the
LED/relay states in memory.

## Real IMU (MPU6050 on the Pi)

Wire the MPU6050 to the Pi I2C bus (SDA GPIO2 / pin 3, SCL GPIO3 / pin 5, 3.3 V, GND), mount it
+X forward, +Y left, +Z up, then start with the I2C device of your image (check `ls /dev/i2c*`):

```sh
export AV_IMU_DEV=/dev/i2c1        # optional: export AV_IMU_ADDR=0x69 if AD0 is high
./start_all.sh robot
```

`sensor_monitor` checks WHO_AM_I, configures ±250 °/s, ±2 g, 100 Hz, calibrates the gyro during the
first 0.5 s (keep the car still and level), and then uses the real yaw rate and acceleration. A
stuck chip (identical data for 2 s), saturated values or gravity ≠ 1 g mean no fresh sample: the IMU
becomes DEAD and the supervisor continues in DEGRADED. The MPU6050 I2C code is based on the
Q-SAFE prototype by a team-mate (`sensor_monitor/imu_mpu6050.c`), declared as reused code.

## GPIO safety outputs (Raspberry Pi 40-pin header)

| GPIO | Pin | Output | Meaning |
|---|---|---|---|
| 17 | 11 | red LED | stopping or stopped (MRM, ESTOP, SAFE) |
| 27 | 13 | green LED | on = NORMAL, blinking = CAUTION / DEGRADED / RECOVERY |
| 22 | 15 | buzzer | stop in progress |
| 23 | 16 | inject marker | high from fault injection until the supervisor reacts (oscilloscope: override latency) |
| 24 | 18 | motor relay | HIGH = motors may run, LOW = motor power cut. Use a pull-down: an unpowered Pi cuts the motors. A crashed supervisor process does not (the pin keeps its last value) - an external dead-man/watchdog is the next hardware step |

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
   20.10 s  DEADLINE   stop started at 0.50 m/s, braking 1.0 m/s^2 -> must be standing still within 925 ms
   20.81 s  DEADLINE   PASS: safe state reached in 710 ms (deadline 925 ms, 215 ms to spare)
```

The safe-state deadline: `deadline = 100 ms + 1.25 × v0 / a + 200 ms` (delivery + 25 % slower
braking + standstill confirmation). Missed → `STOP_DEADLINE_MISSED` → ESTOP and motor relay cut.

Files in `/tmp/av_logs` (copy to Windows with the Momentics Target File System Navigator, open in Excel):

| File | Contents |
|---|---|
| `safety_log.txt` | the readable log above |
| `safety_events.csv` | every safety event with all values |
| `live_values.csv` | state, speed, obstacle distance, nav command, car command every 10 ms |
| `incident_001.csv` … | black box: 0.5 s before + 1 s after each emergency or blocked command |
| `supervisor.out`, `sensor_monitor.out`, `navigation.out`, `simulator.out`, `recovery.out` | console output of each program (`sensor_monitor.out` shows every sensor health change) |

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
| 13 | `inject brake-fail` | brakes weak → BRAKES_NOT_WORKING → ESTOP; deadline **MISSED** → motor relay cut, logged as FAIL |
| 14 | `slay sensor_monitor` (shell) | Sensor Monitor dies → SENSOR_LINK_LOST → **MRM**, recovery starts a new sensor monitor; car waits for `ack` |
| — | `status` | all values, decisions, timing, reaction times and deadline PASS/FAIL counts |

Run `./start_all.sh car vm` and `obstacle 30 unknown` at cruise to show the full-scale case:
80 km/h with an obstacle at 30 m → ESTOP at ~0.9 g.

## Problem statement #17 (PSQNX.xlsx) → where it is

| Excel column | Requirement | Where |
|---|---|---|
| Tasks & scheduling | Navigation Task | `navigation` (prio 10, 50 ms) |
| | Safety Supervisor (highest) | `safety_supervisor` (prio 50, 10 ms) |
| | Sensor Monitor | `sensor_monitor` (prio 30, 10 ms) |
| | Recovery Manager | `recovery_manager` (prio 45, 20 ms) |
| Inter-task communication | Message passing | `MsgSend`/`MsgReceive`/`MsgReply`, pulses, `name_attach` |
| Timers | Safety deadline | `safety_deadline.c` (checked on every 10 ms timer pulse) |
| | Heartbeat | nav heartbeat pulse, 100 ms timeout (`watchdog.c`) |
| | Recovery timer | 20 ms stop ramp, 2 s clear time before recovery (`safety_states.c`) |
| Logging | Safety events, override latency, health status | `safety_log.txt`, CSV files, REACTION / DEADLINE lines, `status` |
| Optional hardware | GPIO safety output | `leds_buzzer.c` (LEDs, buzzer, inject marker, motor relay) |
| Interface | CLI mandatory | `operator_cli` |

## QNX concepts used

| Requirement | Mechanism |
|---|---|
| Supervisory control | supervisor is a message-passing server; navigation's `MsgSend` waits for the verdict |
| Priority override | SCHED_FIFO prio 50 > recovery 45 > sensor monitor 30 > nav 10; `_NTO_CHF_FIXED_PRIORITY` channels; control switches to recovery |
| Watchdog | 10 ms timer pulse; nav heartbeat pulse; `_PULSE_CODE_DISCONNECT` on crash; STM32 timeout |
| Fault tolerance | separate processes (memory isolation), 2-out-of-3 vote, graded states, `posix_spawn` restart of navigation and sensor monitor |
| Timers | `timer_create` + `SIGEV_PULSE` (10 ms tick, 10 ms sensor monitor, 20 ms stop ramp, 50 ms nav, 100 ms logger); safe-state deadline |
| Determinism | `mlockall`, no memory allocation after start, optional CPU pinning (`-c`), `ClockCycles` timing |
| Logging | lock-free shared-memory buffer (`shm_open` + `mmap`) read by a low-priority logger |
| Access control | only the navigation connection may send commands, only the supervisor may drive |

## Real rover (Raspberry Pi only, no simulator)

`vehicle_link` replaces `vehicle_simulator` on the real car. It is the only program that touches
the motors and the vehicle sensors and speaks the same messages, so the supervisor, sensor monitor
and navigation are unchanged. The Jetson only sends camera detections; `perception_gateway`
receives and checks them on the Pi. No STM32, no Jetson code in this repository.

| Program | Priority | What it does |
|---|---|---|
| `vehicle_link` | 48 | L298N motors (PWM + direction), wheel encoders, TF-Luna LiDAR, E-stop / ACK buttons, motor relay, **motor watchdog** (no supervisor command for 30 ms -> motors stopped, relay open) |
| `perception_gateway` | 35 | Jetson detections over UDP -> checked (length, magic, CRC-16, ranges, order, sender IP) -> supervisor + navigation |

### Wiring (Raspberry Pi 5, 40-pin header)

| Part | Pi GPIO / pin | Notes |
|---|---|---|
| L298N ENA / ENB (PWM) | GPIO12 / pin 32, GPIO13 / pin 33 | remove the ENA/ENB jumpers |
| L298N IN1 IN2 (left) | GPIO5 / pin 29, GPIO6 / pin 31 | |
| L298N IN3 IN4 (right) | GPIO16 / pin 36, GPIO26 / pin 37 | L298N GND to Pi GND |
| Wheel encoders L / R | GPIO20 / pin 38, GPIO21 / pin 40 | LM393 modules powered from **3.3 V** (pin 1/17) |
| TF-Luna LiDAR | TX -> GPIO15 RX / pin 10, RX <- GPIO14 TX / pin 8 | TF-Luna on 5 V, UART mode (pin 5 open) |
| MPU6050 | SDA GPIO2 / pin 3, SCL GPIO3 / pin 5 | 3.3 V, flat, chip up |
| E-stop button (normally closed) | GPIO25 / pin 22 to GND | pressed or broken wire = STOP. Best: a second contact also in series with the motor battery |
| ACK button (normally open) | GPIO19 / pin 35 to GND | same as typing `ack` |
| Motor relay (3.3 V, active-HIGH) | GPIO24 / pin 18 | motor battery through the NO contact, pull-down on the input |
| LEDs / buzzer / marker | GPIO17, 27, 22, 23 | as above |

### Calibrate (`shared/settings.h`, then rebuild)

`AV_HW_WHEEL_DIAMETER_M`, `AV_HW_ENCODER_SLOTS`, `AV_HW_MOTOR_V_FULL` (speed at 100 % PWM),
`AV_HW_MOTOR_MIN_DUTY`, and `track_m` of the `rover` profile in `shared/settings.c` (distance
between the wheel centres). The `rover` profile drives at 0.3 m/s and stops 0.30 m before an
obstacle (outside the TF-Luna's 0.2 m blind zone).

### Run

```sh
su                                   # root: priorities, memory lock, GPIO
ls /dev/ser* /dev/i2c* /dev/gpio     # find the LiDAR UART and the I2C bus
export AV_LIDAR_DEV=/dev/ser1 AV_IMU_DEV=/dev/i2c1
./start_rover.sh bench               # first: wheels in the air on the desk
./start_rover.sh                     # then on the floor
```

Check `vehicle.out` (motors, LiDAR, encoders found) and `sensor_monitor.out` (IMU calibrated), then
`operator_cli` -> `start`, press the ACK button. Physical tests: hand in front of the LiDAR (car
slows and stops 0.3 m before it), E-stop, unplug the LiDAR / an encoder / the IMU wire, cover the
camera, `inject slew`, `inject crash`, `inject link-drop`, `./cpu_stress -t 4 &`.

### Jetson -> Pi camera packet (for the Jetson side)

UDP to the Pi, port 5005, one packet per camera frame, 24 bytes, little-endian
(`shared/hw_protocols.h`):

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `0x314A5641` ("AVJ1") |
| 4 | 2 | version `1` |
| 6 | 2 | length `24` |
| 8 | 4 | seq, +1 per frame |
| 12 | 1 | obstacle 0/1 |
| 13 | 1 | zone 0 none, 1 left, 2 center, 3 right |
| 14 | 1 | confidence 0-100 |
| 15 | 1 | class id (COCO) |
| 16 | 4 | distance in m (float), -1 = unknown |
| 20 | 2 | age in ms (capture -> send) |
| 22 | 2 | CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021) of bytes 0-21 |

Python: `body = struct.pack('<IHHIBBBBfH', 0x314A5641, 1, 24, seq, obst, zone, conf, cls, dist, age)`,
then append `struct.pack('<H', crc16(body))`. Without a Jetson (`AV_JETSON_PORT=off`) the rover still
drives safely on the LiDAR; the camera checks are simply inactive.

### Safety chain on the rover

| Failure | Who stops the motors |
|---|---|
| supervisor crashes or hangs | `vehicle_link` motor watchdog: 30 ms without a command -> motor PWM off, relay open |
| `vehicle_link` crashes | supervisor sees the link vanish -> forces the relay open |
| `vehicle_link` stopped normally | it stops the motors and opens the relay before exiting |
| E-stop pressed / wire broken | `vehicle_link` opens the relay at once, supervisor -> ESTOP |
| Pi loses power | relay input pulled low -> motor power cut |
| supervisor **and** `vehicle_link` dead at the same time | not covered in software: needs an external hardware watchdog |

## Status

- Decision logic and rover drivers: 61/61 unit tests pass (`make test`): deadline PASS/FAIL, TF-Luna frames,
  encoder speed, Jetson packet checks, rover profile, ACK button.
- All QNX programs compile without warnings for aarch64le and x86_64 (QNX SDP 8.0).
- The Pi GPIO build (`make GPIO=rpi`) compiles without warnings against QNX's real `sys/rpi_gpio.h`; it has not yet been run on a Pi.
- The MPU6050 code compiles against the SDK's `hw/i2c.h`; it has not yet been run with a real chip.
- The sensor monitor, deadline timer and GPIO changes have not yet been run on the QNX VM.
- `vehicle_link` and `perception_gateway` compile without warnings; they have not yet been run on
  the rover. PWM, GPIO events and the UART follow QNX's Raspberry Pi samples and must be verified
  on the board (`./start_rover.sh bench` first).

Declare this code and all open-source components as pre-built material, as required by the
hackathon rules.
