#!/bin/sh
# Start the AV Safety Supervisor (simulation, no hardware) on a QNX target.
#
#   ./start_all.sh [robot|car] [vm] [bench]
#
#   robot : small prototype car, 0.5 m/s (default)
#   car   : full-size car numbers, 80 km/h
#   vm    : QNX virtual machine - communication timeouts x5 (the host PC can pause the VM).
#           Leave it out on the Raspberry Pi.
#
# Files written to /tmp/av_logs:
#   safety_log.txt       easy-to-read log with values   ->  tail -f /tmp/av_logs/safety_log.txt
#   safety_events.csv    every safety event (open in Excel)
#   live_values.csv      speed, distance, commands every 10 ms
#   incident_001.csv     black box: 0.5 s before + 1 s after each emergency
#   *.out                console output of each program
#
# Control:  ./operator_cli     (then: start, status, inject slew, obstacle 2 unknown, ...)

DIR=$(cd "$(dirname "$0")" && pwd)
PROFILE=${1:-robot}
BENCH=""
for a in "$2" "$3"; do
    [ "$a" = "bench" ] && BENCH="-b"
    [ "$a" = "vm" ] && export AV_TIMESCALE=5
done
LOG=/tmp/av_logs
mkdir -p $LOG

"$DIR/stop_all.sh" >/dev/null 2>&1

"$DIR/event_logger"      -d $LOG                             > $LOG/safety_log.txt  2>&1 &
sleep 1
"$DIR/vehicle_simulator" -s $PROFILE                         > $LOG/simulator.out   2>&1 &
"$DIR/recovery_manager"  -s $PROFILE -n "$DIR/navigation"    > $LOG/recovery.out    2>&1 &
"$DIR/safety_supervisor" -s $PROFILE $BENCH                  > $LOG/supervisor.out  2>&1 &
sleep 1
"$DIR/navigation"        -s $PROFILE                         > $LOG/navigation.out  2>&1 &
sleep 1

echo "AV Safety Supervisor running (profile $PROFILE, timescale x${AV_TIMESCALE:-1})"
echo "Programs (pid, thread, name, priority, state):"
pidin | grep -E "safety_super|vehicle_simul|navigation|recovery_man|event_logger"
echo
echo "Next:  $DIR/operator_cli                  (then: start, status, inject slew, ...)"
echo "Log:   tail -f $LOG/safety_log.txt"
