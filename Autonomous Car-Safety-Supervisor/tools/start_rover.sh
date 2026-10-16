#!/bin/sh
# Start the AV Safety Supervisor on the REAL rover (Raspberry Pi 5, QNX 8). No simulator:
# vehicle_link drives the L298N motors and reads the encoders, TF-Luna LiDAR and buttons.
# Use the build from "make GPIO=rpi" (build/aarch64le-gpio). Run as root.
#
#   ./start_rover.sh [bench]
#
#   bench : wheels in the air on the desk - turn-rate voting is switched off
#
# Hardware settings (environment variables, check the names on your Pi first):
#   AV_LIDAR_DEV   TF-Luna UART        default /dev/ser1   (ls /dev/ser*)
#   AV_IMU_DEV     MPU6050 I2C bus     default /dev/i2c1   (ls /dev/i2c*)
#   AV_JETSON_PORT camera UDP port     default 5005, "off" = no Jetson
#   AV_JETSON_IP   accept camera packets only from this address (optional)
#
# Logs in /tmp/av_logs (same files as start_all.sh, plus vehicle.out and camera.out).

DIR=$(cd "$(dirname "$0")" && pwd)
BENCH=""
[ "$1" = "bench" ] && BENCH="-b"
export AV_LIDAR_DEV=${AV_LIDAR_DEV:-/dev/ser1}
export AV_IMU_DEV=${AV_IMU_DEV:-/dev/i2c1}
PORT=${AV_JETSON_PORT:-5005}
LOG=/tmp/av_logs
mkdir -p $LOG

if [ ! -x "$DIR/vehicle_link" ]; then
    echo "vehicle_link missing: copy the files of build/aarch64le-gpio (make GPIO=rpi)"
    exit 1
fi

"$DIR/stop_all.sh" >/dev/null 2>&1

"$DIR/event_logger"      -d $LOG                                     > $LOG/safety_log.txt     2>&1 &
sleep 1
"$DIR/recovery_manager"  -s rover -n "$DIR/navigation" -m "$DIR/sensor_monitor" > $LOG/recovery.out 2>&1 &
"$DIR/safety_supervisor" -s rover -x $BENCH                         > $LOG/supervisor.out     2>&1 &
sleep 1
"$DIR/sensor_monitor"    -s rover                                   > $LOG/sensor_monitor.out 2>&1 &
"$DIR/vehicle_link"      -s rover -v                                > $LOG/vehicle.out        2>&1 &
if [ "$PORT" != "off" ]; then
    if [ -n "$AV_JETSON_IP" ]; then
        "$DIR/perception_gateway" -p $PORT -j "$AV_JETSON_IP"      > $LOG/camera.out         2>&1 &
    else
        "$DIR/perception_gateway" -p $PORT                          > $LOG/camera.out         2>&1 &
    fi
fi
sleep 1
"$DIR/navigation"        -s rover                                   > $LOG/navigation.out     2>&1 &
sleep 1

echo "AV Safety Supervisor running on the REAL rover (LiDAR $AV_LIDAR_DEV, IMU $AV_IMU_DEV, camera port $PORT)"
echo "Programs (pid, thread, name, priority, state):"
pidin | grep -E "safety_super|sensor_monit|vehicle_link|perception_g|navigation|recovery_man|event_logger"
echo
echo "Check:  cat $LOG/vehicle.out $LOG/sensor_monitor.out      (hardware found?)"
echo "Next:   $DIR/operator_cli    then: start, and press the ACK button (or type ack)"
echo "Log:    tail -f $LOG/safety_log.txt"
