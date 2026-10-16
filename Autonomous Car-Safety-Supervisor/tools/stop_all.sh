#!/bin/sh
# Stop every AV Safety Supervisor program (recovery first, so it does not restart navigation;
# vehicle_link next: on SIGTERM it stops the motors and opens the relay before it exits).
for p in recovery_manager vehicle_link navigation perception_gateway sensor_monitor safety_supervisor vehicle_simulator event_logger; do
    slay -f "$p" 2>/dev/null
done
echo "AV Safety Supervisor stopped."
