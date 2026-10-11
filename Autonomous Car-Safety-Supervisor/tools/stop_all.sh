#!/bin/sh
# Stop every AV Safety Supervisor program (recovery first, so it does not restart navigation).
for p in recovery_manager navigation safety_supervisor vehicle_simulator event_logger; do
    slay -f "$p" 2>/dev/null
done
echo "AV Safety Supervisor stopped."
