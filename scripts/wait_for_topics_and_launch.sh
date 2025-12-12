#!/usr/bin/env bash
set -euo pipefail

# Wait until any ROS2 topic other than /parameter_events and /rosout exists,
# then run the command passed as arguments.
# Usage examples:
#  ROS_DOMAIN_ID=97 ./wait_for_topics_and_launch.sh python3 scripts/your_launcher.py
#  ./wait_for_topics_and_launch.sh -- python3 scripts/your_launcher.py

if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <command...>"
  exit 2
fi

echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<not set>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<not set>}"
echo "Waiting for any topic besides /parameter_events and /rosout..."

while true; do
  # Get topics; ignore errors
  topics=$(ros2 topic list 2>/dev/null || true)
  # Loop lines; check for any non-empty and not equal those two
  found=false
  while IFS= read -r line; do
    # skip empty
    [ -z "$line" ] && continue
    if [ "$line" = "/parameter_events" ] || [ "$line" = "/rosout" ]; then
      continue
    fi
    found=true
    break
  done <<< "$topics"
  if [ "$found" = true ]; then
    break
  fi
  sleep 0.1
done

echo "Detected topics. Running: $*"
exec "$@"
