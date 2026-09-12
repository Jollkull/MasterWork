#!/usr/bin/env bash
set -euo pipefail

ros2 run puma560_http_ingest puma560_http_ingest --ros-args \
  -p endpoint_url:="${ENDPOINT_URL:-http://localhost:9999/unused-placeholder}" \
  -p debounce_sec:="${DEBOUNCE_SEC:-2.0}" \
  -p state_epsilon:="${STATE_EPSILON:-0.005}" \
  -p robot_id:="${ROBOT_ID:-puma560}"

