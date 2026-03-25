#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${MORETEA_START_SPEACHES:-1}" == "1" ]]; then
  "$ROOT_DIR/scripts/run_speaches.sh"
fi

echo "Reminder: keep the robot MCP server and SSH tunnel running before starting the voice worker."

"$ROOT_DIR/scripts/run_openclaw_barebone.sh"
