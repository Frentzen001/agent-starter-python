#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${MORETEA_START_SPEACHES:-1}" == "1" ]]; then
  "$ROOT_DIR/scripts/run_speaches.sh"
fi

echo "Prerequisite: the visible tmux tunnel pane must be running cleanly before starting the voice worker."
echo "Prerequisite: run /home/frentzen/FYP/moretea-robot-mcp/scripts/probe_tunneled_mcp.sh and confirm MCP health succeeds before relying on robot tools."

"$ROOT_DIR/scripts/run_openclaw_barebone.sh"
