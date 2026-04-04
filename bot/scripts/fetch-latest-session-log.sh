#!/usr/bin/env bash
# Copy the newest MatchSessionLogger CSV from the roboRIO into repo ../logs/
# Usage: ./scripts/fetch-latest-session-log.sh [admin@host]
# Default host: admin@169.254.60.16 (override if your RIO uses another address)

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_LOGS="$BOT_DIR/../logs"
mkdir -p "$REPO_LOGS"

DEFAULT_HOST="admin@169.254.60.16"
HOST="${1:-$DEFAULT_HOST}"
REMOTE_PATH=$(ssh -o ConnectTimeout=10 "$HOST" 'ls -t /home/lvuser/logs/session_*.csv 2>/dev/null | head -1' || true)
if [[ -z "$REMOTE_PATH" ]]; then
  echo "No session_*.csv under /home/lvuser/logs/ on $HOST" >&2
  exit 1
fi

echo "Copying $HOST:$REMOTE_PATH -> $REPO_LOGS/"
scp "$HOST:$REMOTE_PATH" "$REPO_LOGS/"
ls -la "$REPO_LOGS"/session_*.csv 2>/dev/null | tail -5
