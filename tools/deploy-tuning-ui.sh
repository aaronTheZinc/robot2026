#!/usr/bin/env bash
# Build tuning-ui and copy output to deploy/tuning-ui (run from repo root).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"
cd tools/tuning-ui
npm run build
mkdir -p ../../deploy/tuning-ui
cp -r dist/* ../../deploy/tuning-ui/
echo "Deployed tuning-ui to deploy/tuning-ui/"
