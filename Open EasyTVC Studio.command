#!/bin/bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
export PATH="/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:$PATH"
if [[ ! -f "$PROJECT_ROOT/build-host/libeasytvc_studio.dylib" && ! -f "$PROJECT_ROOT/build-host/libeasytvc_studio.so" ]]; then
    "$PROJECT_ROOT/scripts/build-studio.sh"
fi
exec python3 "$PROJECT_ROOT/studio/studio.py"
