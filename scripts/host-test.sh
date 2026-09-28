#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$ROOT" -B "$ROOT/build-host"
cmake --build "$ROOT/build-host"
ctest --test-dir "$ROOT/build-host" --output-on-failure
"$ROOT/build-host/easytvc_sim"
