#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$PROJECT_ROOT" -B "$PROJECT_ROOT/build-host"
cmake --build "$PROJECT_ROOT/build-host" --parallel 4
ctest --test-dir "$PROJECT_ROOT/build-host" --output-on-failure
if [[ "$(uname -s)" == Darwin ]]; then
    APP_DIR="$PROJECT_ROOT/EasyTVC Studio.app"
    mkdir -p "$APP_DIR/Contents/MacOS" "$APP_DIR/Contents/Resources/web" "$PROJECT_ROOT/build-host/swift-cache"
    swiftc -module-cache-path "$PROJECT_ROOT/build-host/swift-cache" -O \
        "$PROJECT_ROOT/studio/Launcher.swift" -o "$APP_DIR/Contents/MacOS/EasyTVCStudio" -framework AppKit
    cp "$PROJECT_ROOT/studio/Info.plist" "$APP_DIR/Contents/Info.plist"
    cp "$PROJECT_ROOT/studio/studio.py" "$APP_DIR/Contents/Resources/"
    cp "$PROJECT_ROOT/build-host/libeasytvc_studio.dylib" "$APP_DIR/Contents/Resources/"
    cp "$PROJECT_ROOT/studio/web/index.html" "$PROJECT_ROOT/studio/web/app.js" \
        "$PROJECT_ROOT/studio/web/style.css" "$APP_DIR/Contents/Resources/web/"
    codesign --force --sign - "$APP_DIR"
    echo "Ready: $APP_DIR"
else
    echo "Ready: python3 studio/studio.py"
fi
