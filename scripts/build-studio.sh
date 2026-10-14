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
        "$PROJECT_ROOT/studio/Launcher.swift" -o "$APP_DIR/Contents/MacOS/EasyTVCStudio" -framework AppKit -framework WebKit
    ICONSET="$PROJECT_ROOT/build-host/EasyTVCStudio.iconset"
    rm -rf "$ICONSET"
    swiftc -module-cache-path "$PROJECT_ROOT/build-host/swift-cache" "$PROJECT_ROOT/studio/make_icon.swift" \
        -o "$PROJECT_ROOT/build-host/make_icon" -framework AppKit
    "$PROJECT_ROOT/build-host/make_icon" "$ICONSET"
    iconutil -c icns "$ICONSET" -o "$APP_DIR/Contents/Resources/AppIcon.icns"
    cp "$PROJECT_ROOT/studio/Info.plist" "$APP_DIR/Contents/Info.plist"
    cp "$PROJECT_ROOT/studio/studio.py" "$PROJECT_ROOT/studio/launch.py" \
        "$PROJECT_ROOT/scripts/telemetry_viewer.py" "$APP_DIR/Contents/Resources/"
    cp "$PROJECT_ROOT/build-host/libeasytvc_studio.dylib" "$APP_DIR/Contents/Resources/"
    cp "$PROJECT_ROOT/studio/web/index.html" "$PROJECT_ROOT/studio/web/app.js" "$PROJECT_ROOT/studio/web/launch.js" "$PROJECT_ROOT/studio/web/sim3d.js" "$PROJECT_ROOT/studio/web/gimbal.js" \
        "$PROJECT_ROOT/studio/web/style.css" "$PROJECT_ROOT/studio/web/terminal.css" "$APP_DIR/Contents/Resources/web/"
    codesign --force --sign - "$APP_DIR"
    echo "Ready: $APP_DIR"
else
    echo "Ready: python3 studio/studio.py"
fi
