#!/bin/bash
# Builds "Kontrolleur.app" into ~/Applications
set -euo pipefail
cd "$(dirname "$0")/.."
swift build -c release
if [ ! -f scripts/AppIcon.icns ]; then
  [ -f scripts/AppIcon-1024.png ] || python3 scripts/make_icon.py scripts/AppIcon-1024.png
  ICONSET=$(mktemp -d)/AppIcon.iconset; mkdir -p "$ICONSET"
  for s in 16 32 128 256 512; do
    sips -z $s $s scripts/AppIcon-1024.png --out "$ICONSET/icon_${s}x${s}.png" >/dev/null
    d=$((s*2)); sips -z $d $d scripts/AppIcon-1024.png --out "$ICONSET/icon_${s}x${s}@2x.png" >/dev/null
  done
  iconutil -c icns "$ICONSET" -o scripts/AppIcon.icns
fi
APP="$HOME/Applications/Kontrolleur.app"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp .build/release/DualSenseMapper "$APP/Contents/MacOS/"
cp scripts/Info.plist "$APP/Contents/Info.plist"
cp scripts/AppIcon.icns "$APP/Contents/Resources/"
# device tools come from the repository's scripts/ folder (the same files run on the Pi)
for t in fader16_tool.py mpc20_tool.py ec4_tool.py; do cp "../scripts/$t" "$APP/Contents/Resources/"; done
codesign --force --deep --sign - "$APP" >/dev/null 2>&1 || true
echo "built: $APP"
