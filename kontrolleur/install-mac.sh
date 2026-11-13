#!/bin/bash
# Kontrolleur on macOS: Python environment + ~/Applications/Kontrolleur.app
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SUPPORT="$HOME/Library/Application Support/Kontrolleur"
VENV="$SUPPORT/venv"
APP="$HOME/Applications/Kontrolleur.app"

PY=""
for c in /opt/homebrew/bin/python3 /usr/local/bin/python3 python3; do
  if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)'; then PY="$(command -v "$c")"; break; fi
done
[ -n "$PY" ] || { echo "Python 3.9 or newer is needed (brew install python)."; exit 1; }

echo "→ Python environment ($PY) in $VENV …"
mkdir -p "$SUPPORT"
[ -x "$VENV/bin/python" ] || "$PY" -m venv "$VENV"
"$VENV/bin/python" -m pip install --quiet --upgrade pip
# only Qt core / GUI / widgets: drop the full PySide6 (web engine, 3D, PDF …) if an older install pulled it in
"$VENV/bin/python" -m pip uninstall --quiet -y PySide6 PySide6_Addons PySide6_WebEngine PySide6_Pdf 2>/dev/null || true
"$VENV/bin/python" -m pip install --quiet --force-reinstall --no-deps "PySide6-Essentials>=6.6" "shiboken6>=6.6"
"$VENV/bin/python" -m pip install --quiet -r "$HERE/requirements.txt"

echo "→ $APP"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cat > "$APP/Contents/MacOS/Kontrolleur" <<LAUNCH
#!/bin/sh
exec "$VENV/bin/python" "$HERE/kontrolleur.py" "\$@"
LAUNCH
chmod +x "$APP/Contents/MacOS/Kontrolleur"
[ -f "$HERE/scripts/AppIcon.icns" ] && cp "$HERE/scripts/AppIcon.icns" "$APP/Contents/Resources/AppIcon.icns"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>Kontrolleur</string>
  <key>CFBundleDisplayName</key><string>Kontrolleur</string>
  <key>CFBundleIdentifier</key><string>net.ahab.kontrolleur</string>
  <key>CFBundleExecutable</key><string>Kontrolleur</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>2.0</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict></plist>
PLIST
codesign --force --deep --sign - "$APP" >/dev/null 2>&1 || true
echo "Done: $APP"
