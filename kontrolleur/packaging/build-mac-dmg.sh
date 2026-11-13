#!/bin/bash
# Kontrolleur.app on a .dmg for Apple Silicon Macs (macOS 14+), to give to someone without Python.
# Result: kontrolleur/dist/Kontrolleur-<date>.dmg. Not signed: the first start needs “Open Anyway”
# (see “Read me first.txt” on the disk image).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
APPDIR="$(dirname "$HERE")"
REPO="$(dirname "$APPDIR")"
BUILD="${KONTROLLEUR_BUILD:-$HOME/Library/Caches/kontrolleur-build}"    # build venv + work files (deletable)
VENV="$BUILD/venv"

PY=""
for c in /opt/homebrew/bin/python3 /usr/local/bin/python3 python3; do
  if command -v "$c" >/dev/null && "$c" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then PY="$c"; break; fi
done
[ -n "$PY" ] || { echo "Python 3.9 or newer is needed (brew install python)."; exit 1; }
[ "$(uname -m)" = arm64 ] || { echo "Build this on an Apple Silicon Mac."; exit 1; }

echo "→ build environment in $VENV"
mkdir -p "$BUILD"
[ -x "$VENV/bin/python" ] || "$PY" -m venv "$VENV"
"$VENV/bin/python" -m pip install --quiet --upgrade pip pyinstaller
"$VENV/bin/python" -m pip install --quiet -r "$APPDIR/requirements.txt"

VERSION="$(git -C "$REPO" log -1 --date=format:%Y.%m.%d --format=%cd)"
export KONTROLLEUR_VERSION="$VERSION"
echo "→ Kontrolleur.app ($VERSION)"
cd "$HERE"
"$VENV/bin/pyinstaller" --noconfirm --clean --log-level WARN \
  --distpath "$BUILD/dist" --workpath "$BUILD/work" kontrolleur.spec

echo "→ disk image"
STAGE="$BUILD/dmg"
rm -rf "$STAGE"
mkdir -p "$STAGE"
cp -R "$BUILD/dist/Kontrolleur.app" "$STAGE/"
ln -s /Applications "$STAGE/Applications"
cp "$HERE/Read me first.txt" "$STAGE/"
OUT="$APPDIR/dist"
mkdir -p "$OUT"
DMG="$OUT/Kontrolleur-$VERSION.dmg"
rm -f "$DMG"
hdiutil create -quiet -volname "Kontrolleur" -srcfolder "$STAGE" -fs HFS+ -format UDZO "$DMG"
rm -rf "$STAGE" "$BUILD/work"
echo "Done: $DMG ($(du -h "$DMG" | cut -f1))"
