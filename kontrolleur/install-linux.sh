#!/bin/bash
# Kontrolleur on Linux (Fedora, Debian/Ubuntu, Raspberry Pi OS …)
#   ./install-linux.sh            install for this user: venv, launcher, menu entry
#   ./install-linux.sh --udev     also let you reach an MPC-20 plugged into this computer (asks for sudo)
#   ./install-linux.sh --remove   take it out again (keeps your notes and configurations)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}"
VENV="$DATA/kontrolleur/venv"
BIN="$HOME/.local/bin/kontrolleur"
DESKTOP="$DATA/applications/kontrolleur.desktop"
ICON="$DATA/icons/hicolor/256x256/apps/kontrolleur.png"

if [ "${1:-}" = "--remove" ]; then
  rm -rf "$VENV" "$BIN" "$DESKTOP" "$ICON"
  echo "Removed. Notes and the EC4 backup are still in $DATA/kontrolleur, configurations in ~/Documents/Kontrolleur."
  exit 0
fi

PY="$(command -v python3 || true)"
[ -n "$PY" ] || { echo "python3 is missing (Fedora: sudo dnf install python3)"; exit 1; }
"$PY" - <<'PYEOF' || { echo "Python 3.9 or newer is needed."; exit 1; }
import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)
PYEOF
command -v ssh >/dev/null || echo "note: ssh is missing (Fedora: sudo dnf install openssh-clients)"

echo "→ Python environment in $VENV …"
mkdir -p "$(dirname "$VENV")"
# system site packages: lets the venv use python3-rtmidi / python3-pyusb from dnf or apt
[ -x "$VENV/bin/python" ] || "$PY" -m venv --system-site-packages "$VENV"
"$VENV/bin/python" -m pip install --quiet --upgrade pip
"$VENV/bin/python" -m pip install --quiet "PySide6>=6.6" "pyusb>=1.2" || { echo "pip could not install PySide6."; exit 1; }
# python-rtmidi: only needed for the 16n and the EC4 on a cable to this computer
if ! "$VENV/bin/python" -c "import rtmidi" 2>/dev/null; then
  if ! "$VENV/bin/python" -m pip install --quiet --only-binary=:all: "python-rtmidi>=1.5" 2>/dev/null; then
    RTMIDI_MISSING=1
  fi
fi

echo "→ Launcher $BIN"
mkdir -p "$(dirname "$BIN")"
cat > "$BIN" <<LAUNCH
#!/bin/sh
exec "$VENV/bin/python" "$HERE/kontrolleur.py" "\$@"
LAUNCH
chmod +x "$BIN"

echo "→ Menu entry"
mkdir -p "$(dirname "$DESKTOP")" "$(dirname "$ICON")"
cp "$HERE/kontrolleur/icon.png" "$ICON"
cat > "$DESKTOP" <<DESK
[Desktop Entry]
Type=Application
Name=Kontrolleur
GenericName=Controller mapper
Comment=DualSense → i2c2midi → ER-301, 16n, MPC-20 and EC4 settings
Exec=$BIN
Icon=kontrolleur
Terminal=false
Categories=AudioVideo;Audio;Music;
StartupWMClass=kontrolleur
DESK
command -v update-desktop-database >/dev/null && update-desktop-database "$DATA/applications" >/dev/null 2>&1 || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q "$DATA/icons/hicolor" >/dev/null 2>&1 || true

if [ "${1:-}" = "--udev" ]; then
  echo "→ udev rule for devices on a cable (sudo)"
  sudo install -m 644 "$HERE/70-kontrolleur-cable.rules" /etc/udev/rules.d/70-kontrolleur-cable.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
fi

if [ "${RTMIDI_MISSING:-}" = 1 ]; then
  echo
  echo "note: python-rtmidi is missing. Everything over SSH works; for a 16n or EC4 on a cable to this computer:"
  echo "        Fedora:          sudo dnf install python3-rtmidi"
  echo "        Debian / Ubuntu: sudo apt install python3-rtmidi"
  echo "      then run ./install-linux.sh again."
fi
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) echo "note: add ~/.local/bin to your PATH to start it with 'kontrolleur'";; esac
echo "Done. Start Kontrolleur from the app menu or with: kontrolleur"
