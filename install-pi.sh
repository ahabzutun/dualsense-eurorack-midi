#!/usr/bin/env bash
# Sets up a Raspberry Pi (Raspberry Pi OS / Debian) as the MIDI hub of this project:
# packages, udev rules, user groups and the two services, for the current user and this checkout.
#
#   git clone https://github.com/ahabzutun/dualsense-eurorack-midi.git ~/dualsense-eurorack-midi
#   cd ~/dualsense-eurorack-midi && ./install-pi.sh
#
# Run it as the user the services should run as (not as root); it asks for sudo where needed.
# Running it again is safe: it only rewrites what it installs.
#
# Options:
#   --dry-run       show what would be done, change nothing
#   --no-services   packages, udev rules and groups only (start the scripts by hand)
#   --no-apt        skip apt (packages already installed)
#   --grid-macros   also: the Pi's USB-C as Teletype's grid + keyboard (USB gadget), the grid macros and
#                   the keyboard forwarder (docs/grid-macros.md); needs a reboot the first time

set -euo pipefail

DRY=0; SERVICES=1; APT=1; GRID=0
for a in "$@"; do
    case "$a" in
        --dry-run) DRY=1 ;;
        --no-services) SERVICES=0 ;;
        --no-apt) APT=0 ;;
        --grid-macros) GRID=1 ;;
        -h|--help) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $a (see --help)" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_NAME="$(id -un)"

if [ "$(id -u)" -eq 0 ]; then
    echo "Run this as the user the services should run as (it uses sudo itself), not as root." >&2
    exit 1
fi
if [ ! -f "$REPO/src/dualsense_midi.py" ] || [ ! -f "$REPO/src/midi_passthrough.py" ]; then
    echo "This doesn't look like the dualsense-eurorack-midi checkout: $REPO" >&2
    exit 1
fi
if [ "$(uname -s)" != "Linux" ] || ! command -v apt-get >/dev/null; then
    echo "This is for the Raspberry Pi (Debian / Raspberry Pi OS with apt)." >&2
    exit 1
fi

step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
run() {
    if [ "$DRY" -eq 1 ]; then printf '  + %s\n' "$*"; else "$@"; fi
}
# write stdin to a root-owned file
put() {
    local dest="$1" tmp
    tmp="$(mktemp)"; cat > "$tmp"
    if [ "$DRY" -eq 1 ]; then
        printf '  + write %s:\n' "$dest"; sed 's/^/      /' "$tmp"
    else
        sudo install -m 0644 "$tmp" "$dest"
    fi
    rm -f "$tmp"
}

echo "Repository: $REPO"
echo "User:       $USER_NAME"
[ "$DRY" -eq 1 ] && echo "(dry run: nothing is changed)"

if [ "$APT" -eq 1 ]; then
    step "Packages"
    run sudo apt-get update
    run sudo apt-get install -y git alsa-utils python3-rtmidi python3-evdev python3-usb
fi

step "udev rules"
# 70-dualsense:   light bar, dots and rumble without root (hidraw)
# 99-mpc20-midi:  the MPC-20 pedals' CH345 MIDI interface gets its ALSA port reliably
# 99-doremidi:    the MPC-20 tool may talk to the pedals over USB
for r in 70-dualsense.rules 99-mpc20-midi.rules 99-doremidi.rules; do
    run sudo install -m 0644 "$REPO/udev/$r" "/etc/udev/rules.d/$r"
done
run sudo udevadm control --reload-rules
run sudo udevadm trigger

step "Groups"
# input: read the DualSense (evdev) · audio: ALSA MIDI · plugdev: USB devices
GROUPS_WANTED="input audio plugdev"
[ "$GRID" -eq 1 ] && GROUPS_WANTED="$GROUPS_WANTED dialout"   # dialout: grid + gadget serial ports
for g in $GROUPS_WANTED; do
    if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx "$g"; then
        echo "  $USER_NAME is in $g"
    else
        run sudo usermod -aG "$g" "$USER_NAME"
        ADDED_GROUP=1
    fi
done

if [ "$SERVICES" -eq 1 ]; then
    step "Services"
    # the unit files in systemd/ are written for user ahab in /home/ahab/dualsense-eurorack-midi:
    # install them with this user and this checkout instead
    for unit in dualsense-midi.service midi-passthrough.service; do
        sed -e "s|^User=.*|User=$USER_NAME|" \
            -e "s|/home/ahab/dualsense-eurorack-midi|$REPO|g" \
            "$REPO/systemd/$unit" | put "/etc/systemd/system/$unit"
    done
    run sudo systemctl daemon-reload
    run sudo systemctl enable dualsense-midi.service midi-passthrough.service
    run sudo systemctl restart dualsense-midi.service midi-passthrough.service
fi

if [ "$GRID" -eq 1 ]; then
    step "Grid macros (USB gadget towards Teletype)"
    # the USB-C port becomes a USB device; the USB-A ports are another controller and stay hosts
    CFG=/boot/firmware/config.txt
    [ -f "$CFG" ] || CFG=/boot/config.txt
    if grep -q '^dtoverlay=dwc2' "$CFG" 2>/dev/null; then
        echo "  $CFG already has $(grep '^dtoverlay=dwc2' "$CFG")"
    else
        if [ "$DRY" -eq 1 ]; then
            echo "  + append to $CFG: [all] dtoverlay=dwc2,dr_mode=peripheral"
        else
            printf '\n[all]\ndtoverlay=dwc2,dr_mode=peripheral\n' | sudo tee -a "$CFG" >/dev/null
        fi
        NEED_REBOOT=1
    fi
    printf 'dwc2\nlibcomposite\n' | put /etc/modules-load.d/teletype-gadget.conf
    if [ "$SERVICES" -eq 1 ]; then
        for unit in teletype-gadget.service grid-macros.service keyboard-forward.service; do
            sed -e "s|^User=.*|User=$USER_NAME|" \
                -e "s|/home/ahab/dualsense-eurorack-midi|$REPO|g" \
                "$REPO/systemd/$unit" | put "/etc/systemd/system/$unit"
        done
        run sudo systemctl daemon-reload
        run sudo systemctl enable teletype-gadget.service grid-macros.service keyboard-forward.service
        if [ "${NEED_REBOOT:-0}" -eq 0 ]; then
            run sudo systemctl restart teletype-gadget.service grid-macros.service keyboard-forward.service
        fi
    fi
fi

step "Done"
cat <<EOF
Next:
  - Plug the DualSense in with a USB data cable (no pairing needed), and the XIAO bridges,
    16n / EC4 / pedals as far as you have them. The hub finds everything by name.
  - From the computer that runs Kontrolleur:  ssh-copy-id $USER_NAME@$(hostname).local
    then set that host in Kontrolleur.
  - Logs:  journalctl -u dualsense-midi -f   ·   journalctl -u midi-passthrough -f
EOF
if [ "${NEED_REBOOT:-0}" -eq 1 ]; then
    echo "  - Grid macros: reboot once so the USB-C port can become Teletype's grid + keyboard."
fi
if [ "${ADDED_GROUP:-0}" -eq 1 ]; then
    echo "  - You were added to a group: log out and in again (or reboot) before running the scripts by hand."
fi
