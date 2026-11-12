#!/bin/bash
# Deploys the re-assignable button functions (controls.py + ButtonRouter) to the Pi.
#   scripts/deploy_controls_to_pi.sh [ssh-host]      default: ahab@ahab.local with ~/.ssh/id_ed25519_pi
# Safe order: check the Pi still has the files this was built on -> back up -> copy -> run tests
# on the Pi -> restart dualsense-midi only if the tests pass.
set -euo pipefail
cd "$(dirname "$0")/.."
HOST="${1:-ahab@ahab.local}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=6 -o IdentitiesOnly=yes -i "$HOME/.ssh/id_ed25519_pi" "$HOST")
REMOTE=dualsense-eurorack-midi
BASE=4ef382c                      # commit these changes were made on
STAMP=$(date +%Y%m%d-%H%M%S)

echo "== 1. Pi files unchanged since $BASE?"
for f in src/dualsense_midi.py src/midi/dualsense_hid.py; do
  want=$(git show "$BASE:$f" | shasum -a 256 | cut -c1-64)
  have=$("${SSH[@]}" "sha256sum $REMOTE/$f" | cut -c1-64)
  if [ "$want" != "$have" ]; then echo "   $f differs on the Pi — stopping, nothing changed."; exit 1; fi
  echo "   $f ok"
done

echo "== 2. Backups on the Pi (*.bak.controls-$STAMP)"
"${SSH[@]}" "cd $REMOTE && for f in src/dualsense_midi.py src/midi/dualsense_hid.py src/config/mappings.py scripts/er301_routes_tool.py; do cp \$f \$f.bak.controls-$STAMP; done"

echo "== 3. Copy files"
tar czf - src/controls_config.py src/config/controls.py src/midi/button_router.py src/dualsense_midi.py \
          src/midi/dualsense_hid.py scripts/er301_routes_tool.py test/test_button_router.py \
  | "${SSH[@]}" "cd $REMOTE && tar xzf -"
"${SSH[@]}" "cd $REMOTE && grep -q btn_l1 src/config/mappings.py || python3 - <<'EOF'
import pathlib
p = pathlib.Path('src/config/mappings.py'); s = p.read_text()
old = \"    'btn_west': 23,      # □ button trigger (General Purpose)\n\"
assert s.count(old) == 1, 'mappings.py anchor not found'
s = s.replace(old, old + \"    # Buttons that send their own CC when set to 'midi' in controls.py\n    'btn_l1': 24, 'btn_r1': 25, 'btn_l3': 20, 'btn_r3': 21,\n    'btn_create': 26, 'btn_options': 27, 'btn_ps': 28, 'btn_mic': 29, 'btn_touchpad': 9,\n\")
p.write_text(s); print('   mappings.py: button CCs added')
EOF"

echo "== 4. Tests on the Pi"
if ! "${SSH[@]}" "cd $REMOTE && python3 -m py_compile src/dualsense_midi.py src/midi/button_router.py && python3 test/test_button_router.py | tail -1"; then
  echo "   tests failed — restoring backups"
  "${SSH[@]}" "cd $REMOTE && for f in src/dualsense_midi.py src/midi/dualsense_hid.py src/config/mappings.py scripts/er301_routes_tool.py; do cp \$f.bak.controls-$STAMP \$f; done; rm -f src/midi/button_router.py src/controls_config.py src/config/controls.py"
  exit 1
fi

echo "== 5. Restart dualsense-midi"
"${SSH[@]}" "echo pi | sudo -S systemctl restart dualsense-midi 2>/dev/null; sleep 6; systemctl is-active dualsense-midi; journalctl -u dualsense-midi --since '-15s' --no-pager -o cat | grep -E 'Controls|controls|router|Error|Traceback' | tail -5"
echo "done. Undo: restore the *.bak.controls-$STAMP files and remove button_router.py / controls_config.py / config/controls.py"
