# Kontrolleur

Mac app that configures the whole rig of this repository from one place:

| Tab | Device | How it talks to it |
|---|---|---|
| DualSense | PS5 controller on the Pi | `er301_routes.py` + `controls.py` over SSH (`scripts/er301_routes_tool.py`) |
| 16n | 16n faderbank (fork with curves) | `scripts/fader16_tool.py`, on the Pi or this Mac |
| Pedals | DOREMiDi MPC-20 | `scripts/mpc20_tool.py`, on the Pi or this Mac |
| EC4 | Faderfox EC4 | `scripts/ec4_tool.py` (transfers are started on the EC4) |
| ER-301 | i2c2midi → ER-301 | port overview, MIDI routes, pitch & LFO |

Also: notes per control (also written into `er301_routes.py` comments), saved configurations
(`~/Documents/Kontrolleur/*.dsmconfig`), adjustable appearance, zoomable drawings.

## Build

```sh
cd kontrolleur
scripts/make_app.sh        # → ~/Applications/Kontrolleur.app
```

Needs Swift 5.10 / macOS 14. The app bundles the three device tools from `../scripts/`.
For devices plugged into the Mac, Python needs `python-rtmidi` (16n, EC4).
