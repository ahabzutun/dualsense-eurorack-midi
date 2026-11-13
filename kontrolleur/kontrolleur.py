#!/usr/bin/env python3
"""Kontrolleur — configure the DualSense → i2c2midi → ER-301 rig, the 16n, the MPC-20 and the EC4."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _run_script():
    """Packed app: `Kontrolleur --run-script tool.py args…` runs one of the device tools (cable mode)."""
    import runpy
    script = sys.argv[2]
    sys.argv = [script, *sys.argv[3:]]
    sys.path.insert(0, os.path.dirname(script))
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--run-script":
        _run_script()
    else:
        from kontrolleur.app import main
        main()
