#!/usr/bin/env python3
"""Kontrolleur — configure the DualSense → i2c2midi → ER-301 rig, the 16n, the MPC-20 and the EC4."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kontrolleur.app import main  # noqa: E402

if __name__ == "__main__":
    main()
