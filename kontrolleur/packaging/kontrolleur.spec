# PyInstaller spec: Kontrolleur.app for Apple Silicon Macs. Built by build-mac-dmg.sh.
import ast
import importlib.util
import os
from pathlib import Path

HERE = Path(SPECPATH)
APP = HERE.parent
REPO = APP.parent
VERSION = os.environ.get("KONTROLLEUR_VERSION", "0")
# device tools that run on this computer in cable mode (packed as data, run with `Kontrolleur --run-script`)
TOOLS = ["fader16_tool.py", "ec4_tool.py", "mpc20_tool.py", "rtmidi_compat.py"]


def imports(path):
    names = set()
    for n in ast.walk(ast.parse(path.read_text())):
        if isinstance(n, ast.Import):
            names |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module and n.level == 0:
            names.add(n.module.split(".")[0])
    return names


# the tools are not analysed by PyInstaller, so everything they import goes in as a hidden import
hidden = sorted(m for t in TOOLS for m in imports(REPO / "scripts" / t)
                if m != "rtmidi_compat" and importlib.util.find_spec(m) is not None) + ["runpy"]

a = Analysis(
    [str(APP / "kontrolleur.py")],
    pathex=[str(APP)],
    hiddenimports=hidden,
    datas=[(str(REPO / "scripts" / t), "scripts") for t in TOOLS]
          + [(str(APP / "kontrolleur" / "icon.png"), "kontrolleur")],
    excludes=["tkinter"],
)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="Kontrolleur", console=False,
          target_arch="arm64", codesign_identity=None)
coll = COLLECT(exe, a.binaries, a.datas, name="Kontrolleur")
app = BUNDLE(
    coll,
    name="Kontrolleur.app",
    icon=str(APP / "scripts" / "AppIcon.icns"),
    bundle_identifier="com.kontrolleur.Kontrolleur",
    version=VERSION,
    info_plist={
        "CFBundleDisplayName": "Kontrolleur",
        "CFBundleShortVersionString": VERSION,
        "LSMinimumSystemVersion": "14.0",
        "NSHighResolutionCapable": True,
        "LSApplicationCategoryType": "public.app-category.music",
    },
)
