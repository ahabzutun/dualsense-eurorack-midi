"""Files Kontrolleur keeps on this computer: notes, the last EC4 dump, saved configurations."""
from __future__ import annotations

import datetime as dt
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

from PySide6.QtCore import QSettings

APP = "Kontrolleur"


def settings() -> QSettings:
    return QSettings(APP, APP)


def data_dir() -> Path:
    """macOS: the folder the Swift app used, so notes and the EC4 dump carry over."""
    if os.environ.get("KONTROLLEUR_DATA"):
        d = Path(os.environ["KONTROLLEUR_DATA"])
    elif sys.platform == "darwin":
        d = Path.home() / "Library" / "Application Support" / "DualSense Mapper"
    else:
        d = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share")) / "kontrolleur"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _write_atomic(path: Path, text: str):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


# ── notes ───────────────────────────────────────────────────────────────────────────────────────

class NotesStore:
    """Free-text notes per control ("what is this routed for?"). Notes of controls with an ER-301
    route also end up as comments in er301_routes.py."""

    def __init__(self):
        self.path = data_dir() / "notes.json"
        self.notes: Dict[str, str] = {}
        try:
            self.notes = json.loads(self.path.read_text(encoding="utf-8"))
        except Exception:
            pass

    @staticmethod
    def ds(input_id: str) -> str: return "ds:" + input_id
    @staticmethod
    def fader(i: int) -> str: return f"16n:{i}"
    @staticmethod
    def pedal(inp: int, slot: int) -> str: return f"mpc:{inp}:{slot}"
    @staticmethod
    def ec4(s: int, g: int, e: int) -> str: return f"ec4:{s}:{g}:{e}"
    @staticmethod
    def ec4_push(s: int, g: int, e: int) -> str: return f"ec4p:{s}:{g}:{e}"
    @staticmethod
    def port(kind: str, port: int) -> str: return f"er:{kind}{port}"     # what an ER-301 port steers

    def note(self, key: str) -> Optional[str]:
        n = (self.notes.get(key) or "").strip()
        return n or None

    def raw(self, key: str) -> str:
        return self.notes.get(key, "")

    def set(self, key: str, value: str):
        if value.strip():
            self.notes[key] = value
        else:
            self.notes.pop(key, None)
        self.save()

    def replace(self, notes: Dict[str, str]):
        self.notes = dict(notes)
        self.save()

    def save(self):
        try:
            _write_atomic(self.path, json.dumps(self.notes, indent=2, sort_keys=True, ensure_ascii=False))
        except Exception:
            pass

    @staticmethod
    def one_line(s: str) -> str:
        return s.replace("\n", " ").replace("  ", " ")


# ── the last EC4 dump ───────────────────────────────────────────────────────────────────────────

def ec4_cache_path() -> Path:
    return data_dir() / "ec4-last.json"


def save_ec4_cache(board):
    d = {"ok": True, "version": board.version, "mem": board.mem, "setups": board.setups}
    try:
        _write_atomic(ec4_cache_path(), json.dumps(d))
    except Exception:
        pass


def load_ec4_cache(board) -> bool:
    p = ec4_cache_path()
    try:
        board.apply(json.loads(p.read_text(encoding="utf-8")))
        board.received_at = dt.datetime.fromtimestamp(p.stat().st_mtime)
    except Exception:
        return False
    return board.loaded


# ── configurations (.dsmconfig — the same JSON as the Mac app) ──────────────────────────────────

FORMAT = "dualsense-mapper-configuration"
PARTS = [("dualsense", "DualSense routing & button functions"), ("faderbank", "16n faders"),
         ("pedals", "MPC-20 pedals"), ("ec4", "EC4 setups"), ("notes", "Notes")]


def config_folder() -> Path:
    docs = Path.home() / "Documents"
    if not docs.exists() and sys.platform.startswith("linux"):
        try:
            import subprocess
            x = subprocess.run(["xdg-user-dir", "DOCUMENTS"], capture_output=True, text=True, timeout=2).stdout.strip()
            if x:
                docs = Path(x)
        except Exception:
            pass
    d = docs / "Kontrolleur"
    d.mkdir(parents=True, exist_ok=True)
    return d


def config_has(c: dict, part: str) -> bool:
    if part == "dualsense":
        return c.get("mapping") is not None
    if part == "faderbank":
        return c.get("faderbank") is not None
    if part == "pedals":
        return c.get("pedals") is not None
    if part == "ec4":
        return c.get("ec4") is not None
    return bool(c.get("notes"))


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).strftime("%Y-%m-%dT%H:%M:%SZ")


def parse_iso(s: str) -> Optional[dt.datetime]:
    try:
        return dt.datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=dt.timezone.utc).astimezone()
    except Exception:
        return None


def write_config(c: dict, path: Path):
    c = {k: v for k, v in c.items() if v is not None}
    _write_atomic(Path(path), json.dumps(c, indent=2, sort_keys=True, ensure_ascii=False))
    remember(Path(path))


def read_config(path: Path) -> dict:
    c = json.loads(Path(path).read_text(encoding="utf-8"))
    if c.get("format") != FORMAT:
        raise ValueError("This is not a Kontrolleur configuration.")
    remember(Path(path))
    return c


def remember(path: Path):
    s = settings()
    lst = [x for x in (s.value("recentConfigs") or []) if x != str(path)]
    lst.insert(0, str(path))
    s.setValue("recentConfigs", lst[:20])


def recent(limit: int = 10) -> List[Path]:
    urls = [Path(x) for x in (settings().value("recentConfigs") or [])]
    folders = [config_folder(), Path.home() / "Documents" / "DualSense Mapper"]
    for d in folders:
        if d.is_dir():
            urls += [p for p in d.iterdir() if p.suffix == ".dsmconfig"]
    seen, out = set(), []
    for u in urls:
        try:
            key = str(u.resolve())
        except Exception:
            continue
        if u.exists() and key not in seen:
            seen.add(key)
            out.append(u)
    out.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return out[:limit]
