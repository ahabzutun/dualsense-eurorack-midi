"""Running the helper scripts: on the SSH host over ssh, or on this computer (devices on a cable)."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal, Slot

# scripts/ of the repository this app lives in (the same files run on the SSH host);
# in the packed Mac app (PyInstaller) they travel inside the app
FROZEN = bool(getattr(sys, "frozen", False))
REPO = Path(__file__).resolve().parents[2]
SCRIPTS = Path(getattr(sys, "_MEIPASS", "")) / "scripts" if FROZEN else REPO / "scripts"


class ToolError(Exception):
    pass


def _expand(p: str) -> str:
    return os.path.expanduser(p.strip())


def run_ssh(host: str, identity: str, remote: str, stdin: Optional[str] = None, timeout: int = 300,
            connect_timeout: int = 6) -> str:
    args = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={connect_timeout}", "-o", "StrictHostKeyChecking=accept-new"]
    if identity.strip():
        args += ["-o", "IdentitiesOnly=yes", "-i", _expand(identity)]
    args += [host, remote]
    try:
        p = subprocess.run(args, input=stdin or "", capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise ToolError("ssh is not installed on this computer.")
    except subprocess.TimeoutExpired:
        raise ToolError(f"{host} did not answer within {timeout} s.")
    if p.returncode == 255:
        raise ToolError(p.stderr.strip() or f"ssh to {host} failed.")
    return p.stdout if p.stdout else p.stderr


def run_local(python: str, script: str, args, stdin: Optional[str] = None, timeout: int = 300) -> str:
    path = SCRIPTS / script
    if not path.exists():
        raise ToolError(f"{script} is missing in {SCRIPTS}")
    py = _expand(python) if python.strip() else ""
    if FROZEN and (not py or os.path.realpath(py) == os.path.realpath(sys.executable)):
        # the packed app runs the tool itself; started without --run-script it would open another window
        cmd = [sys.executable, "--run-script", str(path)]
    else:
        cmd = [py or sys.executable, str(path)]
    py = cmd[0]
    env = dict(os.environ, KONTROLLEUR_TOOL="1")    # a tool run never opens the app's window (see kontrolleur.py)
    try:
        p = subprocess.run([*cmd, *args], input=stdin or "", capture_output=True, text=True, timeout=timeout, env=env)
    except FileNotFoundError:
        raise ToolError(f"can't start {py}")
    except subprocess.TimeoutExpired:
        raise ToolError(f"{script} did not finish within {timeout} s.")
    return p.stdout if p.stdout else p.stderr


def last_json(text: str, what: str) -> dict:
    """The tools print one JSON object as their last '{' line."""
    for line in reversed(text.splitlines()):
        if line.startswith("{"):
            return json.loads(line)
    raise ToolError(f"no answer from {what}" if not text.strip() else text.strip()[-300:])


class Host:
    """Connection settings (from the globe popover)."""

    def __init__(self, host: str, identity: str, repo: str, python: str):
        self.host, self.identity, self.repo, self.python = host, identity, repo, python

    def remote(self, script: str, args: str, stdin: Optional[str] = None, timeout: int = 300) -> str:
        return run_ssh(self.host, self.identity, f"python3 {self.repo}/scripts/{script} {args}".rstrip(), stdin, timeout)

    def tool(self, on_ssh: bool, script: str, args, stdin: Optional[str] = None, timeout: int = 300) -> dict:
        if isinstance(args, str):
            args = args.split()
        if on_ssh:
            text = self.remote(script, " ".join(args), stdin, timeout)
        else:
            text = run_local(self.python, script, args, stdin, timeout)
        return last_json(text, script)

    # er301_routes_tool.py (always on the SSH host)
    def export(self) -> dict:
        return last_json(self.remote("er301_routes_tool.py", "export"), "the SSH host")

    def install(self, source: str) -> dict:
        return last_json(self.remote("er301_routes_tool.py", "install", source), "the SSH host")

    def install_controls(self, source: str) -> dict:
        return last_json(self.remote("er301_routes_tool.py", "install-controls", source), "the SSH host")


# ── background jobs: run in a thread, answer on the GUI thread ──────────────────────────────────

class _Signals(QObject):
    done = Signal(object, object)    # result, exception


class _Job(QRunnable):
    def __init__(self, fn: Callable):
        super().__init__()
        self.fn = fn
        self.signals = _Signals()

    @Slot()
    def run(self):
        try:
            r, e = self.fn(), None
        except Exception as ex:     # noqa: BLE001 — shown in the status bar
            r, e = None, ex
        try:
            self.signals.done.emit(r, e)
        except RuntimeError:        # the window was closed while this ran
            pass


_jobs = set()


def background(fn: Callable, done: Callable[[object, Optional[Exception]], None]):
    job = _Job(fn)
    _jobs.add(job)

    def finish(r, e):
        _jobs.discard(job)
        done(r, e)

    job.signals.done.connect(finish)
    job.setAutoDelete(False)
    QThreadPool.globalInstance().start(job)
