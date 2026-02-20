"""
Clock source for sequencer system.
Handles both internal BPM clock and external MIDI clock sync from NerdSEQ.
"""

import threading
import time
from typing import Optional, Callable, List
import rtmidi

class ClockSource:
    """
    Manages timing for all sequencers.
    Syncs to external MIDI clock when available, otherwise uses internal BPM.
    """

    # MIDI Clock constants
    CLOCK_MSG = 0xF8        # MIDI timing clock (24 ppqn)
    START_MSG = 0xFA        # MIDI start
    CONTINUE_MSG = 0xFB     # MIDI continue
    STOP_MSG = 0xFC         # MIDI stop

    def __init__(self, internal_bpm: int = 120):
        self.internal_bpm = internal_bpm
        self.is_running = False
        self.is_playing = False  # For start/stop control

        # Clock sync state
        self.use_external_clock = False
        self.last_clock_time = 0
        self.clock_interval = 0
        self.external_bpm = 0
        self.clock_count = 0  # Track 24ppqn pulses

        # Thread management
        self.clock_thread: Optional[threading.Thread] = None
        self.midi_listener_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Subscriber management
        self.subscribers: List[Callable[[int], None]] = []  # Functions to call on clock tick
        self.on_sync_change: Optional[Callable[[bool], None]] = None  # Called when sync state changes

        # MIDI input for clock sync
        self.midi_in = None
        self._connected_port_name = None   # Track which port we have open
        self._hotplug_thread: Optional[threading.Thread] = None
        self._setup_midi_clock_listener()

    def _find_nerdseq_port(self):
        """Return (index, name) of the NerdSEQ MIDI port, or (None, None)."""
        try:
            probe = rtmidi.MidiIn()
            ports = probe.get_ports()
            for idx, name in enumerate(ports):
                if 'nerdseq' in name.lower():
                    return idx, name
            return None, None
        except Exception:
            return None, None

    def _setup_midi_clock_listener(self):
        """Open the NerdSEQ MIDI port and start receiving clock. Safe to call
        multiple times — does nothing if already connected to the same port."""
        idx, name = self._find_nerdseq_port()
        if idx is None:
            if self._connected_port_name is not None:
                # Was connected, now gone — clean up
                self._teardown_midi_listener()
                print("[ClockSource] NerdSEQ disconnected, using internal clock")
            return  # Nothing to open

        if self._connected_port_name == name:
            return  # Already connected to this port, nothing to do

        # Close existing connection before opening new one
        self._teardown_midi_listener()

        try:
            self.midi_in = rtmidi.MidiIn()
            self.midi_in.open_port(idx)
            self.midi_in.set_callback(self._midi_clock_callback)
            self._connected_port_name = name
            print(f"[ClockSource] ✅ Listening for MIDI clock on: {name}")
        except Exception as e:
            print(f"[ClockSource] Could not open NerdSEQ port: {e}")
            self.midi_in = None
            self._connected_port_name = None

    def _teardown_midi_listener(self):
        """Close the current MIDI input cleanly."""
        was_synced = self.use_external_clock
        if self.midi_in is not None:
            try:
                self.midi_in.cancel_callback()
                self.midi_in.close_port()
            except Exception:
                pass
            self.midi_in = None
        self._connected_port_name = None
        with self.lock:
            self.use_external_clock = False
        if was_synced and self.on_sync_change:
            self.on_sync_change(False)

    def _hotplug_monitor(self):
        """Background thread: scan for NerdSEQ every 5 seconds and
        connect/disconnect as it appears or disappears."""
        while self.is_running:
            self._setup_midi_clock_listener()
            time.sleep(5.0)

    def _midi_clock_callback(self, message, data):
        """Handle incoming MIDI clock messages"""
        msg = message[0]
        msg_type = msg[0]

        current_time = time.perf_counter()

        if msg_type == self.CLOCK_MSG:
            # MIDI clock tick (24 ppqn)
            with self.lock:
                just_synced = not self.use_external_clock  # First clock pulse?

                if self.last_clock_time > 0:
                    interval = current_time - self.last_clock_time
                    if self.clock_interval > 0:
                        self.clock_interval = (self.clock_interval * 0.8) + (interval * 0.2)
                    else:
                        self.clock_interval = interval

                    if self.clock_interval > 0:
                        self.external_bpm = int(60.0 / (self.clock_interval * 24))

                self.last_clock_time = current_time
                self.clock_count += 1
                self.use_external_clock = True

                if just_synced:
                    print(f"[ClockSource] 🟢 External clock LOCKED — NerdSEQ BPM will appear shortly")
                    if self.on_sync_change:
                        self.on_sync_change(True)

                # Log BPM once we have a stable reading (after first 24 pulses)
                if self.clock_count == 24:
                    print(f"[ClockSource] 🎵 NerdSEQ BPM: {self.external_bpm}")

                # Notify subscribers on quarter note boundaries (every 24 clocks)
                if self.clock_count % 24 == 0:
                    self._notify_subscribers(self.clock_count // 24)

        elif msg_type == self.START_MSG:
            with self.lock:
                self.is_playing = True
                self.clock_count = 0
                print("[ClockSource] External MIDI Start received")

        elif msg_type == self.CONTINUE_MSG:
            with self.lock:
                self.is_playing = True
                print("[ClockSource] External MIDI Continue received")

        elif msg_type == self.STOP_MSG:
            with self.lock:
                self.is_playing = False
                print("[ClockSource] External MIDI Stop received")

    def _internal_clock_loop(self):
        """Generate internal clock ticks based on BPM"""
        quarter_note_interval = 60.0 / self.internal_bpm  # Seconds per quarter note
        tick_interval = quarter_note_interval / 24  # 24ppqn

        next_tick = time.perf_counter()
        tick_count = 0

        while self.is_running:
            current_time = time.perf_counter()

            # Check if we should use external clock
            with self.lock:
                using_external = self.use_external_clock
                # If we haven't received clock in 2 seconds, fall back to internal
                if using_external and (current_time - self.last_clock_time) > 2.0:
                    self.use_external_clock = False
                    using_external = False
                    print("[ClockSource] External clock timeout, switching to internal")
                    if self.on_sync_change:
                        self.on_sync_change(False)

            if not using_external and self.is_playing:
                # Generate internal clock
                if current_time >= next_tick:
                    tick_count += 1

                    # Notify on quarter note boundaries
                    if tick_count % 24 == 0:
                        self._notify_subscribers(tick_count // 24)

                    # Calculate next tick time
                    next_tick += tick_interval

                    # Prevent drift
                    if next_tick < current_time:
                        next_tick = current_time + tick_interval

            # Small sleep to prevent CPU spinning
            time.sleep(0.001)

    def _notify_subscribers(self, beat_count: int):
        """Notify all subscribers of a clock tick"""
        for subscriber in self.subscribers:
            try:
                subscriber(beat_count)
            except Exception as e:
                print(f"[ClockSource] Error notifying subscriber: {e}")
                import traceback
                traceback.print_exc()

    def subscribe(self, callback: Callable[[int], None]):
        """
        Subscribe to clock ticks.
        Callback will be called with beat count (quarter notes) on each tick.
        """
        self.subscribers.append(callback)
        print(f"[ClockSource] Subscriber added, total subscribers: {len(self.subscribers)}")

    def unsubscribe(self, callback: Callable[[int], None]):
        """Unsubscribe from clock ticks"""
        if callback in self.subscribers:
            self.subscribers.remove(callback)

    def start(self):
        """Start the clock source"""
        if self.is_running:
            return

        self.is_running = True
        self.is_playing = True

        # Start internal clock thread
        self.clock_thread = threading.Thread(target=self._internal_clock_loop, daemon=True)
        self.clock_thread.start()

        # Start NerdSEQ hot-plug monitor
        self._hotplug_thread = threading.Thread(target=self._hotplug_monitor, daemon=True)
        self._hotplug_thread.start()

        print(f"[ClockSource] Started - Internal BPM: {self.internal_bpm}")

    def stop(self):
        """Stop the clock source"""
        self.is_running = False
        self.is_playing = False

        if self.clock_thread:
            self.clock_thread.join(timeout=2.0)

        self._teardown_midi_listener()

        print("[ClockSource] Stopped")

    def play(self):
        """Start playback (without restarting threads)"""
        with self.lock:
            self.is_playing = True
            self.clock_count = 0
            print("[ClockSource] Play")

    def pause(self):
        """Pause playback"""
        with self.lock:
            self.is_playing = False
            print("[ClockSource] Pause")

    def set_bpm(self, bpm: int):
        """Set internal BPM"""
        self.internal_bpm = max(20, min(300, bpm))  # Clamp between 20-300
        print(f"[ClockSource] BPM set to {self.internal_bpm}")

    def get_current_bpm(self) -> int:
        """Get current effective BPM (external or internal)"""
        with self.lock:
            if self.use_external_clock and self.external_bpm > 0:
                return self.external_bpm
            return self.internal_bpm

    def is_synced_external(self) -> bool:
        """Check if currently synced to external clock"""
        with self.lock:
            return self.use_external_clock
