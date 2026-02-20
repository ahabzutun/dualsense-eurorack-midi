"""
MIDI loop recording and playback for DualSense controller
Handles recording, playback, and clearing of MIDI loops per channel

FIX: _playback_loop was allocating 2-3 new list/tuple objects every 1ms
     (1000x/second), causing ~100-500MB/hr of heap growth. Fixed by:
     - Caching the buffer copy outside the while loop
     - Only refreshing the cache when recording is active
     - Reusing messages_in_window via .clear() instead of re-allocating
     - Moving the sort key lambda outside the loop
"""

import time
import threading


class LoopState:
    """Manages MIDI loop recording and playback for a single channel"""
    def __init__(self):
        self.recording = False
        self.playing = False
        self.midi_buffer = []  # List of (timestamp, midi_message) tuples
        self.record_start_time = 0
        self.loop_duration = 0
        self.playback_thread = None
        self.playback_stop_event = threading.Event()
        self.buffer_lock = threading.Lock()  # Thread-safe buffer access
        self.MAX_LOOP_DURATION = 60.0  # 60 seconds max

        # Quantization: None = off, 4/8/16/32 = subdivision
        # Raw timestamps are always preserved; snapping is applied at
        # playback time so you can toggle quantization mid-loop freely.
        self.quantize_subdivision = None

    def start_recording(self):
        """Start recording MIDI messages"""
        # FIX: Signal playback to stop BEFORE acquiring buffer_lock.
        # The old code called stop_playback() (which joins the thread) while
        # holding buffer_lock. The playback thread's first iteration tries to
        # acquire buffer_lock too → deadlock until the 0.5s join timeout.
        # Pattern matches clear_loop() which already solved this correctly.
        if self.playing:
            self.playing = False
            self.playback_stop_event.set()
            if self.playback_thread and self.playback_thread.is_alive():
                self.playback_thread.join(timeout=0.5)

        with self.buffer_lock:
            self.midi_buffer = []
            self.recording = True
            self.record_start_time = time.time()
            self.loop_duration = 0
            self.quantize_subdivision = None  # Reset quantization on new recording

    def stop_recording(self):
        """Stop recording and calculate loop duration"""
        with self.buffer_lock:
            if self.recording:
                self.recording = False
                self.loop_duration = time.time() - self.record_start_time
                # Only keep the loop if it has content and is under max duration
                if not self.midi_buffer or self.loop_duration > self.MAX_LOOP_DURATION:
                    self.midi_buffer = []
                    self.loop_duration = 0
                    return False  # Recording failed/discarded
                return True  # Recording successful
            return False

    def record_message(self, midi_message):
        """Record a MIDI message with timestamp"""
        if not self.recording:
            return

        with self.buffer_lock:
            current_time = time.time()
            relative_time = current_time - self.record_start_time

            # Check if we've exceeded max duration
            if relative_time > self.MAX_LOOP_DURATION:
                print(f"\n⚠️  Max loop duration ({self.MAX_LOOP_DURATION}s) reached - stopping recording")
                self.stop_recording()
                return

            self.midi_buffer.append((relative_time, midi_message))

    def start_playback(self, midiout, window_position_func=None, bpm_func=None):
        """
        Start loop playback in background thread

        Args:
            midiout: MIDI output interface
            window_position_func: Optional function returning scrub position (0.0-1.0)
            bpm_func: Optional function returning current BPM (for quantization)
        """
        if not self.midi_buffer or self.playing:
            return False

        self.playing = True
        self.playback_stop_event.clear()
        self.playback_thread = threading.Thread(
            target=self._playback_loop,
            args=(midiout, window_position_func, bpm_func),
            daemon=True
        )
        self.playback_thread.start()
        return True

    def stop_playback(self):
        """Stop loop playback"""
        if self.playing:
            self.playing = False
            self.playback_stop_event.set()
            if self.playback_thread:
                self.playback_thread.join(timeout=0.5)

    @staticmethod
    def _snap_to_grid(timestamp, bpm, subdivision, loop_duration):
        """
        Snap a timestamp (seconds) to the nearest grid point.

        grid_size = one quarter note * (4 / subdivision)
        e.g. subdivision=16 → grid every (60/bpm * 4/16) = 60/(bpm*4) seconds

        The snapped value is wrapped to stay inside [0, loop_duration).
        """
        if bpm <= 0:
            return timestamp
        quarter_note = 60.0 / bpm
        grid_size = quarter_note * (4.0 / subdivision)
        snapped = round(timestamp / grid_size) * grid_size
        return snapped % loop_duration

    def _playback_loop(self, midiout, window_position_func=None, bpm_func=None):
        """
        Playback loop with optional real-time quantization.
        Wrapped in try/finally so any unhandled exception resets state cleanly
        and surfaces in the logs instead of leaving playing=True forever.

        window_size is now fixed at 1.0 (full loop); touchpad Y controls
        quantize_subdivision instead. Touchpad X still scrubs position.

        Quantization is non-destructive — raw timestamps are preserved in
        midi_buffer and snapping is applied per-grain so it can be toggled
        freely during playback.
        """

        try:
            self._playback_loop_inner(midiout, window_position_func, bpm_func)
        except Exception as e:
            import traceback
            print(f"\n🔴 PLAYBACK LOOP CRASHED: {e}")
            traceback.print_exc()
        finally:
            self.playing = False
            self.playback_stop_event.set()
            print("⏹️  Playback thread exiting")

    def _playback_loop_inner(self, midiout, window_position_func=None, bpm_func=None):
        """Inner playback logic, called by _playback_loop wrapper."""
        last_window_pos = None
        POSITION_CHANGE_THRESHOLD = 0.05

        cached_buffer = []
        cached_duration = 0
        buffer_loaded = False
        messages_in_window = []
        _sort_key = lambda x: x[0]

        while self.playing and not self.playback_stop_event.is_set():

            if self.recording:
                with self.buffer_lock:
                    cached_buffer = list(self.midi_buffer)
                    cached_duration = self.loop_duration
            elif not buffer_loaded:
                with self.buffer_lock:
                    cached_buffer = list(self.midi_buffer)
                    cached_duration = self.loop_duration
                buffer_loaded = True

            if cached_duration == 0:
                time.sleep(0.01)
                continue

            # Window position from touchpad X (scrubbing); size fixed at 1.0
            window_pos = window_position_func() if window_position_func else 0.0
            window_size = 1.0  # always full loop — granular sizing removed

            window_start_time = window_pos * cached_duration
            window_length = cached_duration  # always the full loop
            window_end_time = window_start_time + window_length

            grain_duration = window_length

            # Snapshot quantization state for this grain so it stays
            # consistent even if the user changes it mid-grain.
            subdivision = self.quantize_subdivision
            bpm = bpm_func() if (bpm_func and subdivision) else 0

            messages_in_window.clear()

            for timestamp, midi_msg in cached_buffer:
                in_window = False

                if window_end_time <= cached_duration:
                    if window_start_time <= timestamp < window_end_time:
                        in_window = True
                else:
                    wrap_amount = window_end_time - cached_duration
                    if timestamp >= window_start_time or timestamp < wrap_amount:
                        in_window = True

                if in_window:
                    # Apply quantization to the original timestamp, then
                    # derive the grain-relative position from the snapped value.
                    if subdivision and bpm > 0:
                        snapped = self._snap_to_grid(timestamp, bpm, subdivision, cached_duration)
                    else:
                        snapped = timestamp

                    relative_pos = snapped - window_start_time
                    if relative_pos < 0:
                        relative_pos += cached_duration
                    messages_in_window.append((relative_pos, midi_msg))

            messages_in_window.sort(key=_sort_key)

            grain_start = time.time()

            for rel_time, midi_msg in messages_in_window:
                if not self.playing:
                    break

                current_pos = window_position_func() if window_position_func else 0.0
                if last_window_pos is not None and abs(current_pos - window_pos) > POSITION_CHANGE_THRESHOLD:
                    break

                if window_length > 0:
                    scaled_time = (rel_time / window_length) * grain_duration
                else:
                    scaled_time = 0

                target_time = grain_start + scaled_time
                sleep_time = target_time - time.time()

                if sleep_time > 0:
                    if self.playback_stop_event.wait(timeout=sleep_time):
                        break

                try:
                    midiout.send_message(midi_msg)
                except Exception:
                    pass

            last_window_pos = window_pos
            self.playback_stop_event.wait(timeout=0.001)

    def clear_loop(self):
        """Clear the loop buffer (safe version that prevents deadlock)"""
        # First, signal the playback thread to stop
        if self.playing:
            self.playing = False
            self.playback_stop_event.set()

        # Try to acquire lock with timeout to prevent permanent hang
        lock_acquired = self.buffer_lock.acquire(timeout=1.0)

        if lock_acquired:
            try:
                # Clear all loop data
                self.midi_buffer = []
                self.loop_duration = 0
                self.recording = False
                self.quantize_subdivision = None

                # Wait for playback thread to finish (non-blocking)
                if self.playback_thread and self.playback_thread.is_alive():
                    self.playback_thread.join(timeout=0.2)

            finally:
                self.buffer_lock.release()
            return True
        else:
            print("⚠️  Could not clear loop (timeout) - try again")
            return False
