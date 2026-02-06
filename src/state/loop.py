"""
MIDI loop recording and playback for DualSense controller
Handles recording, playback, and clearing of MIDI loops per channel
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

    def start_recording(self):
        """Start recording MIDI messages"""
        with self.buffer_lock:
            # If playing, stop playback first
            if self.playing:
                self.stop_playback()

            # Clear buffer and start fresh
            self.midi_buffer = []
            self.recording = True
            self.record_start_time = time.time()
            self.loop_duration = 0

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

    def start_playback(self, midiout, window_position_func=None, window_size_func=None):
        """
        Start loop playback in background thread

        Args:
            midiout: MIDI output interface
            window_position_func: Optional function returning window position (0.0-1.0)
            window_size_func: Optional function returning window size (0.0-1.0)
        """
        if not self.midi_buffer or self.playing:
            return False

        self.playing = True
        self.playback_stop_event.clear()
        self.playback_thread = threading.Thread(
            target=self._playback_loop,
            args=(midiout, window_position_func, window_size_func),
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

    def _playback_loop(self, midiout, window_position_func=None, window_size_func=None):
        """
        Granular scanning playback - window continuously retriggers
        Small windows = glitchy stuttering, large windows = smooth playback
        Moving touchpad X = immediate scrubbing through the recording
        """

        last_window_pos = None
        POSITION_CHANGE_THRESHOLD = 0.05  # 5% movement triggers new grain

        while self.playing and not self.playback_stop_event.is_set():
            with self.buffer_lock:
                buffer_copy = list(self.midi_buffer)
                loop_duration = self.loop_duration

            if loop_duration == 0:
                time.sleep(0.01)
                continue

            # Get current window parameters
            window_pos = window_position_func() if window_position_func else 0.0
            window_size = window_size_func() if window_size_func else 1.0

            # Calculate window boundaries in seconds
            window_start_time = window_pos * loop_duration
            window_length = window_size * loop_duration
            window_end_time = window_start_time + window_length

            # Minimum grain length to prevent CPU overload with tiny windows
            MIN_GRAIN_MS = 10
            grain_duration = max(window_length, MIN_GRAIN_MS / 1000.0)

            # Collect all messages in the current window
            messages_in_window = []
            for timestamp, midi_msg in buffer_copy:
                in_window = False

                if window_end_time <= loop_duration:
                    # Normal case: window doesn't wrap
                    if window_start_time <= timestamp < window_end_time:
                        in_window = True
                else:
                    # Wrapped case: window extends past loop end
                    wrap_amount = window_end_time - loop_duration
                    if timestamp >= window_start_time or timestamp < wrap_amount:
                        in_window = True

                if in_window:
                    # Calculate relative position within the window
                    relative_pos = timestamp - window_start_time
                    if relative_pos < 0:  # Handle wrap
                        relative_pos += loop_duration
                    messages_in_window.append((relative_pos, midi_msg))

            # Sort messages by their position in the window
            messages_in_window.sort(key=lambda x: x[0])

            # PLAY THE GRAIN - retrigger all messages in window
            grain_start = time.time()

            for rel_time, midi_msg in messages_in_window:
                if not self.playing:
                    break

                # Check if window position moved significantly during playback
                # If so, abort this grain and start a new one at the new position
                current_pos = window_position_func() if window_position_func else 0.0
                if last_window_pos is not None and abs(current_pos - window_pos) > POSITION_CHANGE_THRESHOLD:
                    # Window moved - retrigger immediately with new position
                    break

                # Calculate when to play this message within the grain
                # Scale the timing to fit the grain duration
                if window_length > 0:
                    scaled_time = (rel_time / window_length) * grain_duration
                else:
                    scaled_time = 0

                target_time = grain_start + scaled_time
                sleep_time = target_time - time.time()

                if sleep_time > 0:
                    if self.playback_stop_event.wait(timeout=sleep_time):
                        break

                # Send the MIDI message
                try:
                    midiout.send_message(midi_msg)
                except:
                    pass

            last_window_pos = window_pos

            # Tiny delay before retriggering grain (prevents CPU overload)
            # This is what creates the continuous scanning effect!
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

                # Wait for playback thread to finish (non-blocking)
                if self.playback_thread and self.playback_thread.is_alive():
                    self.playback_thread.join(timeout=0.2)

            finally:
                self.buffer_lock.release()
            return True
        else:
            print("⚠️  Could not clear loop (timeout) - try again")
            return False
