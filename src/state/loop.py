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

    def start_playback(self, midiout):
        """Start loop playback in background thread"""
        if not self.midi_buffer or self.playing:
            return False

        self.playing = True
        self.playback_stop_event.clear()
        self.playback_thread = threading.Thread(
            target=self._playback_loop,
            args=(midiout,),
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

    def _playback_loop(self, midiout):
        """Background thread that plays back the loop continuously"""
        while self.playing and not self.playback_stop_event.is_set():
            loop_start = time.time()

            with self.buffer_lock:
                buffer_copy = list(self.midi_buffer)  # Copy for thread safety

            for timestamp, midi_msg in buffer_copy:
                if not self.playing or self.playback_stop_event.is_set():
                    break

                # Wait until it's time to play this message
                target_time = loop_start + timestamp
                sleep_time = target_time - time.time()

                if sleep_time > 0:
                    # Use event.wait for interruptible sleep
                    if self.playback_stop_event.wait(timeout=sleep_time):
                        break  # Stop event was set

                # Send the MIDI message
                try:
                    midiout.send_message(midi_msg)
                except:
                    pass  # Ignore errors during playback

            # Wait for loop to complete before repeating
            if self.playing and not self.playback_stop_event.is_set():
                remaining_time = self.loop_duration - (time.time() - loop_start)
                if remaining_time > 0:
                    self.playback_stop_event.wait(timeout=remaining_time)

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
