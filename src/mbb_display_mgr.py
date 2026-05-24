"""
Andrea Favero 20260524

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/mirrorballbot/

Code handling the openCV windows for:
    - ball color calibration (manual and auto)
    - ball tracking when testing the camera or the robot without the GUI




MIT License

Copyright (c) 2026 Andrea Favero

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


# ============================================================================
# DISPLAY MANAGER - Centralized window management for all threads
# ============================================================================

__version__ = "0.0.1"

import os
import cv2
import threading
from time import sleep


class DisplayManager:
    """
    Thread-safe display manager for OpenCV windows.
    This is used by the camera Class to visualize:
    - the ball colour calibration.
    - the ball detection by colour thresholding.
    """
    
    def __init__(self, verbose=False):
        self.verbove = verbose
        self.windows = {}
        self.lock = threading.Lock()
        self.calibration_mode = False  # prevents detection window interference
        self.window_ready = False
    
    
    
    def create_window(self, name, width, height, position):
        """Create a window safely."""
        with self.lock:
            if not self.window_ready:
                stderr_fd = os.dup(2)
                devnull = os.open(os.devnull, os.O_WRONLY)
                try:
                    # settings preventing garbage prints when setting the first CV2 window
                    os.dup2(devnull, 2)
                    
                    # create display window
                    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(name, width, height)
                    cv2.moveWindow(name, position[0], position[1])
                    self.windows[name] = True
                    if self.verbove:
                        print(f"[Display] Created: {name}")
                finally:
                    # reset settings preventing garbage prints when setting the first CV2 window
                    os.dup2(stderr_fd, 2)
                    os.close(devnull)
                    os.close(stderr_fd)
                    self.window_ready = True
            elif self.window_ready:
                # create display window
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(name, width, height)
                cv2.moveWindow(name, position[0], position[1])
                self.windows[name] = True
                if self.verbove:
                    print(f"[Display] Created: {name}")
                
    
    
    def show_image(self, name, image):
        """Show image in window."""
        with self.lock:
            if name in self.windows:
                cv2.imshow(name, image)
    
    
    
    def destroy_window(self, name):
        """Destroy a window."""
        with self.lock:
            if name in self.windows:
                cv2.destroyWindow(name)
                del self.windows[name]
                if self.verbove:
                    print(f"[Display] Destroyed: {name}")
    
    
    
    def destroy_all(self):
        """Destroy all windows."""
        with self.lock:
            cv2.destroyAllWindows()
            self.windows.clear()
    
    
    
    def poll_events(self, wait_ms=1):
        """Poll OpenCV events (call from main thread only)."""
        with self.lock:
            if self.windows:
                return cv2.waitKey(wait_ms) & 0xFF
        return -1
    
    
    
    def set_mouse_callback(self, window_name, callback):
        """Set mouse callback for a window."""
        with self.lock:
            cv2.setMouseCallback(window_name, callback)
    
    
    
    def get_trackbar_pos(self, window_name, trackbar_name):
        """Get trackbar position."""
        return cv2.getTrackbarPos(trackbar_name, window_name)
    
    
    
    def set_trackbar_pos(self, window_name, trackbar_name, value):
        """Set trackbar position."""
        cv2.setTrackbarPos(trackbar_name, window_name, value)
    
    
    
    def is_window_active(self, name):
        """Check if window exists."""
        with self.lock:
            return name in self.windows

