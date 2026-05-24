"""
Andrea Favero 20260524

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the camera and image analysis for ball tracking



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
# CAMERA CLASS for MIRRORBALLBOT by ANDREA FAVERO
# ============================================================================

__version__ = "0.0.1"

# limit the feedback (prints to Shell) from the camera module
from os import environ
environ["LIBCAMERA_LOG_LEVELS"] = "2"

# camera and image modules
from picamera2 import Picamera2
from libcamera import controls

# standard libraries
from time import time, sleep, ctime
from queue import Queue, Full
from json import load
from math import sqrt
from enum import Enum
from os import system
import numpy as np
import threading
import cv2
import sys

# MirrorBallBot custom modules
from mbb_display_mgr import DisplayManager
from mbb_settings_mgr import SettingsManager


class Camera:
    def __init__(self, gui_mode=False, verbose=False, display_manager=None):
        """Initialize low-latency camera.
        
        Args:
            gui_mode (bool): If True, camera won't add FPS on image
            verbose (bool): If True, informative prints to the Shell
            display_manager (object): It handles the cv2 windows
        """
        # verbose mode
        self.verbose = verbose
        
        if self.verbose:
            print(f"\nmbb_camera.py VERSION:  {__version__}\n")
        
        # GUI mode
        self.gui_mode = gui_mode  # when True, NO OpenCV windows are created
        
        # set display mode based on gui_mode
        if self.gui_mode:
            self.display_mode = "NONE"       # used with Tkinter
        else:
            self.display_mode = "DETECTION"  # standalone mode
        
        # display manager
        self.display_manager = display_manager
        
        # store settings manager reference
        self.settings_mgr = SettingsManager()
        
        # load settings if available
        if self.settings_mgr:
            settings = self.settings_mgr.get()
            # apply hardware settings
            self.width = settings.hardware.camera_resolution["width"]
            self.height = settings.hardware.camera_resolution["height"]
            self.ball_mm = settings.hardware.ball_diameter_mm
            self.platform_mm = settings.hardware.platform_diameter_mm
            self.focus_distance_m = settings.hardware.camera_focus_distance_m
        else:
            print("Warning: File settings_manager.py not found. Code exit")
            sys.exit(1)
        
        # set the openCV environment
        cv2.setUseOptimized(True)
        cv2.ocl.setUseOpenCL(True)
        
        if self.verbose:
            print(f"Loading the camera module\n")
        
        # camera object
        self.picam2 = Picamera2()
        
        # disable Wide Dynamic Range (WDR)
        system("v4l2-ctl --set-ctrl wide_dynamic_range=0 -d /dev/v4l-subdev0")
        
        # set the camera (resolution, fix AWB, fix AE, etc)
        ret = self.set_camera()
        
        # apply HSV settings from saved calibration
        self.lower_color = np.array(settings.vision.hsv_lower)
        self.upper_color = np.array(settings.vision.hsv_upper)
        
        # fallback ball hsv thresholds (my table football orange ball at evening)
        if self.lower_color is None or self.upper_color is None:
            self.lower_color = np.array([0, 123, 0])
            self.upper_color = np.array([16, 255, 203])
        
        # platforms and ball size/relation
        self.pixels_to_mm = self.platform_mm / self.width     # camera conversion from pixels to mm
        self.mm_to_pixels = self.width / self.platform_mm     # camera conversion from mm to pixels
        
        # ball size reference
        ball_min_mm = round(0.4 * self.ball_mm)               # min ball diameter
        ball_max_mm = round(1.7 * self.ball_mm)               # max ball diameter

        # min and max area of the ball, to filter out wrong contours
        self.min_area = int(ball_min_mm / self.platform_mm * ball_min_mm/self.platform_mm * self.width * self.height)
        self.max_area = int(ball_max_mm / self.platform_mm * ball_max_mm/self.platform_mm * self.width * self.height)
        
        # min and max radius the detect contours should be, for correct detection of the ball
        self.min_radius = int(sqrt(self.min_area/3.1415))
        self.max_radius = int(sqrt(self.max_area/3.1415))

        # flag for the ball detection and its radius
        self.prev_radius = None
        self.latest_ball_pos = (0, 0)
        self.ball_x = -9999
        self.ball_y = -9999
        
        # FPS tracking
        self.fps_check = 100
        self.fps = 0
        
        # skip frames on image show
        self.display_frame_skip = 10  # show every 10th frame
        self.frame_counter = 0
        
        # flag to interrupt the camera when updating its settings
        self.detection_paused = False

        # common frame management for different threads
        self.latest_annotated_frame = None
        
        # target visualization
        self.target_x = None
        self.target_y = None
        self.target_visible = False
        self.trajectory_points = []  # for storing path points
        self.trajectory_type = None  # 'square', 'circle', etc.
        
        # thread for capturing pictures
        self.window = False
        self.running = True
        self.latest_frame = None
        self.capture_thread = threading.Thread(target=self._capture_worker, daemon=True)
        self.capture_thread.start()
        
        # thread for tracking ball position
        self.position_queue = Queue(maxsize=50)
        self.detection_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self.detection_thread.start()
        
        print(f"\nCamera initialized: {self.width}x{self.height} (gui_mode={gui_mode})")
    
    
    
    def set_camera(self) -> bool:
        """
        Set the camera for low-latency mode:
        - minimum resolution
        - minimum color deep
        - fises AWB, AE and other parameter to the best value when in Auto mode
        """
        
        if self.verbose:
            print("Setting the camera")

        try:
            self.picam2.stop()
                
            # rounding resolution to multiples of 32 
            self.width = 32 * int(self.width / 32)
            self.height = 32 * int (self.height / 32)
            
            # apply the fix focus
            focus_dist = 1 / self.focus_distance_m if self.focus_distance_m > 0 else 3
            
            # set the camera mode
            mode = self.picam2.sensor_modes[0]
            sensor = {"output_size": mode["size"], "bit_depth": mode["bit_depth"]}
            main = {"format": "RGB888", "size": (self.width, self.height), "preserve_ar": False}
            # preserve_ar meanse preserving aspect ratio, not needed when setting width and height
            # as mutliple of 32. When False, the camera gets slightly faster
            
            # set the camea in auto-mode, to get the best settings according to the current light conditions
            self.config = self.picam2.create_video_configuration(
                sensor = sensor,                             # sets the sensor to lowest bit_depth mode
                main = main,                                 # determines the resolution for the frame
                buffer_count = 3,                            # frame buffer at the camera
                queue = True,                                # allows frames queueing
                controls={
                    "AeEnable": True,                        # auto Exposure
                    "AwbEnable": True,                       # auto White Balance
                    "AwbMode": 0,                            # any type of illuminance
                    "FrameRate": 120,                        # target to frame rate
                    "AfMode": controls.AfModeEnum.Manual,    # fix focus distance
                    "LensPosition": focus_dist,              # focus distance    
                    "NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Fast # fast noise reduction
                    },
                )
            
            self.picam2.configure(self.config)               # config the camera
            self.picam2.align_configuration(self.config)     # rounds frame size to the camera needs
            self.picam2.start(show_preview=False)            # starts the camera
            sleep(2)                                         # warm up time for the camera
            
            for i in range(10):                              # iterate for 10 times
                req = self.picam2.capture_request()          # grab a request
                arr = req.make_array("main")                 # image array
                meta = req.get_metadata()                    # metadata dict
                req.release()
            
            # set explicit controls (replace values with your measured averages)
            fixed_exposure = meta["ExposureTime"]            # microseconds
            fixed_analogue = meta["AnalogueGain"]            # analogue gain
            fixed_digital = meta["DigitalGain"]              # digital gain
            fixed_colour_gains = meta["ColourGains"]         # (red, blue) gains, format used by metadata
            
            print()
            if self.verbose:
                print("Based on the current light condition:")
                print("fixed_exposure set to:     ", fixed_exposure)
                print("fixed_analogue set to:     ", fixed_analogue)
                print("fixed_digital set to:      ", fixed_digital)
                print("fixed_colour_gains set to: ", fixed_colour_gains, "\n")

            
            # set the camera to manual mode (fixed settings) to lower the latency
            self.picam2.set_controls({
                    "ExposureTime": int(fixed_exposure),
                    "AnalogueGain": float(fixed_analogue),
                    "ColourGains": fixed_colour_gains,
                    "AeEnable": False,
                    "AwbEnable": False,
                    "NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Off, #Fast filter
                    })
            
            if self.verbose:
#                 print("\nCamera configuration:")
#                 print(self.picam2.camera_configuration())
                print()

            return True
        
        except Exception as e:
            print(f"Capture error: {e}")
            sleep(0.1)
            return False
    
    
    
    def _capture_worker(self):
        """
        Continuously grab the latest frame in a background thread.
        Calculate the FPS of rame capturing
        """

        # FPS
        frames = 0
        fps_check = self.fps_check
        fps_start_time = time()
        
        while self.running:

            try:
                if hasattr(self, 'detection_paused') and self.detection_paused:
                    sleep(0.01)
                    continue
                
                # retreieves the latest frame from the camera buffer
                self.latest_frame = self.picam2.capture_array("main")

                # calculate FPS
                frames += 1
                if frames >= fps_check:
                    current_time = time()
                    elapsed = current_time - fps_start_time
                    if elapsed > 0:
                        self.fps = round(frames / elapsed)
                    fps_start_time = current_time
                    frames = 0
                
                sleep(0.005)
                
            except Exception as e:
                print(f"Capture error: {e}")
                sleep(0.1)
    
    
    
    def _detect_loop(self):
        """Capture and detect loop - simplified, no window operations."""
        
        while self.running:
            
            # skip detection if paused 
            if hasattr(self, 'detection_paused') and self.detection_paused:
                sleep(0.01)
                continue
            
            try:
                # capture and detect
                self.ball_x, self.ball_y = self.find_ball(self.latest_frame)

                # put in queue
                try:
                    self.position_queue.put_nowait((self.ball_x, self.ball_y))
                except Full:
                    pass
                
                sleep(0.005)
                    
            except Exception as e:
                print(f"Ball detection error: {e}")
                sleep(0.01)
    
    
    
    def get_ball_position(self, block=False, timeout=0.01):
        """Get current ball position as integers."""
        if block:
            try:
                x, y = self.position_queue.get(timeout=timeout)
                return int(x), int(y)
            except queue.Empty:
                return self.ball_x, self.ball_y
        else:
            return self.ball_x, self.ball_y
    
    
    
    def get_fast_frame(self):
        """Return the most recent frame."""
        return self.latest_frame
    
    
    
    def get_frame(self):
        """Return a just taken frame."""
        try:
            return self.picam2.capture_array("main")
        except Exception as e:
            print(f"Capture error: {e}")
            sleep(0.01)
    
    
    
    def add_quadrants(self, image):
        """Plot lines to show the platform quadrants (unchanged from original)"""
        
        x_mid = self.width//2
        x_max = self.width-1
        y_mid = self.height//2
        y_max = self.height -1
        
        x_mid_1 = x_mid -1
        x_mid_2 = x_mid +1
        y_mid_1 = y_mid -1
        y_mid_2 = y_mid +1
        
        # white and thin lines
        cv2.line(image, ( 0, y_mid_1), ( x_max, y_mid_1), (255, 255, 255), 1)
        cv2.line(image, ( 0, y_mid_2), ( x_max, y_mid_2), (255, 255, 255), 1)
        cv2.line(image, ( x_mid_1, 0), ( x_mid_1, y_max), (255, 255, 255), 1)
        cv2.line(image, ( x_mid_2, 0), ( x_mid_2, y_max), (255, 255, 255), 1)
        
        # black and thin lines
        cv2.line(image, ( 0, y_mid), ( x_max, y_mid), (0, 0, 0), 1)
        cv2.line(image, ( x_mid, 0), ( x_mid, y_max), (0, 0, 0), 1)
            
#         cv2.circle(image, (x_mid, y_mid), int(0.5 * self.platform_mm * self.mm_to_pixels), (255, 0, 0), 3)
    
    
    
    def find_ball(self, image):
        """Ball position detection - modified to not show windows directly."""
        
        if image is None:
            return -9999, -9999

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            # store frame without ball
            self._handle_frame_display(image)
            return -9999, -9999

        largest = max(contours, key=cv2.contourArea)
        (x_img, y_img), radius = cv2.minEnclosingCircle(largest)
        area = cv2.contourArea(largest)
        
        if self.min_area < area < self.max_area:
            alpha = 0.02
            if self.prev_radius is None:
                filtered_radius = radius
            else:
                filtered_radius = alpha * radius + (1-alpha) * self.prev_radius
            self.prev_radius = filtered_radius
        
            # store frame with ball
            self._handle_frame_display(image, xy=(int(x_img), int(y_img)), radius=int(filtered_radius))
            return int(x_img - self.width / 2), int(self.height / 2 - y_img)
        
        else:
            # store frame without ball
            self._handle_frame_display(image)
            return -9999, -9999
    
    
    
    def _handle_frame_display(self, image, xy=None, radius=None):
        """
        Handle frame display - stores annotated frame, never creates windows directly.
        All window display is handled externally (by GUI or standalone test)
        """
        # always create and store the annotated frame
        annotated = self._create_annotated_frame(image, xy, radius)
        self.latest_annotated_frame = annotated

    
    
    
    def _create_annotated_frame(self, image, xy=None, radius=None):
        """Create an annotated copy of the frame without displaying it."""
        _image = image.copy()
        
        # plot the ball contour on the image
        if xy is not None and radius is not None:
            cv2.circle(_image, xy, radius, (0, 255, 0), 2)
            
        # draw trajectory if available
        if self.trajectory_type == 'circle' and len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                cv2.line(_image, self.trajectory_points[i], 
                        self.trajectory_points[i + 1], (255, 0, 0), 1)
            if len(self.trajectory_points) > 2:
                cv2.line(_image, self.trajectory_points[-1], 
                        self.trajectory_points[0], (255, 0, 0), 1)
                        
        elif self.trajectory_type == 'square' and len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                cv2.line(_image, self.trajectory_points[i], 
                        self.trajectory_points[i + 1], (255, 0, 0), 1)
        
        elif self.trajectory_type == 'infinity' and len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                cv2.line(_image, self.trajectory_points[i], 
                        self.trajectory_points[i + 1], (255, 0, 0), 2)
        
        elif self.trajectory_type == 'triangle' and len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                cv2.line(_image, self.trajectory_points[i], 
                        self.trajectory_points[i + 1], (255, 0, 0), 1)
            if len(self.trajectory_points) > 2:
                cv2.line(_image, self.trajectory_points[-1], 
                        self.trajectory_points[0], (255, 0, 0), 1)
        
        elif self.trajectory_type == 'line' and len(self.trajectory_points) > 1:
            cv2.line(_image, self.trajectory_points[0], 
                    self.trajectory_points[-1], (255, 0, 0), 1)
            
        elif self.trajectory_type == 'free' and len(self.trajectory_points) > 1:
            for i in range(len(self.trajectory_points) - 1):
                cv2.line(_image, self.trajectory_points[i], 
                        self.trajectory_points[i + 1], (255, 100, 0), 1)
        
        # draw current target if visible
        if self.target_visible and self.target_x is not None and self.target_y is not None:
            cv2.circle(_image, (self.target_x, self.target_y), 6, (255, 0, 0), -1)
            cv2.circle(_image, (self.target_x, self.target_y), 7, (255, 255, 255), 1)
        
        if self.trajectory_type is None:
            self.add_quadrants(_image)
        
        return _image
    
    
    
    def get_annotated_frame(self):
        """Return the most recent annotated frame (with drawings)."""
        if hasattr(self, 'latest_annotated_frame') and self.latest_annotated_frame is not None:
            frame = self.latest_annotated_frame.copy()
            
            # only add FPS overlay in standalone mode (not in GUI mode)
            if not self.gui_mode:
                cv2.rectangle(frame, (0, 0), (140, 30), (255, 255, 255), -1)
                cv2.putText(frame, f"FPS {self.fps}", (0, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
            
            return frame
        return None
    
    
    
    def save_calibration(self) -> bool:
        """
        Save current HSV calibration to centralized settings.
        Settings will be saved in mbb_settings.json file.
        """
        if self.settings_mgr:
            # convert numpy arrays to lists for JSON serialization
            lower_list = self.lower_color.tolist()
            upper_list = self.upper_color.tolist()
            
            # update settings via manager
            self.settings_mgr.update_hsv(lower_list, upper_list)
            
            if self.verbose:
                print(f"HSV calibration saved to mbb_settings.json: Lower={lower_list}, Upper={upper_list}")
            return True
        else:
            # fallback warning if no settings manager
            print("Warning: No settings manager available - calibration not saved")
            print("HSV values will be lost when the program restarts")
            return False
    
    
    
    def clean_up_cam(self):
        """Graceful shutdown."""
        self.running = False
        self.capture_thread.join(timeout=0.5)
        self.picam2.stop()
        self.picam2.close()
        if not self.gui_mode:
            cv2.destroyAllWindows()
    
    
    
    def set_target_position(self, x_mm, y_mm, trajectory_type=None):
        """Set target position in mm coordinates."""
        # convert mm to pixels for display
        self.target_x = int(self.width//2 + x_mm * self.mm_to_pixels)
        self.target_y = int(self.height//2 - y_mm * self.mm_to_pixels)  # Flip Y for display
        self.target_visible = True
        self.trajectory_type = trajectory_type
    
    
    
    def clear_target(self):
        """Remove target display."""
        self.target_visible = False
        self.target_x = None
        self.target_y = None
    
    
    
    def set_trajectory_points(self, points_mm, trajectory_type):
        """Set trajectory points for visualization.
        points_mm: list of (x, y) tuples in mm coordinates"""
        self.trajectory_type = trajectory_type
        self.trajectory_points = []
        for x_mm, y_mm in points_mm:
            # convert to pixel coordinates
            x_px = int(self.width//2 + x_mm * self.mm_to_pixels)
            y_px = int(self.height//2 - y_mm * self.mm_to_pixels)
            self.trajectory_points.append((x_px, y_px))
    
    
    
    def clear_trajectory(self):
        """Remove trajectory visualization."""
        self.trajectory_points = []
        self.trajectory_type = None
    
    
    
    def pause_detection(self):
        """Pause the ball detection thread during manual calibration."""
        self.detection_paused = True
        # clear the queue to avoid stale data
        while not self.position_queue.empty():
            try:
                self.position_queue.get_nowait()
            except:
                break
    
    
    
    def resume_detection(self):
        """Resume the ball detection thread after manual calibration."""
        self.detection_paused = False
        
    
    
    def manual_hsv_adjust(self, frame=None, watchdog_timeout=30):
        """
        Interactive HSV adjustment with trackbars and touch-friendly buttons.
        Includes watchdog timer that auto-quits if no interaction for specified seconds.
        
        Args:
            frame: Optional frame to use for calibration
            watchdog_timeout: Seconds of inactivity before auto-quit (default 30)
        
        Returns:
            (lower_color, upper_color) tuple
        """
        
        # store original values
        original_lower = self.lower_color.copy() if self.lower_color is not None else np.array([0, 0, 0])
        original_upper = self.upper_color.copy() if self.upper_color is not None else np.array([179, 255, 255])
        
        # pause detection during manual adjustment
        self.pause_detection()
        
        # set the camera back to auto then manual
        self.set_camera()
        
        # get a fresh frame
        if frame is None:
            for i in range(50):
                frame = self.latest_frame
                if frame is not None:
                    working_frame = cv2.resize(frame, (220, 220), interpolation=cv2.INTER_LINEAR)
                    break
                sleep(0.05)
        
        if frame is None:
            print("No frame available for HSV adjustment")
            self.resume_detection()
            return self.lower_color, self.upper_color
        
        working_frame = cv2.resize(frame, (220, 220), interpolation=cv2.INTER_LINEAR)
        
        # create windows via DisplayManager
        self.display_manager.create_window('HSV Calibration', 450, 430, (0, 30))
        self.display_manager.create_window('Result', 349, 430, (451, 30))
        
        # create trackbars
        cv2.createTrackbar('H Low',  'HSV Calibration', int(original_lower[0]), 179, lambda x: None)
        cv2.createTrackbar('H High', 'HSV Calibration', int(original_upper[0]), 179, lambda x: None)
        cv2.createTrackbar('S Low',  'HSV Calibration', int(original_lower[1]), 255, lambda x: None)
        cv2.createTrackbar('S High', 'HSV Calibration', int(original_upper[1]), 255, lambda x: None)
        cv2.createTrackbar('V Low',  'HSV Calibration', int(original_lower[2]), 255, lambda x: None)
        cv2.createTrackbar('V High', 'HSV Calibration', int(original_upper[2]), 255, lambda x: None)
        
        # store previous trackbar positions to detect actual changes
        prev_trackbar_positions = {
            'H Low': int(original_lower[0]),
            'H High': int(original_upper[0]),
            'S Low': int(original_lower[1]),
            'S High': int(original_upper[1]),
            'V Low': int(original_lower[2]),
            'V High': int(original_upper[2])
        }
        
        # button states
        button_state = {'save': False, 'quit': False, 'reset': False}
        button_rects = {}
        
        # watchdog timer variables
        watchdog_active = True
        last_user_interaction_time = time()
        WATCHDOG_TIMEOUT = watchdog_timeout
        WARNING_TIMEOUT = watchdog_timeout - 5
        warning_shown = False
        user_interacted = False
        
        # mouse callback
        def mouse_callback(event, x, y, flags, param):
            nonlocal last_user_interaction_time, user_interacted
            if event == cv2.EVENT_LBUTTONDOWN:
                # Check if click is on any button
                button_clicked = False
                for button_name, rect in button_rects.items():
                    if rect[0] <= x <= rect[0] + rect[2] and rect[1] <= y <= rect[1] + rect[3]:
                        button_clicked = True
                        button_state[button_name] = True
                        print(f"Button '{button_name}' clicked!")
                        break
                
                # ONLY reset watchdog if a button was actually clicked
                if button_clicked:
                    last_user_interaction_time = time()
                    user_interacted = True
        
        self.display_manager.set_mouse_callback('HSV Calibration', mouse_callback)
        
        print("\n" + "="*60)
        print("=== MANUAL HSV CALIBRATION ===")
        print("="*60)
        print("Adjust trackbars to isolate the ball (white in mask)")
        print("  [SAVE] - Save current settings")
        print("  [RESET] - Reset to original values")
        print("  [QUIT] - Quit without saving")
        print("  Press 's' to save, 'r' to reset, 'q' or ESC to quit")
        print(f"\n  ⏱️  Watchdog active: Auto-quit after {WATCHDOG_TIMEOUT} seconds of inactivity")
        print("="*60 + "\n")
        
        calibration_result = None
        
        try:
            while self.running:
                # check watchdog
                if watchdog_active and not user_interacted:
                    idle_time = time() - last_user_interaction_time
                    
                    if idle_time > WARNING_TIMEOUT and not warning_shown:
                        print(f"\n⚠️  Warning: No activity for {int(idle_time)} seconds!")
                        print(f"   Calibration will auto-quit in {WATCHDOG_TIMEOUT - int(idle_time)} seconds\n")
                        warning_shown = True
                    
                    if idle_time > WATCHDOG_TIMEOUT:
                        print(f"\n⏰ No activity for {WATCHDOG_TIMEOUT} seconds - auto-quitting without saving")
                        button_state['quit'] = True
                
                user_interacted = False
                
                # get fresh frame
                if self.latest_frame is not None:
                    working_frame = cv2.resize(self.latest_frame, (220, 220), interpolation=cv2.INTER_LINEAR)
                
                hsv = cv2.cvtColor(working_frame, cv2.COLOR_BGR2HSV)
                
                # get trackbar positions
                h_low = cv2.getTrackbarPos('H Low', 'HSV Calibration')
                h_high = cv2.getTrackbarPos('H High', 'HSV Calibration')
                s_low = cv2.getTrackbarPos('S Low', 'HSV Calibration')
                s_high = cv2.getTrackbarPos('S High', 'HSV Calibration')
                v_low = cv2.getTrackbarPos('V Low', 'HSV Calibration')
                v_high = cv2.getTrackbarPos('V High', 'HSV Calibration')
                
                # check if any trackbar position changed (user interaction)
                current_positions = {
                    'H Low': h_low, 'H High': h_high,
                    'S Low': s_low, 'S High': s_high,
                    'V Low': v_low, 'V High': v_high
                }
                for name, pos in current_positions.items():
                    if pos != prev_trackbar_positions[name]:
                        last_user_interaction_time = time()
                        warning_shown = False
                        prev_trackbar_positions[name] = pos
                
                lower = np.array([h_low, s_low, v_low])
                upper = np.array([h_high, s_high, v_high])
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(working_frame, working_frame, mask=mask)
                
                # Calculate available space
                h, w = working_frame.shape[:2]  # w = 220, h = 220

                # Use wider buttons to fill the space
                button_width = 180  # Much wider than 125
                button_margin = 10

                # Calculate total width needed
                total_width = w + button_width + 15 #30  # 220 + 180 + 30 = 430 (fits in 450 window)

                # Create canvas with exact needed width
                canvas = np.ones((h, total_width, 3), dtype=np.uint8) * 240

                # Center the image vertically (it's already full height)
                # Position image at the left with some padding
                image_padding = 5 #10
                canvas[0:h, image_padding:image_padding + w] = working_frame

                # Button X position (to the right of image)
                start_x = image_padding + w + 5 #10  # 10px gap between image and buttons

                # Button Y positions (start from top)
                start_y = 5
                button_height = 45

                # SAVE button (now wider)
                save_rect = (start_x, start_y, button_width, button_height)
                button_rects['save'] = save_rect
                cv2.rectangle(canvas, (start_x, start_y), (start_x + button_width, start_y + button_height), (0, 255, 0), -1)
                cv2.rectangle(canvas, (start_x, start_y), (start_x + button_width, start_y + button_height), (0, 100, 0), 2)
                # Center text in wider button
                text_x = start_x + (button_width // 2) - 35
                cv2.putText(canvas, "SAVE", (text_x, start_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

                # RESET button
                reset_y = start_y + button_height + button_margin
                reset_rect = (start_x, reset_y, button_width, button_height)
                button_rects['reset'] = reset_rect
                cv2.rectangle(canvas, (start_x, reset_y), (start_x + button_width, reset_y + button_height), (0, 255, 255), -1)
                cv2.rectangle(canvas, (start_x, reset_y), (start_x + button_width, reset_y + button_height), (0, 100, 100), 2)
                text_x = start_x + (button_width // 2) - 40
                cv2.putText(canvas, "RESET", (text_x, reset_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

                # QUIT button
                quit_y = reset_y + button_height + button_margin
                quit_rect = (start_x, quit_y, button_width, button_height)
                button_rects['quit'] = quit_rect
                cv2.rectangle(canvas, (start_x, quit_y), (start_x + button_width, quit_y + button_height), (0, 0, 255), -1)
                cv2.rectangle(canvas, (start_x, quit_y), (start_x + button_width, quit_y + button_height), (0, 0, 100), 2)
                text_x = start_x + (button_width // 2) - 35
                cv2.putText(canvas, "QUIT", (text_x, quit_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                # Timer bar (below QUIT button, using full button width)
                if watchdog_active:
                    idle_time = time() - last_user_interaction_time
                    time_left = max(0, WATCHDOG_TIMEOUT - int(idle_time))
                    
                    timer_y = quit_y + button_height + 42
                    bar_height = 10
                    bar_width = button_width - 6
                    bar_x = start_x + 3
                    
                    # Timer text above bar
                    if time_left > 10:
                        timer_color = (0, 255, 0)
                    elif time_left > 5:
                        timer_color = (0, 255, 255)
                    else:
                        timer_color = (0, 0, 255)
                    
                    timer_text = f"Quit: {time_left}s"
                    font_scale = 0.8
                    text_size = cv2.getTextSize(timer_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
                    text_x = start_x + (button_width - text_size[0]) // 2
                    cv2.putText(canvas, timer_text, (text_x, timer_y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), 2)
                    
                    # Timer bar background
                    cv2.rectangle(canvas, (bar_x, timer_y), (bar_x + bar_width, timer_y + bar_height), (100, 100, 100), -1)
                    # Timer bar foreground
                    bar_width_current = int(bar_width * (time_left / WATCHDOG_TIMEOUT))
                    cv2.rectangle(canvas, (bar_x, timer_y), (bar_x + bar_width_current, timer_y + bar_height), timer_color, -1)
                
                # show images
                self.display_manager.show_image('HSV Calibration', canvas)
                self.display_manager.show_image('Result', result)
                
                # poll events
                key = self.display_manager.poll_events(1)
                
                # reset watchdog on keyboard shortcuts
                if key != -1:
                    if key in [ord('s'), ord('r'), ord('q'), 27]:
                        last_user_interaction_time = time()
                        warning_shown = False
                
                # handle button states
                if button_state['save']:
                    self.lower_color = lower
                    self.upper_color = upper
                    if self.verbose:
                        print(f"\n[Manual Calibration] Saved: Lower HSV={lower}, Upper HSV={upper}")
                    
                    # save to centralized settings
                    if self.settings_mgr:
                        self.save_calibration()
                    calibration_result = (lower, upper)
                    break
                
                elif button_state['quit']:
                    self.lower_color = original_lower
                    self.upper_color = original_upper
                    if self.verbose:
                        print(f"\n[Manual Calibration] QUIT (no changes saved)")
                    calibration_result = (original_lower, original_upper)
                    break
                
                elif button_state['reset']:
                    cv2.setTrackbarPos('H Low', 'HSV Calibration', int(original_lower[0]))
                    cv2.setTrackbarPos('H High', 'HSV Calibration', int(original_upper[0]))
                    cv2.setTrackbarPos('S Low', 'HSV Calibration', int(original_lower[1]))
                    cv2.setTrackbarPos('S High', 'HSV Calibration', int(original_upper[1]))
                    cv2.setTrackbarPos('V Low', 'HSV Calibration', int(original_lower[2]))
                    cv2.setTrackbarPos('V High', 'HSV Calibration', int(original_upper[2]))
                    
                    # update stored positions
                    prev_trackbar_positions = {
                        'H Low': int(original_lower[0]),
                        'H High': int(original_upper[0]),
                        'S Low': int(original_lower[1]),
                        'S High': int(original_upper[1]),
                        'V Low': int(original_lower[2]),
                        'V High': int(original_upper[2])
                    }
                    
                    lower = np.array([original_lower[0], original_lower[1], original_lower[2]])
                    upper = np.array([original_upper[0], original_upper[1], original_upper[2]])
                    
                    last_user_interaction_time = time()
                    warning_shown = False
                    button_state['reset'] = False
                    
                    if self.verbose:
                        print(f"[Manual Calibration] RESET to original values")
                
                # keyboard shortcuts
                if key == ord('s'):
                    button_state['save'] = True
                elif key == ord('q') or key == 27:
                    button_state['quit'] = True
                elif key == ord('r'):
                    button_state['reset'] = True
        
        finally:
            # clean up windows
            self.display_manager.destroy_window('HSV Calibration')
            self.display_manager.destroy_window('Result')
            if self.verbose:
                print()
            sleep(0.5)
        
        # resume detection
        self.resume_detection()
        
        if calibration_result:
            return calibration_result[0], calibration_result[1]
        else:
            return self.lower_color, self.upper_color    
    
    
    
    
    
    def auto_calibrate_with_windows(self, pause_before=0, pause_after=1, show_windows=True):
        """
        Complete auto calibration with optional windows.
        Windows are shown only if display_manager exists AND gui_mode is False.
        
        Args:
            pause_before: Seconds to wait before starting calibration (default: 1.0)
            pause_after: Seconds to show results before closing windows (default: 1.5)
        
        Returns:
            (success, lower_color, upper_color)
        """
        
        if show_windows:
            print("\n[Auto Calibration] Starting with visual feedback...")
        else:
            print("\n[Auto Calibration] Running silently (no windows)...")
        
        # store original values in case of failure
        original_lower = self.lower_color.copy() if self.lower_color is not None else np.array([0, 0, 0])
        original_upper = self.upper_color.copy() if self.upper_color is not None else np.array([179, 255, 255])
        
        # pause detection during calibration
        self.pause_detection()
        
        window_names = None
        
        if show_windows:
            # create windows only if we're showing them
            window_detection = "Ball Detection (Auto)"
            window_result = "Ball Only (Auto)"
            window_names = (window_detection, window_result)
            
            self.display_manager.create_window(window_detection, 352, 352, (0, 30))
            self.display_manager.create_window(window_result, 352, 352, (360, 30))
            
            # show initial message
            if self.latest_frame is not None:
                init_frame = self.latest_frame.copy()
                self.display_manager.show_image(window_detection, init_frame)
                self.display_manager.show_image(window_result, init_frame)
                self.display_manager.poll_events(10)
            
            # wait before starting
            print(f"  Waiting {pause_before} seconds...")
            for _ in range(int(pause_before * 10)):
                if self.display_manager:
                    self.display_manager.poll_events(100)
                sleep(0.1)
        
        # run the actual calibration
        ret, lower, upper = self.auto_hsv_calib(display_windows=window_names if show_windows else None,
                                                pause_before=0,
                                                pause_after=pause_after if show_windows else 0
                                                )
        
        if ret:
            self.lower_color = lower
            self.upper_color = upper
            if self.verbose:
                print(f"[Auto Calibration] Complete! Lower={self.lower_color}, Upper={self.upper_color}\n")
            if self.settings_mgr:
                self.save_calibration()
        else:
            print("[Auto Calibration] Failed! Keeping original HSV values\n")
            # restore original values on failure
            self.lower_color = original_lower
            self.upper_color = original_upper
            
        
        # clean up windows if they were shown
        if show_windows and window_names:
            # give user time to see final result
            if ret:
                for _ in range(int(pause_after * 10)):
                    if self.display_manager:
                        self.display_manager.poll_events(100)
                    sleep(0.1)
            
            # destroy windows
            self.display_manager.destroy_window(window_names[0])
            self.display_manager.destroy_window(window_names[1])
        

        # resume detection
        self.resume_detection()
        
        return ret, self.lower_color, self.upper_color
    
    
    
    def _optimize_channel(self, hsv, center_x, center_y, roi_size, 
                          base_lower, base_upper, channel_idx, channel_name,
                          base_ball_contour, base_ball_area, min_val, max_val, step=2):
        """
        Optimize a single HSV channel - now extremely conservative.
        """
        best_lower = base_lower.copy().astype(np.uint8)
        best_upper = base_upper.copy().astype(np.uint8)
        best_score = -float('inf')
        best_ball_area = 0
        best_ball_contour = None
        
        # get ball's bounding box and center
        x, y, w, h = cv2.boundingRect(base_ball_contour)
        ball_center = (x + w//2, y + h//2)
        ball_radius = max(w, h) // 2
        
        original_range = base_upper[channel_idx] - base_lower[channel_idx]
        
        # for saturation, ONLY consider contracting, never expanding below 100
        if channel_name == 'Saturation':
            min_sat = max(100, base_lower[1])  # never go below 100
            max_expand = 0  # no expansion allowed for saturation lower bound
            test_deltas = range(-int(original_range), 1, step)  # only negative deltas (contracting)
        else:
            max_expand = int(original_range // 2)  # max 50% expansion
            test_deltas = range(-max_expand, max_expand + 1, step)
        
        if self.verbose:
            print(f"  Testing {channel_name} adjustments...")
        
        for delta in test_deltas:
            if delta == 0:
                continue
                
            test_lower = base_lower.copy().astype(np.uint8)
            test_upper = base_upper.copy().astype(np.uint8)
            
            # apply delta
            if channel_idx == 0:  # Hue
                new_low = int(base_lower[0]) + delta
                new_high = int(base_upper[0]) + delta
                test_lower[0] = np.clip(new_low, 0, 179)
                test_upper[0] = np.clip(new_high, 0, 179)
                
            elif channel_idx == 1:  # Saturation
                if delta < 0:       # contracting - raise lower bound
                    new_low = int(base_lower[1]) - delta  # delta negative, so subtract makes it larger
                    test_lower[1] = np.clip(new_low, 100, 255)  # never below 100
                    # keep upper bound same when contracting lower
                # never expand saturation lower bound!
                
            else:  # Value
                if delta < 0:  # contracting
                    new_low = int(base_lower[2]) - delta
                    new_high = int(base_upper[2]) + delta
                    test_lower[2] = np.clip(new_low, 20, 255)
                    test_upper[2] = np.clip(new_high, 20, 255)
                else:         # expanding (limited)
                    new_low = int(base_lower[2]) - delta
                    new_high = int(base_upper[2]) + delta
                    test_lower[2] = np.clip(new_low, 20, 255)
                    test_upper[2] = np.clip(new_high, 20, 250)  # cap at 250
            
            # ensure lower <= upper
            if test_lower[channel_idx] > test_upper[channel_idx]:
                continue
            
            # create mask and analyze
            test_mask = cv2.inRange(hsv, test_lower, test_upper)
            test_contours, _ = cv2.findContours(test_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not test_contours:
                continue
            
            # find ball contour
            ball_contour = None
            ball_area = 0
            
            for contour in test_contours:
                area = cv2.contourArea(contour)
                if area < base_ball_area * 0.7:  # must be at least 70% of original
                    continue
                    
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    dist = np.sqrt((cx - ball_center[0])**2 + (cy - ball_center[1])**2)
                    
                    if dist < ball_radius * 1.2:  # must be very close to original
                        ball_contour = contour
                        ball_area = area
                        break
            
            if ball_contour is None:
                continue
            
            # check for merged contours (background bleeding)
            ball_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(ball_mask, [ball_contour], -1, 255, -1)
            
            # count other contours
            other_count = len(test_contours) - 1
            
            # calculate score - VERY conservative
            ball_score = (ball_area / base_ball_area) * 1000
            
            # HEAVY penalties for ANY other contours
            other_penalty = other_count * 10000
            
            # penalty for changing range
            new_range = test_upper[channel_idx] - test_lower[channel_idx]
            range_penalty = abs(new_range - original_range) * 100
            
            # for Saturation, extra penalty for lowering the threshold
            if channel_name == 'Saturation' and test_lower[1] < base_lower[1]:
                range_penalty += 5000  # massive penalty for reducing saturation requirement
            
            score = ball_score - other_penalty - range_penalty
            
            if score > best_score:
                best_score = score
                best_lower = test_lower.copy()
                best_upper = test_upper.copy()
                best_ball_area = ball_area
                best_ball_contour = ball_contour
        
        return best_lower, best_upper, best_ball_contour, best_ball_area
    
    
    
    def auto_hsv_calib(self, frame=None, display_windows=None, pause_before=1.0, pause_after=1.0):
        """
        Auto calibration with optional display windows that update in real-time.
        
        Args:
            frame: Optional frame to use
            display_windows: Tuple of (detection_window_name, result_window_name) or None
            pause_before: Seconds to wait before starting calibration
            pause_after: Seconds to wait after calibration before destroying windows
        
        Returns:
            (success, lower_color, upper_color)
        """
        
        # store original values
        original_lower = self.lower_color.copy() if self.lower_color is not None else np.array([0, 0, 0])
        original_upper = self.upper_color.copy() if self.upper_color is not None else np.array([179, 255, 255])
        
        # pause detection
        self.pause_detection()
        
        # if display windows requested, show initial frames
        if display_windows and self.latest_frame is not None and self.display_manager:
            window_detection, window_result = display_windows
            
            # show initial frames
            if self.display_manager.is_window_active(window_detection):
                self.display_manager.show_image(window_detection, self.latest_frame)
                self.display_manager.show_image(window_result, self.latest_frame)
                self.display_manager.poll_events(10)
            
            # wait before starting (user can see windows)
            if self.verbose and pause_before > 0:
                print(f"  Waiting {pause_before} seconds before calibration...")
            for _ in range(int(pause_before * 10)):
                self.display_manager.poll_events(100)
                sleep(0.1)
        
        # set camera for calibration
        self.set_camera()
        
        # get frame
        if frame is None:
            for i in range(30):
                frame = self.latest_frame
                if frame is not None:
                    break
                sleep(0.1)
        
        if frame is None:
            print("No frame available")
            self.resume_detection()
            return False, original_lower, original_upper
        
        # ball size reference
        ball_min_mm = round(0.4 * self.ball_mm)
        ball_max_mm = round(1.7 * self.ball_mm)
        
        # min and max area of the ball
        self.min_area = int(ball_min_mm / self.platform_mm * ball_min_mm/self.platform_mm * self.width * self.height)
        self.max_area = int(ball_max_mm / self.platform_mm * ball_max_mm/self.platform_mm * self.width * self.height)
        
        h, w = frame.shape[:2]
        center_x, center_y = w//2, h//2
        
        # convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # extract center ROI
        roi_radius = min(w, h) // 4
        x1 = max(0, center_x - roi_radius)
        y1 = max(0, center_y - roi_radius)
        x2 = min(w, center_x + roi_radius)
        y2 = min(h, center_y + roi_radius)
        
        roi = hsv[y1:y2, x1:x2]
        roi_flat = roi.reshape(-1, 3)
        
        # find dominant colors using k-means
        Z = roi_flat.astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5
        _, labels, centers = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        
        labels = labels.flatten()

        # calculate color scores
        color_scores = []

        for idx in range(len(centers)):
            cluster_mask = (labels == idx)
            count = np.sum(cluster_mask)

            if count == 0:
                continue

            color = centers[idx]
            cluster_pixels = roi_flat[cluster_mask]

            h_val = int(color[0])
            s_val = int(color[1])
            v_val = int(color[2])
    
            if count > 10:
                h_cluster = cluster_pixels[:, 0]
                s_cluster = cluster_pixels[:, 1]
                v_cluster = cluster_pixels[:, 2]
                h_std = np.std(h_cluster)
                s_std = np.std(s_cluster)
                v_std = np.std(v_cluster)
                h_range_actual = np.max(h_cluster) - np.min(h_cluster)
                s_range_actual = np.max(s_cluster) - np.min(s_cluster)
                v_range_actual = np.max(v_cluster) - np.min(v_cluster)
            
            else:
                h_std = s_std = v_std = 5
                h_range_actual = s_range_actual = v_range_actual = 20
            
            colorfulness = (s_val / 255.0) * 100
            brightness = (v_val / 255.0) * 50
            population = (count / len(roi_flat)) * 100
            variation_penalty = (h_std + s_std + v_std) / 30
            score = (colorfulness * 3) + brightness + (population * 0.5) - variation_penalty
            
            color_scores.append({
                'score': score,
                'hsv': (h_val, s_val, v_val),
                'count': count,
                'percentage': (count / len(roi_flat)) * 100,
                'h_std': h_std,
                's_std': s_std,
                'v_std': v_std,
                'h_range': h_range_actual,
                's_range': s_range_actual,
                'v_range': v_range_actual
            })
        
        color_scores.sort(key=lambda x: x['score'], reverse=True)
        dominant_colors = color_scores[:3]
        
        best_ball = None
        best_color_idx = -1
        
        # test each dominant color
        for idx, cinfo in enumerate(dominant_colors):
            h_center, s_center, v_center = cinfo['hsv']
            
            h_range_base = max(15, int(cinfo['h_range'] * 1.2))
            s_range = max(40, int(cinfo['s_range'] * 1.2))
            v_range = max(40, int(cinfo['v_range'] * 1.2))
            s_range = min(80, s_range)
            v_range = min(80, v_range)
            
            if self.verbose:
                print(f"\n  Testing Color {idx+1}: HSV={h_center},{s_center},{v_center}")
            
            is_wraparound = (h_center < 20) or (h_center > 160)
            
            if is_wraparound:
                hue_ranges_to_try = [10, 15, 20, 25, 30, 35, 40]
                ball_found_for_color = False
                
                for test_h_range in hue_ranges_to_try:
                    if self.verbose:
                        print(f"Trying hue range ±{test_h_range}...")
                    
                    if h_center < 20:
                        test_lower = np.array([0, max(30, s_center - s_range), max(30, v_center - v_range)], dtype=np.uint8)
                        test_upper = np.array([min(20, h_center + test_h_range), min(255, s_center + s_range), min(255, v_center + v_range)], dtype=np.uint8)
                    else:
                        test_lower = np.array([max(140, h_center - test_h_range), max(30, s_center - s_range), max(30, v_center - v_range)], dtype=np.uint8)
                        test_upper = np.array([179, min(255, s_center + s_range), min(255, v_center + v_range)], dtype=np.uint8)
                    
                    mask = cv2.inRange(hsv, test_lower, test_upper)
                    center_mask = np.zeros_like(mask)
                    cv2.circle(center_mask, (center_x, center_y), roi_radius, 255, -1)
                    mask = cv2.bitwise_and(mask, center_mask)
                    
                    kernel = np.ones((3,3), np.uint8)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                    
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    # show current mask result
                    if display_windows and self.display_manager:
                        window_detection, window_result = display_windows
                        temp_result = cv2.bitwise_and(frame, frame, mask=mask)
                        display_frame = frame.copy()
                        self.display_manager.show_image(window_detection, display_frame)
                        self.display_manager.show_image(window_result, temp_result)
                        self.display_manager.poll_events(10)
                        sleep(0.25)
                    
                    if not contours:
                        continue
                    
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if not (self.min_area * 0.5 < area < self.max_area * 1.5):
                            continue
                        
                        shape_result = self.analyze_shape(contour)
                        
                        if not shape_result['is_ball']:
                            if self.verbose:
                                print(f"  Rejected: {', '.join(shape_result['reasons'])}")
                            continue
                        
                        M = cv2.moments(contour)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            dist_to_center = np.sqrt((cx - center_x)**2 + (cy - center_y)**2)
                            
                            if dist_to_center > roi_radius * 0.9:
                                continue
                                
                            if self.verbose:
                                print(f"  Found ball candidate with hue range ±{test_h_range}!")
                            
                            ball_found_for_color = True
                            
                            if best_ball is None or shape_result['metrics']['circularity'] > best_ball['circularity']:
                                best_ball = {
                                    'contour': contour,
                                    'area': area,
                                    'circularity': shape_result['metrics']['circularity'],
                                    'center': (cx, cy),
                                    'color_idx': idx,
                                    'color_hsv': (h_center, s_center, v_center),
                                    'thresholds': (test_lower.copy(), test_upper.copy()),
                                    'color_info': cinfo,
                                    'hue_range_used': test_h_range,
                                    'shape_metrics': shape_result['metrics']
                                }
                                best_color_idx = idx
                            break
                    
                    if ball_found_for_color:
                        break
                
                if not ball_found_for_color:
                    print(f"No ball found with any hue range")
            
            else:
                h_range = min(40, h_range_base)
                print(f"Using ranges: H±{h_range}, S±{s_range}, V±{v_range}")
                
                test_lower = np.array([max(0, h_center - h_range), max(30, s_center - s_range), max(30, v_center - v_range)], dtype=np.uint8)
                test_upper = np.array([min(179, h_center + h_range), min(255, s_center + s_range), min(255, v_center + v_range)], dtype=np.uint8)
                
                mask = cv2.inRange(hsv, test_lower, test_upper)
                center_mask = np.zeros_like(mask)
                cv2.circle(center_mask, (center_x, center_y), roi_radius, 255, -1)
                mask = cv2.bitwise_and(mask, center_mask)
                
                kernel = np.ones((3,3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # show current mask result
                if display_windows and self.display_manager:
                    window_detection, window_result = display_windows
                    temp_result = cv2.bitwise_and(frame, frame, mask=mask)
                    display_frame = frame.copy()
                    self.display_manager.show_image(window_detection, display_frame)
                    self.display_manager.show_image(window_result, temp_result)
                    self.display_manager.poll_events(10)
                    sleep(2)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if not contours:
                    if self.verbose:
                        print(f"No contours found")
                    continue
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if not (self.min_area * 0.5 < area < self.max_area * 1.5):
                        continue
                    
                    shape_result = self.analyze_shape(contour)
                    
                    if not shape_result['is_ball']:
                        if self.verbose:
                            print(f"  Rejected: {', '.join(shape_result['reasons'])}")
                        continue
                    
                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        dist_to_center = np.sqrt((cx - center_x)**2 + (cy - center_y)**2)
                        
                        if dist_to_center > roi_radius * 0.9:
                            continue
                        
                        if self.verbose:
                            print(f"  Found ball candidate!")
                        
                        if best_ball is None or shape_result['metrics']['circularity'] > best_ball['circularity']:
                            best_ball = {
                                'contour': contour,
                                'area': area,
                                'circularity': shape_result['metrics']['circularity'],
                                'center': (cx, cy),
                                'color_idx': idx,
                                'color_hsv': (h_center, s_center, v_center),
                                'thresholds': (test_lower.copy(), test_upper.copy()),
                                'color_info': cinfo,
                                'hue_range_used': h_range,
                                'shape_metrics': shape_result['metrics']
                            }
                            best_color_idx = idx
        
        if best_ball is None:
            print("\n  Could not find a suitable ball with any dominant color")
            self.resume_detection()
            return False, original_lower, original_upper
        
        if self.verbose:
            print(f"\n=== Best ball found with Color {best_color_idx+1} ===")
            print(f"  Color HSV: {best_ball['color_hsv']}")
        
        # refine thresholds
        ball_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(ball_mask, [best_ball['contour']], -1, 255, -1)
        ball_pixels = hsv[ball_mask == 255]
        
        if len(ball_pixels) < 50:
            if self.verbose:
                print("  Not enough ball pixels for refinement")
            best_lower, best_upper = best_ball['thresholds']
        else:
            h_vals = ball_pixels[:, 0]
            s_vals = ball_pixels[:, 1]
            v_vals = ball_pixels[:, 2]
            
            h_low = max(0,    int(np.percentile(h_vals, 2)) - 3)
            h_high = min(179, int(np.percentile(h_vals, 98)) + 3)
            s_low = max(40,   int(np.percentile(s_vals, 5)) - 15)
            s_high = min(255, int(np.percentile(s_vals, 95)) + 15)
            v_low = max(25,   int(np.percentile(v_vals, 5)) - 15)
            v_high = min(250, int(np.percentile(v_vals, 95)) + 15)
            
            if h_low < 20 and h_high > 160:
                left_count = np.sum(h_vals < 30)
                right_count = np.sum(h_vals > 150)
                if left_count > right_count:
                    h_low, h_high = 0, min(20, h_high)
                else:
                    h_low, h_high = max(140, h_low), 179
            
            best_lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
            best_upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
        
        # create final visualization
        debug = frame.copy()
        cv2.circle(debug, (center_x, center_y), roi_radius, (255, 255, 0), 2)
        cv2.drawContours(debug, [best_ball['contour']], -1, (0, 255, 0), 2)
        cv2.circle(debug, best_ball['center'], 4, (0, 0, 255), -1)
        
        h_val, s_val, v_val = best_ball['color_hsv']
        
        final_mask = cv2.inRange(hsv, best_lower, best_upper)
        final_result = cv2.bitwise_and(frame, frame, mask=final_mask)
        
        # show the calibrated result
        if display_windows and self.display_manager:
            window_detection, window_result = display_windows
            self.display_manager.show_image(window_detection, debug)
            self.display_manager.show_image(window_result, final_result)
            self.display_manager.poll_events(10)
        
        if self.verbose:
            print("\n=== FINAL CALIBRATION RESULTS ===")
            print(f"self.lower_color = np.array([{best_lower[0]}, {best_lower[1]}, {best_lower[2]}])")
            print(f"self.upper_color = np.array([{best_upper[0]}, {best_upper[1]}, {best_upper[2]}])")
        
        # apply new values
        self.lower_color = best_lower
        self.upper_color = best_upper
        
        # wait after calibration if requested
        if display_windows and pause_after > 0:
            if self.verbose:
                print(f"  Waiting {pause_after} seconds after calibration...")
            for _ in range(int(pause_after * 10)):
                if self.display_manager:
                    self.display_manager.poll_events(100)
                sleep(0.1)
        
        self.resume_detection()
        return True, self.lower_color, self.upper_color
    
    
    
    def analyze_shape(self, contour):
        """Comprehensive shape analysis to distinguish balls from squares."""
        
        # basic properties
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0 or area == 0:
            return {
                'is_ball': False,
                'reasons': ['zero area or perimeter']
            }
        
        # 1. circularity (standard measure)
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # 2. bounding rectangle aspect ratio
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 10
        
        # 3. enclosing circle fill ratio
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        circle_area = np.pi * radius * radius
        circle_fill_ratio = area / circle_area if circle_area > 0 else 0
        
        # 4. convexity
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        convexity = area / hull_area if hull_area > 0 else 0
        
        # 5. equivalent diameter consistency
        equiv_diameter = np.sqrt(4 * area / np.pi)
        diameter_ratio = (2 * radius) / equiv_diameter if equiv_diameter > 0 else 0
        
        # 6. moments-based eccentricity (how elongated)
        moments = cv2.moments(contour)
        if moments['mu20'] + moments['mu02'] > 0:
            eccentricity = np.sqrt(1 - (min(moments['mu20'], moments['mu02']) / 
                                       max(moments['mu20'], moments['mu02'])))
        else:
            eccentricity = 1
        
        # 7. contour smoothness (Hu moments - 1st invariant, scale invariant)
        hu = cv2.HuMoments(moments).flatten()
        hu1 = abs(hu[0])  # first Hu moment - higher for complex shapes
        
        # decision logic
        reasons = []
        is_ball = True
        
        # ball criteria
        if circularity < 0.5:
            is_ball = False
            reasons.append(f'circularity {circularity:.3f} < 0.5')
        
        if aspect_ratio > 2.0:
            is_ball = False
            reasons.append(f'aspect_ratio {aspect_ratio:.2f} > 1.7')
        
        if circle_fill_ratio < 0.45:
            is_ball = False
            reasons.append(f'circle_fill {circle_fill_ratio:.3f} < 0.7')


        if convexity < 0.75:
            is_ball = False
            reasons.append(f'convexity {convexity:.3f} < 0.7')
        
        if eccentricity > 1.0:
            is_ball = False
            reasons.append(f'eccentricity {eccentricity:.3f} > 0.4')
        
        # print debug info for suspicious cases
        if 0.65 < circularity < 0.85:
            
            if self.verbose:
                print(f"  Shape analysis:")
                print(f"Circularity: {circularity:.3f}")
                print(f"Aspect ratio: {aspect_ratio:.2f}")
                print(f"Circle fill: {circle_fill_ratio:.3f}")
                print(f"Convexity: {convexity:.3f}")
                print(f"Eccentricity: {eccentricity:.3f}")
                print(f"Diameter ratio: {diameter_ratio:.3f}")
                print(f"Hu1: {hu1:.3f}")
        
        return {
            'is_ball': is_ball,
            'reasons': reasons,
            'metrics': {
                'circularity': circularity,
                'aspect_ratio': aspect_ratio,
                'circle_fill_ratio': circle_fill_ratio,
                'convexity': convexity,
                'eccentricity': eccentricity,
                'diameter_ratio': diameter_ratio,
                'hu1': hu1
            }
        }




# ============================================================================
# STAND-ALONE TEST
# ============================================================================

if __name__ == "__main__":
    
    import os          # used to prevent garbage prints at the first CV2 windows creation
    
    verbose = False    # set True for debug, when standalone testing
    
    # create display manager
    display_mgr = DisplayManager(verbose=verbose)
    
    # create camera (gui_mode=False for standalone - WILL show windows)
    cam = Camera(verbose=verbose, display_manager=display_mgr, gui_mode=False)
    
    print("\n" + "="*60)
    print("CAMERA TEST MODE")
    print("="*60)
    
    
    stderr_fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)

    try:
        # settings preventing garbage prints when setting the first CV2 window
        os.dup2(devnull, 2)
        
        # create main display window
        main_window = "Ball Balance Platform"
        cv2.namedWindow(main_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(main_window, 352, 352)
        cv2.moveWindow(main_window, 0, 30)
        cv2.waitKey(1)

    finally:
        # reset settings preventing garbage prints when setting the first CV2 window
        os.dup2(stderr_fd, 2)
        os.close(devnull)
        os.close(stderr_fd)
        
    
    print("\n"*2)
    print("="*60)
    print("Controls:")
    print("  'a' - Auto calibration   ( ball HSV )")
    print("  'm' - Manual calibration ( ball HSV )")
    print("  'q' - Quit")
    print()
    print("Note: The windows must be selected to accept the choice")
    print("="*60 + "\n")
    
    try:
        while cam.running:
            frame = cam.get_annotated_frame()
            if frame is not None:
                cv2.imshow(main_window, frame)
            
            key = cv2.waitKey(33) & 0xFF
            
            if key == ord('a'):
                # close main window temporarily
                cv2.destroyWindow(main_window)
                cv2.waitKey(100)
                
                # run auto calibration (handles its own windows)
                ret, cam.lower_color, cam.upper_color = cam.auto_calibrate_with_windows(
                    pause_before= 0,
                    pause_after= 1.0,
                    show_windows = True
                )
                
                # recreate main window
                cv2.namedWindow(main_window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(main_window, 352, 352)
                cv2.moveWindow(main_window, 0, 30)
                cv2.waitKey(100)
            
            elif key == ord('m'):
                cv2.destroyWindow(main_window)
                cv2.waitKey(100)
                
                cam.lower_color, cam.upper_color = cam.manual_hsv_adjust()
                
                cv2.namedWindow(main_window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(main_window, 352, 352)
                cv2.moveWindow(main_window, 0, 30)
                cv2.waitKey(100)
            
            elif key == ord('q'):
                break
            
            sleep(0.005)
    
    
    finally:
        display_mgr.destroy_all()
        cv2.destroyAllWindows()
        cam.clean_up_cam()