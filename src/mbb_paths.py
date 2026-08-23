"""
Andrea Favero 20260822

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the Paths execution module for MirrorBallBot.
Contains all path implementations: square, circle, infinity, triangle, line, free.


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
# PATHS CLASS for MIRRORBALLBOT by ANDREA FAVERO
# ============================================================================

__version__ = "0.0.2"

import numpy as np
from time import time, sleep
import threading
from math import cos, sin, pi, sqrt, radians, degrees

# NOTE: Do NOT import from mbb_robot here to avoid circular imports.
# The PathExecutor class receives robot and controller references via __init__.

class PathExecutor:
    """Executes various paths using the robot's control system."""
    
    def __init__(self, robot, verbose=False):
        """
        Initialize path executor with robot reference.
        
        Args:
            robot: BallBalancingSystem instance
        """
        self.robot = robot
        self.verbose = verbose
        self.camera = robot.camera
        self.controller = robot.controller
        
        # Free path recording state
        self._recorded_points = []
        self._recording_active = False
        self._recording_start_time = 0
    
    # ==================== SQUARE PATH ====================
    
    def square_path(self, side_mm: int, repeats=1, 
                    tolerance_mm: int = 40, settle_time_ms: int = 400,
                    stop_event: threading.Event = None, completion_callback=None):
        """
        Execute a square path by setting sequential target positions.
        Advances to next vertex only after ball is within tolerance for settle_time.
        
        Args:
            side_mm: Side length of square in mm
            repeats: Number of squares to complete
            tolerance_mm: Distance from target to consider "arrived" (mm)
            settle_time_ms: Time ball must stay within tolerance before advancing (ms)
            stop_event: Event to signal path stop
        """
        
        side_mm = min(140, max(side_mm, 2 * self.camera.ball_mm))
        
        if self.verbose:
            print("\n" + "="*60)
            print("SQUARE PATH FOLLOWING - POSITION BASED")
            print("="*60)
            print(f"Square side: {side_mm} mm")
            print(f"Arrival tolerance: ±{tolerance_mm} mm")
            print(f"Settle time: {settle_time_ms} ms")
            print(f"Repeats: {repeats}")
            print("="*60)
        
        # calculate square vertices (centered around 0,0)
        half_side = side_mm // 2
        
        # define vertices in order
        vertices = [(-half_side, -half_side),       # bottom-left
                    (half_side, -half_side),        # bottom-right
                    (half_side, half_side),         # top-right
                    (-half_side, half_side)]        # top-left
        
        # generate full path (including repeats)
        full_path = [(0, 0)]                        # start at center
        for _ in range(repeats):                    # iteration for number of repeats
            full_path.extend(vertices)              # adds the verteces
        full_path.append((-half_side, -half_side))  # return to first corner
        full_path.append((0, 0))                    # return to center
        
        # set trajectory for visualization
        self.camera.set_trajectory_points(vertices + [vertices[0]], 'square')
        
        # execute using the generic vertex path method
        self._execute_vertex_path(full_path, tolerance_mm, settle_time_ms, 
                                  stop_event, 'square', completion_callback)
    
    
    
    # ==================== CIRCLE PATH ====================
    
    def circle_path(self, radius_mm: int, repeats=1, direction='cw', speed_factor: float = 1.0,
                    stop_event: threading.Event = None, completion_callback=None):
        """
        Execute a circular path with orbit visualization.
        
        Args:
            radius_mm: Radius of the circle in millimeters
            repeats: Number of circles to complete
            direction: 'cw' for clockwise, 'ccw' for counter-clockwise
            speed_factor: Speed multiplier (0.5 = half speed, 2.0 = double speed)
            stop_event: Event to signal path stop
        """
        
        radius_mm = min(100, max(radius_mm, self.camera.ball_mm))
        
        # Base speed: 150 mm/s (works well across different radii)
        BASE_SPEED_MM_PER_S = 150.0
        
        # Apply speed factor
        peripheral_speed = BASE_SPEED_MM_PER_S * speed_factor
        
        # calculate angular speed from peripheral speed and radius
        # v = ω * r  =>  ω = v / r
        omega = peripheral_speed / radius_mm  # rad/s
        
        period_sec = (2 * np.pi * radius_mm) / peripheral_speed
        
        if self.verbose:
            print("\n" + "="*60)
            print("CIRCLE PATH FOLLOWING - ORBIT VISUALIZATION")
            print("="*60)
            print(f"Circle radius: {radius_mm} mm")
            print(f"Speed factor: {speed_factor:.1f}x")
            print(f"Peripheral speed: {peripheral_speed:.1f} mm/s")
            print(f"Angular speed: {omega:.3f} rad/s ({omega*180/np.pi:.1f}°/s)")
            print(f"Circumference: {2*np.pi*radius_mm:.1f} mm")
            print(f"Direction: {direction}")
            print(f"Repeats: {repeats}")
            print("="*60)
        
        # direction multiplier
        dir_mult = -1 if direction.lower() == 'cw' else 1
        
        # generate circle points for visualization (orbit)
        num_points = 36  # one point every 10 degrees
        circle_points = []
        for i in range(num_points + 1):
            angle = 2 * np.pi * i / num_points
            x = radius_mm * np.cos(angle)
            y = radius_mm * np.sin(angle)
            circle_points.append((x, y))
        
        def param_generator(elapsed):
            # use radial speed and time to determine theta
            theta = omega * elapsed * dir_mult
            x = radius_mm * np.cos(theta)
            y = radius_mm * np.sin(theta)
            return x, y
        
        self._execute_continuous_path(param_generator=param_generator,
                                      period_sec=period_sec,
                                      repeats=repeats,
                                      stop_event=stop_event,
                                      trajectory_type='circle',
                                      trajectory_points=circle_points,
                                      completion_callback=completion_callback
                                      )
        
    
    
    # ==================== INFINITY PATH ====================
    
    def infinity_path(self, size_mm: int, speed_factor: float = 1.0,
                      repeats: int = 1, stretch: float = 1.5,
                      stop_event: threading.Event = None,
                      completion_callback=None):
        """
        Execute an infinity symbol (lemniscate) path.
        
        The path follows a figure-8 pattern where the ball traces an infinity symbol.
        
        Args:
            size_mm: The overall size of the infinity symbol in mm (width and height)
            speed_factor: Speed multiplier (0.5 = slower, 2.0 = faster, 1.0 = normal)
            repeats: Number of times to complete the full infinity pattern
            stretch: Vertical stretch factor (>1 = taller, <1 = shorter)
            stop_event: Event to signal path stop
        """
        
        # limit size to reasonable bounds
        size_mm = min(160, max(130, size_mm))
        
        # base angular speed (radians per second)
        # complete one full infinity loop in about 4 seconds at speed factor 1.0
        BASE_ANGULAR_SPEED = 1.57  # rad/s
        
        angular_speed = BASE_ANGULAR_SPEED * speed_factor
        period_sec = (2 * np.pi) / angular_speed
        
        if self.verbose:
            print("\n" + "="*60)
            print("INFINITY PATH FOLLOWING")
            print("="*60)
            print(f"Path size: {size_mm} mm")
            print(f"Vertical stretch: {stretch}x")
            print(f"Speed factor: {speed_factor}x")
            print(f"Repeats: {repeats}")
            print("="*60)
        
        # check if auto-balance is running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(2)
        
        # parametric equations for infinity symbol (lemniscate)
        a = size_mm * 0.6
        
        # generate infinity path points for visualization
        num_points = 72       # one point every 5 degrees (360/72 = 5°)
        infinity_points = []
        
        for i in range(num_points + 1):
            # parameter t from 0 to 2π (one full loop)
            t = 2 * np.pi * i / num_points
            denominator = 1 + np.cos(t)**2
            x = a * np.sin(t) / denominator
            y = stretch * a * np.sin(t) * np.cos(t) / denominator
            
            infinity_points.append((x, y))
        
        def param_generator(elapsed):
            # use speed and time to determine the parameter t
            t = angular_speed * elapsed
            denominator = 1 + np.cos(t)**2
            x = a * np.sin(t) / denominator
            y = stretch * a * np.sin(t) * np.cos(t) / denominator
            return x, y
        
        self._execute_continuous_path(param_generator=param_generator,
                                      period_sec=period_sec,
                                      repeats=repeats,
                                      stop_event=stop_event,
                                      trajectory_type='infinity',
                                      trajectory_points=infinity_points,
                                      completion_callback=completion_callback
                                      )
    
    
    
    # ==================== TRIANGLE PATH ====================
    
    def triangle_path(self, side_mm: int, orientation: str = 'point_up', repeats: int = 1,
                      tolerance_mm: int = 40, settle_time_ms: int = 400,
                      stop_event: threading.Event = None, completion_callback=None):
        """
        Execute a triangular path.
        
        Args:
            side_mm: Side length of triangle in mm
            orientation: 'point_up' or 'point_down'
            repeats: Number of triangles to complete
            tolerance_mm: Distance from target to consider "arrived" (mm)
            settle_time_ms: Time ball must stay within tolerance before advancing (ms)
            stop_event: Event to signal path stop
        """
        
        side_mm = min(180, max(50, side_mm))
        
        # calculate triangle vertices (centered around 0,0)
        height = side_mm * np.sqrt(3) / 2  # equilateral triangle height
        
        if orientation == 'point_up':
            # pointing up: top vertex at y = +height/3, base at y = -2*height/3
            vertices = [
                (0, height * 2/3),           # top
                (-side_mm/2, -height/3),     # bottom-left
                (side_mm/2, -height/3)       # bottom-right
            ]
        else:
            # pointing down: bottom vertex at y = -height/3, base at y = +2*height/3
            vertices = [
                (0, -height * 2/3),          # bottom
                (-side_mm/2, height/3),      # top-left
                (side_mm/2, height/3)        # top-right
            ]
        
        # generate full path (including repeats)
        full_path = [(0, 0)]           # start at center
        for _ in range(repeats):
            full_path.extend(vertices)
        full_path.append(vertices[0])  # return to first vertex
        full_path.append((0, 0))       # return to center
        
        if self.verbose:
            print("\n" + "="*60)
            print("TRIANGLE PATH FOLLOWING")
            print("="*60)
            print(f"Side length: {side_mm} mm")
            print(f"Orientation: {orientation}")
            print(f"Repeats: {repeats}")
            print(f"Tolerance: ±{tolerance_mm} mm")
            print(f"Settle time: {settle_time_ms} ms")
            print("="*60)
        
        # set trajectory for visualization
        self.camera.set_trajectory_points(vertices + [vertices[0]], 'triangle')
        
        self._execute_vertex_path(full_path, tolerance_mm, settle_time_ms,
                                  stop_event, 'triangle',
                                  completion_callback=completion_callback)
    
    # ==================== LINE PATH ====================
    
    def line_path(self, length_mm: int, angle_deg: int, repeats: int = 1,
                  tolerance_mm: int = 40, settle_time_ms: int = 400,
                  stop_event: threading.Event = None, completion_callback=None):
        """
        Execute a line path at specified angle.
        
        Args:
            length_mm: Length of the line in mm
            angle_deg: Direction angle in degrees (0 = right, 90 = up, etc.)
            repeats: Number of times to traverse the line
            tolerance_mm: Distance from target to consider "arrived" (mm)
            settle_time_ms: Time ball must stay within tolerance before advancing (ms)
            stop_event: Event to signal path stop
        """
        
        length_mm = min(160, max(40, length_mm))
        
        # calculate endpoints
        angle_rad = radians(angle_deg)
        half_length = length_mm / 2
        x1 = half_length * cos(angle_rad)
        y1 = half_length * sin(angle_rad)
        x2 = -half_length * cos(angle_rad)
        y2 = -half_length * sin(angle_rad)
        
        # generate path points
        full_path = [(0, 0)]  # start at center
        for _ in range(repeats):
            full_path.append((x1, y1))
            full_path.append((x2, y2))
        full_path.append((0, 0))  # return to center
        
        if self.verbose:
            print("\n" + "="*60)
            print("LINE PATH FOLLOWING")
            print("="*60)
            print(f"Length: {length_mm} mm")
            print(f"Angle: {angle_deg}°")
            print(f"Repeats: {repeats}")
            print(f"Tolerance: ±{tolerance_mm} mm")
            print(f"Settle time: {settle_time_ms} ms")
            print("="*60)
        
        # set trajectory for visualization
        trajectory_points = [(x1, y1), (x2, y2)]
        self.camera.set_trajectory_points(trajectory_points, 'line')
        
        self._execute_vertex_path(full_path, tolerance_mm, settle_time_ms,
                                  stop_event, 'line',
                                  completion_callback=completion_callback)
    
    
    
    # ==================== FREE PATH FUNCTIONS ====================
    
    def free_path_continuous(self, stop_event: threading.Event = None):
        """
        Continuous free path mode - follows finger touches.
        
        Args:
            stop_event: Event to signal path stop
        """
        if self.verbose:
            print("\n" + "="*60)
            print("FREE PATH - CONTINUOUS MODE")
            print("="*60)
            print("Touch the video area to move the ball")
            print("="*60)
        
        # start auto-balance if not running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(1)
        
        # clear any previous visualization - this sets trajectory_type = None (shows quadrants)
        self.camera.clear_trajectory()
        self.camera.clear_target()
        
        # wait for stop signal
        while not (stop_event and stop_event.is_set()):
            # small sleep to prevent CPU hogging
            sleep(0.05)
        
        if self.verbose:
            print("\nFree path stopped")
        
        # clear everything to show quadrants
        self.camera.clear_trajectory()
        self.camera.clear_target()
        
        # ensure the controller target is centered
        self.controller.target_x = 0
        self.controller.target_y = 0


    
    def free_path_record(self, stop_event: threading.Event = None):
        """
        Recording mode for free path - records touch points with timestamps.
        This is the main method called by mbb_robot.py.
        
        Args:
            stop_event: Event to signal stop recording
        
        Returns:
            List of recorded points (x, y, timestamp)
        """
        if self.verbose:
            print("\n" + "="*60)
            print("FREE PATH - RECORDING MODE")
            print("="*60)
            print("Draw on the video area to record a path")
            print("Press STOP RECORDING when done")
            print("="*60)
        
        # clear previous recording - store points with timestamps
        self._recorded_points = []
        self._recording_active = True
        self._recording_start_time = time()
        
        # start auto-balance if not running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(1)
        
        # clear any previous visualization
        self.camera.clear_trajectory()
        self.camera.clear_target()
        
        # wait for stop signal
        while not (stop_event and stop_event.is_set()):
            sleep(0.05)
        
        self._recording_active = False
        if self.verbose:
            print(f"\nRecording stopped: {len(self._recorded_points)} points recorded")
        
        # clear the target when recording stops
        self.camera.clear_target()
        
        # return recorded points
        return self._recorded_points


    
    def free_path_start_recording(self):
        """
        Start recording mode for free path.
        Alternative entry point for GUI.
        """
        if self.verbose:
            print("\n" + "="*60)
            print("FREE PATH - RECORDING MODE")
            print("="*60)
            print("Draw on the video area to record a path")
            print("Press STOP RECORDING when done")
            print("="*60)
        
        # clear previous recording
        self._recorded_points = []
        self._recording_active = True
        self._recording_start_time = time()
        
        # start auto-balance if not running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(1)
        
        # clear any previous visualization
        self.camera.clear_trajectory()
        self.camera.clear_target()


    
    def free_path_stop_recording(self):
        """
        Stop recording mode for free path.
        """
        self._recording_active = False
        if self.verbose:
            print(f"\nRecording stopped: {len(self._recorded_points)} points recorded")
        
        # clear the target when recording stops
        self.camera.clear_target()
        
        # return recorded points
        return self._recorded_points


    
    def free_path_playback(self, points, stop_event: threading.Event = None, completion_callback=None):
        """
        Playback recorded free path.
        
        Args:
            points: List of (x, y) tuples (coordinates in mm)
            stop_event: Event to signal path stop
            completion_callback: Function to call when playback completes
        """
        if self.verbose:
            print("\n" + "="*60)
            print(f"FREE PATH - PLAYBACK MODE ({len(points)} points)")
            print("="*60)
        
        if not points:
            if self.verbose:
                print("No points to play back")
            if completion_callback:
                completion_callback()
            return
        
        # start auto-balance if not running - with minimal delay
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            # HERE
            # reduce sleep time - the controller will signal when ready
            # use a shorter wait since we just need it to start, not fully stabilize
            sleep(0.2)
        else:
            # auto-balance is already running, just a tiny delay for stability
            sleep(0.05)
        
        # generate trajectory points for visualization
        trajectory_points = []
        for point in points:
            x_mm, y_mm = point[0], point[1]
            trajectory_points.append((x_mm, y_mm))
        
        # set trajectory for visualization
        self.camera.set_trajectory_points(trajectory_points, 'free')
        
        try:
            if self.verbose:
                print("Starting playback...")
            
            # use fixed delay between points
            BASE_DELAY = 0.05  # 50ms
            
            # pre-warm the controller by setting the first target immediately
            if points:
                first_x, first_y = points[0][0], points[0][1]
                self.controller.target_x = int(first_x * 10)
                self.controller.target_y = int(first_y * 10)
                self.camera.set_target_position(first_x, first_y, 'free')
            
            for i, (x_mm, y_mm) in enumerate(points):
                if stop_event and stop_event.is_set():
                    if self.verbose:
                        print("\nPlayback stopped by user")
                    break
                
                # update target
                self.controller.target_x = int(x_mm * 10)
                self.controller.target_y = int(y_mm * 10)
                
                # update visualization
                if i % 10 == 0 or i == len(points) - 1:
                    self.camera.set_target_position(x_mm, y_mm, 'free')
                
                sleep(BASE_DELAY)
            
            # return to center after playback completes
            if self.verbose:
                print("Returning to center...")
            self.controller.target_x = 0
            self.controller.target_y = 0
            self.camera.set_target_position(0, 0, 'free')
#             sleep(0.5)
        
        finally:
            if self.verbose:
                print("\nPlayback completed")
            # clear visualization to show quadrants
            self.camera.clear_trajectory()
            self.camera.clear_target()
            
            # call completion callback if provided
            if completion_callback:
                completion_callback()


    
    def get_recorded_points(self):
        """Get the recorded points."""
        return self._recorded_points


    
    def update_touch_target(self, x_mm, y_mm):
        """
        Called by GUI during continuous free path or recording.
        
        Args:
            x_mm: X coordinate in mm
            y_mm: Y coordinate in mm
        """
        # clamp to valid range
        x_mm = max(-80, min(80, x_mm))
        y_mm = max(-80, min(80, y_mm))
        
        # update target position
        self.controller.target_x = int(x_mm * 10)
        self.controller.target_y = int(y_mm * 10)
        
        # if recording, store the point with timestamp
        if self._recording_active:
            elapsed_time = time() - self._recording_start_time
            self._recorded_points.append((x_mm, y_mm, elapsed_time))
            # during recording, show the target with free trajectory
            self.camera.set_target_position(x_mm, y_mm, 'free')
        else:
            # during continuous mode, show target to hide quadrants
            # use 'free' to hide quadrants during active touch
            self.camera.set_target_position(x_mm, y_mm, 'free')
    
    
    
    # ==================== HELPER METHODS ====================
    
    def _execute_vertex_path(self, vertices, tolerance_mm, settle_time_ms, stop_event,
                             trajectory_type='path', completion_callback=None):
        """
        Generic method to execute a path defined by vertices.
        
        Args:
            vertices: List of (x, y) tuples in mm
            tolerance_mm: Distance tolerance for arrival
            settle_time_ms: Time to settle at each vertex
            stop_event: Event to signal stop
            trajectory_type: Type of trajectory for visualization ('triangle', 'line', etc.)
        """
        
        # check if auto-balance is running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(2)
        
        settle_sec = settle_time_ms / 1000.0
        path_start_time = time()
        
        if self.verbose:
            print(f"\nStarting {trajectory_type} path with {len(vertices)-1} moves...")
            print("-" * 40)
        
        try:
            for i, (target_x_mm, target_y_mm) in enumerate(vertices):
                # check stop signal
                if stop_event and stop_event.is_set():
                    if self.verbose:
                        print("\nPath stopped by user request")
                    break
                
                target_x_tenths = target_x_mm * 10
                target_y_tenths = target_y_mm * 10
                
                if self.verbose:
                    print(f"\nStep {i+1}/{len(vertices)}: Moving to ({target_x_mm:5.1f}, {target_y_mm:5.1f}) mm")
                step_start_time = time()
                
                # set new target
                self.controller.target_x = target_x_tenths
                self.controller.target_y = target_y_tenths
                
                # update visualization target
                self.camera.set_target_position(target_x_mm, target_y_mm, trajectory_type)
                
                # reset integral terms for clean start
                self.controller.integral_x = 0
                self.controller.integral_y = 0
                
                # clear derivative state
                if hasattr(self.controller, 'last_position_x'):
                    self.controller.last_position_x = None
                    self.controller.last_position_y = None
                
                arrived_time = None
                last_status_time = time()
                
                # wait for arrival + settle time
                while True:
                    if stop_event and stop_event.is_set():
                        if self.verbose:
                            print("\nPath stopped by user request")
                        return
                    
                    current_time = time()
                    x_px, y_px = self.camera.get_ball_position()
                    
                    if x_px != -9999 and y_px != -9999:
                        x_tenths, y_tenths = self.controller.pixel_to_tenths_mm(x_px, y_px)
                        dx = x_tenths - target_x_tenths
                        dy = y_tenths - target_y_tenths
                        distance_mm = np.sqrt(dx*dx + dy*dy) / 10
                        
                        if current_time - last_status_time >= 1.0:
                            if arrived_time is None:
                                elapsed = current_time - step_start_time
                                if self.verbose:
                                    print(f"    t={elapsed:4.1f}s | Distance: {distance_mm:5.1f} mm")
                            else:
                                stable_time = current_time - arrived_time
                                if self.verbose:
                                    print(f"    t={current_time - path_start_time:4.1f}s | Stable for {stable_time:.1f}s")
                            last_status_time = current_time
                        
                        if distance_mm <= tolerance_mm:
                            if arrived_time is None:
                                arrived_time = current_time
                                if self.verbose:
                                    print(f"   Arrived at target ({current_time - step_start_time:.1f}s)")
                            elif current_time - arrived_time >= settle_sec:
                                if self.verbose:
                                    print(f"   Stable - moving to next point")
                                break
                        else:
                            arrived_time = None
                    
                    sleep(0.05)
                
                if i == len(vertices) - 1:
                    break
        
        except KeyboardInterrupt:
            if self.verbose:
                print("\n\nPath interrupted by user")
        
        finally:
            if self.verbose:
                total_time = time() - path_start_time
                print("\n" + "="*40)
                print(f"{trajectory_type} path completed in {total_time:.1f} seconds")
            self._cleanup_path()
            
            if completion_callback:
                ended_normally = not (stop_event and stop_event.is_set())
                completion_callback(ended_normally=ended_normally)
        
        
    
    def _execute_continuous_path(self, param_generator, period_sec, repeats, 
                                 stop_event, trajectory_type='path',
                                 trajectory_points=None, completion_callback=None):
        """
        Generic method to execute a continuous path, like circle, Infinity, Spiral, etc
        
        Args:
            param_generator: Function that yields (x_mm, y_mm) for each step
                             Takes (elapsed_time, completed_count) as parameters
            period_sec: Time in seconds for one complete cycle
            repeats: Number of times to repeat
            stop_event: Event to signal stop
            trajectory_type: Type for visualization
            trajectory_points: Points for trajectory visualization
            completion_callback: Called when path completes
        """
        
        # check if auto-balance is running
        if not self.controller.auto_balance:
            if self.verbose:
                print("Starting auto-balance mode...")
            self.robot.start_auto_balance()
            sleep(2)
        
        # set trajectory for visualization
        if trajectory_points:
            self.camera.set_trajectory_points(trajectory_points, trajectory_type)
        
        if self.verbose:
            print(f"\nStarting {trajectory_type} path...")
            print("-" * 40)
        
        try:
            # reset integral terms for clean start
            self.controller.integral_x = 0
            self.controller.integral_y = 0
            
            # clear derivative state
            if hasattr(self.controller, 'last_position_x'):
                self.controller.last_position_x = None
                self.controller.last_position_y = None
            
            # track progress using period-based counting
            completed_cycles = 0
            cycle_count = 0
            
            start_time = time()
            last_status_time = start_time          
            
            while completed_cycles < repeats and not (stop_event and stop_event.is_set()):
               
                current_time = time()
                elapsed = current_time - start_time
                
                # generate current position using elapsed time 
                x_mm, y_mm = param_generator(elapsed)
                x_tenths = int(x_mm * 10)
                y_tenths = int(y_mm * 10)
                
                # set new target
                self.controller.target_x = x_tenths
                self.controller.target_y = y_tenths
                
                # update visualization target
                self.camera.set_target_position(x_mm, y_mm, trajectory_type)
                
                # track cycle completion using period
                new_cycle_count = int(elapsed / period_sec)
                if new_cycle_count > cycle_count:
                    cycle_count = new_cycle_count
                    completed_cycles = min(cycle_count, repeats)
                    
                    if self.verbose and completed_cycles < repeats:
                        print(f"Completed cycle {completed_cycles}/{repeats}")

                # print status every second
                if current_time - last_status_time >= 1.0:
                    # get actual ball position
                    x_px, y_px = self.camera.get_ball_position()
                    
                    if x_px != -9999 and y_px != -9999:
                        x_tenths_actual, y_tenths_actual = self.controller.pixel_to_tenths_mm(x_px, y_px)
                        error_x = x_tenths_actual - x_tenths
                        error_y = y_tenths_actual - y_tenths
                        error_dist = np.sqrt(error_x**2 + error_y**2) / 10  # in mm
                        
                        if self.verbose:
                            print(f"    t={elapsed:4.1f}s | "
                                  f"Target: ({x_mm:6.1f}, {y_mm:6.1f}) mm | "
                                  f"Error: {error_dist:5.1f} mm")
                    else:
                        if self.verbose:
                            print(f"    t={elapsed:4.1f}s | Ball not visible")
                    
                    last_status_time = current_time
                
                # small sleep to prevent CPU hogging
                sleep(0.01)
        
        except KeyboardInterrupt:
            if self.verbose:
                print(f"\n\n{trajectory_type} path interrupted by user")
        
        finally:
            if self.verbose:
                total_time = time() - start_time
                print("\n" + "="*40)
                print(f"Completed {repeats} {trajectory_type}(s) in {total_time:.1f} seconds")
            
            self._cleanup_path()
            
            if completion_callback:
                ended_normally = not (stop_event and stop_event.is_set())
                completion_callback(ended_normally=ended_normally)
    
    
    
    def _cleanup_path(self):
        """Clean up after path execution."""
        if self.verbose:
            print("Returning to center and clearing visualization...")
        self.controller.target_x = 0
        self.controller.target_y = 0
        self.controller.integral_x = 0
        self.controller.integral_y = 0
        self.camera.clear_target()
        self.camera.clear_trajectory()
        sleep(2)
        if self.verbose:
            print("Path completed")
            print("="*40)
