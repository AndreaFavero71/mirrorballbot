#!/usr/bin/env python
# coding: utf-8

"""
Andrea Favero 20260517

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the touchscreen GUI




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
# gui for mirrorballbot by andrea favero
# ============================================================================

__version__ = "0.0.2"


VERBOSE = False  # set True for debugging (mind it slows down the robot)


import datetime as dt
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

from math import sqrt, radians, degrees, cos, sin 
from PIL import Image, ImageTk
from time import time, sleep
from json import dump, load
from sys import platform
from os import environ
import threading
import cv2


# mirrorballbot custom modules
from mbb_robot import BallBalancingSystem, MotorAngles
from mbb_display_mgr import DisplayManager
from mbb_settings_mgr import SettingsManager


DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
TITLE_BAR_HEIGHT = 30
PI_TASKBAR_HEIGHT = 30
WINDOWED_HEIGHT = DISPLAY_HEIGHT - TITLE_BAR_HEIGHT - PI_TASKBAR_HEIGHT  # 480 - 60 = 420


class OptimizedCanvas(tk.Canvas):
    """
    Optimized canvas with hardware-accelerated drawing and flicker reduction.
    note: this uses single-buffered drawing with frame skipping, which works well
    for the dsi display when vnc is not active. true double buffering is not
    implemented here as it introduced additional flicker in testing.
    """
    
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self.current_image = None
        self.current_photo = None
        self.original_width = 1
        self.original_height = 1
        self.canvas_width = 1
        self.canvas_height = 1
        self.image_id = None
        self.is_alive = True
        self.touch_callback = None
        self.aspect_ratio = 1.0
        
        # used for the pixels to mm conversion
        self.camera = None
        self.mm_per_display_pixel = None
        
        # configure canvas for optimal performance
        self.configure(highlightthickness=0, bd=0)
        
        # bind touch/mouse events for free path mode
        self.bind('<Button-1>', self._on_touch_start)
        self.bind('<B1-Motion>', self._on_touch_move)
        self.bind('<ButtonRelease-1>', self._on_touch_end)
        
        # bind configure event to handle resizing
        self.bind('<Configure>', self._on_configure)
    
    
    
    def _on_configure(self, event):
        """Handle canvas resize while maintaining aspect ratio."""
        if self.is_alive and (event.width != self.canvas_width or event.height != self.canvas_height):
            self.canvas_width = event.width
            self.canvas_height = event.height
            
            # calculate new position for centered image
            if self.original_width > 0 and self.original_height > 0:
                self._center_image()
            
            if self.image_id:
                self.delete(self.image_id)
                self.image_id = None
    
    
    
    def _center_image(self):
        """Calculate centered position for image maintaining aspect ratio."""
        # calculate display area preserving aspect ratio
        display_ratio = self.canvas_width / self.canvas_height
        image_ratio = self.original_width / self.original_height
        
        if display_ratio > image_ratio:
            # image is taller relative to display
            self.display_height = self.canvas_height
            self.display_width = int(self.canvas_height * image_ratio)
        else:
            # image is wider relative to display
            self.display_width = self.canvas_width
            self.display_height = int(self.canvas_width / image_ratio)
        
        # center position
        self.x_offset = (self.canvas_width - self.display_width) // 2
        self.y_offset = (self.canvas_height - self.display_height) // 2
        
        # update conversion factor when display dimensions change
        self._update_conversion_factor()
    
    
    
    def _get_usable_radius(self):
        """Get the usable radius for ball movement (mm)."""
        if self.camera:
            platform_radius = self.camera.platform_mm / 2
            ball_radius = self.camera.ball_mm / 2
            safety_margin = ball_radius    # one ball radius margin
            usable_radius = platform_radius - ball_radius - safety_margin
            return max(usable_radius, 40)  # minimum 40mm for safety
        else:
            return 80  # default fallback
    
    
    
    def set_camera(self, camera):
        """Set camera reference for coordinate conversion."""
        self.camera = camera
        # try to update conversion factor based on display dimensions
        self._update_conversion_factor()
    
    
    
    def _update_conversion_factor(self):
        """Calculate mm per display pixel using camera calibration."""
        if not self.camera:
            return
        
        # ensure valid dimensions
        if not hasattr(self, 'display_width') or self.display_width <= 0:
            # will be called again when image is loaded and centered
            return
        
        if self.original_width <= 0:
            return
    
        # calculate using camera calibration
        scale = self.original_width / self.display_width
        self.mm_per_display_pixel = scale * self.camera.pixels_to_mm
    
    
    
    def _on_touch_start(self, event):
        """Handle touch start for free path mode."""
        if self.touch_callback:
            relative_x = event.x - self.x_offset
            relative_y = event.y - self.y_offset
            
            if 0 <= relative_x <= self.display_width and 0 <= relative_y <= self.display_height:
                # ensure conversion factor is calculated
                if self.mm_per_display_pixel is None:
                    self._update_conversion_factor()
                
                # convert screen pixels to mm (assuming full square)
                x_mm = (relative_x - self.display_width/2) * self.mm_per_display_pixel
                y_mm = (self.display_height/2 - relative_y) * self.mm_per_display_pixel
                
                # calculate distance from center
                distance = sqrt(x_mm*x_mm + y_mm*y_mm)
                
                # get usable radius from camera calibration
                usable_radius = self._get_usable_radius()
                
                # clip to circular platform boundary
                if distance > usable_radius:
                    # scale down to the edge of the usable circle
                    scale = usable_radius / distance
                    x_mm = x_mm * scale
                    y_mm = y_mm * scale
                
                self.touch_callback('start', x_mm, y_mm)
    
    
    
    def _on_touch_move(self, event):
        """Handle touch move for free path mode."""
        if self.touch_callback:
            relative_x = event.x - self.x_offset
            relative_y = event.y - self.y_offset
            
            if 0 <= relative_x <= self.display_width and 0 <= relative_y <= self.display_height:
                # ensure conversion factor is calculated
                if self.mm_per_display_pixel is None:
                    self._update_conversion_factor()
                
                # convert screen pixels to mm (assuming full square)
                x_mm = (relative_x - self.display_width/2) * self.mm_per_display_pixel
                y_mm = (self.display_height/2 - relative_y) * self.mm_per_display_pixel
                
                # calculate distance from center
                distance = sqrt(x_mm*x_mm + y_mm*y_mm)
                
                # get usable radius from camera calibration
                usable_radius = self._get_usable_radius()
                
                # clip to circular platform boundary
                if distance > usable_radius:
                    # scale down to the edge of the usable circle
                    scale = usable_radius / distance
                    x_mm = x_mm * scale
                    y_mm = y_mm * scale
                
                self.touch_callback('move', x_mm, y_mm)
    
    
    
    def _on_touch_end(self, event):
        """Handle touch end for free path mode."""
        if self.touch_callback:
            self.touch_callback('end', 0, 0)
    
    
    
    def set_touch_callback(self, callback):
        """Set callback for touch events in free path mode."""
        self.touch_callback = callback
    
    
    
    def update_image(self, cv_image):
        """Update canvas with new image maintaining aspect ratio."""
        if not self.is_alive or cv_image is None:
            return
        
        try:
            # store original dimensions
            self.original_height, self.original_width = cv_image.shape[:2]
            self.aspect_ratio = self.original_width / self.original_height
            
            # update centered position if canvas size changed
            if self.canvas_width > 0 and self.canvas_height > 0:
                self._center_image()
            
            # convert BGR to RGB
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = cv_image
            
            # convert to PIL Image
            pil_image = Image.fromarray(rgb_image)
            
            # resize maintaining aspect ratio to fit display area
            if self.display_width > 10 and self.display_height > 10:
                pil_image = pil_image.resize((self.display_width, self.display_height), 
                                            Image.Resampling.LANCZOS)
            
            # create PhotoImage
            photo = ImageTk.PhotoImage(pil_image)
            
            # update or create image on canvas at centered position
            if self.image_id is None:
                self.image_id = self.create_image(self.x_offset, self.y_offset, 
                                                  anchor='nw', image=photo)
            else:
                self.itemconfig(self.image_id, image=photo)
                self.coords(self.image_id, self.x_offset, self.y_offset)
            
            self.current_photo = photo
            
        except Exception as e:
            pass
    
    
    
    def destroy(self):
        """Clean up canvas resources."""
        self.is_alive = False
        self.current_photo = None
        super().destroy()





class StaticFrame(tk.Frame):
    """Frame that minimizes redraw events by caching its content."""
    
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self._redraw_enabled = True
    
    
    
    def freeze_updates(self):
        """Temporarily freeze UI updates to prevent flicker."""
        self._redraw_enabled = False
    
    
    
    def thaw_updates(self):
        """Resume UI updates."""
        self._redraw_enabled = True





class BallBalancingGUI(tk.Tk):
    """Main GUI application for the ball balancing robot."""
    
    def __init__(self, verbose=False, settings_mgr=None):
        super().__init__()
        
        self.title("MirrorBallBot Control v" + __version__)
        
        # verbose mode
        self.verbose = verbose
        
        if self.verbose:
            print(f"\nmbb_gui.py VERSION:  {__version__}\n")
        
        self.colors = {
            'bg': '#2b2b2b',
            'fg': '#ffffff',
            'button': '#404040',
            'accent': '#0078d4',
            'success': '#28a745',
            'warning': '#ffc107',
            'error': '#dc3545',
            'video_bg': '#000000',
            'info_bar': '#1e1e1e'
        }
        
        # configure grid - 60/40 split
        self.grid_columnconfigure(0, weight=60, uniform='panel')
        self.grid_columnconfigure(1, weight=40, uniform='panel')
        self.grid_rowconfigure(0, weight=1)
        self.configure(bg=self.colors['bg'])

        # create a display_manager instance
        display_mgr = DisplayManager()
        
        # initialize robot
        print("Initializing robot...")
        
        self.system = BallBalancingSystem(gui_mode=True,
                                          auto_calibrate=True,
                                          verbose=self.verbose,
                                          display_manager=display_mgr,
                                          settings_mgr=settings_mgr)
        
            
        # load some of the GUI settings
        if settings_mgr is not None:
            settings = settings_mgr.get()
            self.video_update_interval_ms = settings.display.video_update_interval_ms 
            self.status_update_interval_ms = settings.display.status_update_interval_ms 
            self.frame_skip = settings.display.frame_skip
        else:
            self.video_update_interval_ms = 66 
            self.status_update_interval_ms = 500 
            self.frame_skip = 5
        
        # other settings to maximize camera FPS by limiting CPU overhead amd flickering
        self.use_optimized_canvas = True
        self.frame_counter = 0
        
        
        self.running = True
        self.current_page = "main"
        self.video_running = True
        self.is_closing = False
        self.current_path_page = None
        self.is_fullscreen = False

        # store page references
        self.main_page = None
        self.paths_page = None
        self.calibration_page = None
        self.ball_page = None
        self.motor_page = None
        self.homing_page = None
        self.plat_page = None
        self.pid_page = None
        
        # store previous shift values for delta calculation
        self.prev_shift_a = self.system.controller.BA_SHIFT_A
        self.prev_shift_b = self.system.controller.BA_SHIFT_B
        self.prev_shift_c = self.system.controller.BA_SHIFT_C

        self.old_shift_a = self.prev_shift_a
        self.old_shift_b = self.prev_shift_b
        self.old_shift_c = self.prev_shift_c
        
        # create UI
        self.create_video_panel()
        self.create_control_panel()
        
        # set window size and position - start in windowed mode
        self._set_windowed_geometry()
        
        # DSI display optimizations
        self._optimize_for_dsi_display()
        
        # start background tasks
        self.update_video()
        self.update_status()
        
        # start auto-balance by default
        self.after(100, self.auto_start)
        self.after(2000, self.auto_start)  # repeat with larger delay ensuring checkbutton flag presence
        
        # fullscreen after startup
        self.after(100, lambda: self.toggle_fullscreen())
        self.bind('<Escape>', self.toggle_fullscreen)
        self.update_idletasks()
    
        # setup close protocol
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        print("GUI ready")
    
    
    
    def _set_windowed_geometry(self):
        """Set window to proper size for windowed mode."""
        # calculate window size that fits below taskbar
        win_w = DISPLAY_WIDTH
        win_h = WINDOWED_HEIGHT
        
        # position at top-left, below the Pi taskbar
        x = 0
        y = PI_TASKBAR_HEIGHT
        
        self.geometry(f'{win_w}x{win_h}+{x}+{y}')
        self.is_fullscreen = False
    
    
    
    def _set_fullscreen_geometry(self):
        """Set window to fullscreen."""
        self.attributes('-fullscreen', True)
        self.is_fullscreen = True
    
    
    
    def _optimize_for_dsi_display(self):
        """Apply comprehensive optimizations for DSI displays."""
        self.tk.call('tk', 'scaling', 1.0)
        try:
            self.tk.call('wm', 'attributes', '.', '-alpha', 1.0)
        except:
            pass
        try:
            self.tk.call('tk', 'useinputmethods', '0')
        except:
            pass
        if platform == 'linux':
            try:
                environ['TK_SCALING'] = '1'
                environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
            except:
                pass
        self.update_idletasks()
    
    
    
    def auto_start(self):
        """Start auto-balance if platform is ready."""
        if not self.is_closing:
            if hasattr(self, 'auto_balance_var'):
                self.auto_balance_var.set(True)
                self.toggle_auto_balance()
                self.update_idletasks()
    
    
    
    def remove_highlight(self, button, highlightthickness=0):
        """Completely remove any visual highlight/pressed effect from a button."""
        if not button.winfo_exists():
            return
    
        button.config(
            activebackground=button.cget('bg'),
            activeforeground=button.cget('fg'),
            highlightthickness=highlightthickness,
            relief=tk.RAISED
        )
    
    
    
    
    # ==================== UI CREATION ====================
    
    def create_video_panel(self):
        """Create left panel with camera feed using optimized canvas."""
        self.video_frame = tk.Frame(self, bg=self.colors['video_bg'], relief=tk.SUNKEN, bd=2)
        self.video_frame.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        self.video_frame.grid_rowconfigure(0, weight=1)
        self.video_frame.grid_columnconfigure(0, weight=1)
        
        if self.use_optimized_canvas:
            self.video_canvas = OptimizedCanvas(
                self.video_frame,
                bg=self.colors['video_bg'],
                highlightthickness=0
            )
            self.video_canvas.grid(row=0, column=0, sticky="nsew")
            
            # pass camera reference for coordinate conversion
            if hasattr(self, 'system') and hasattr(self.system, 'camera'):
                self.video_canvas.set_camera(self.system.camera)
            
            self.video_display = self.video_canvas
        else:
            self.video_label = tk.Label(self.video_frame, bg=self.colors['video_bg'])
            self.video_label.grid(row=0, column=0, sticky="nsew")
            self.video_display = self.video_label
        
        # fullscreen button (top-left)
        fs_container = tk.Frame(self.video_frame, bg=self.colors['bg'], bd=1, relief=tk.RAISED)
        fs_container.place(x=5, y=5, anchor='nw')
        
        self.fullscreen_btn = tk.Button(
            fs_container, text="\u25BC", bg=self.colors['button'], fg=self.colors['fg'],
            font=('Arial', 14), width=2, relief=tk.FLAT, bd=0,
            command=self.toggle_fullscreen
        )
        self.fullscreen_btn.pack()
        
        # temperature display (top-right)
        temp_container = tk.Frame(self.video_frame, bg=self.colors['bg'], bd=1, relief=tk.RAISED)
        temp_container.place(relx=1.0, x=-10, y=5, anchor='ne')
        
        self.temp_value = tk.Label(
            temp_container,
            text="--°C",
            bg=self.colors['bg'],
            fg=self.colors['fg'],
            font=('Arial', 16, 'bold')
        )
        self.temp_value.pack(side=tk.LEFT, padx=5)
        
        # start temperature updates
        self.temp_update_interval_ms = 15000  # 15 seconds
        self.update_cpu_temperature()
    
    
    
    def create_control_panel(self):
        """Create right panel with tabs and status bar."""
        control = StaticFrame(self, bg=self.colors['bg'])
        control.grid(row=0, column=1, sticky="nsew", padx=5, pady=2)
        
        control.grid_rowconfigure(1, weight=1)
        control.grid_columnconfigure(0, weight=1)
        
        # title
        title_label = tk.Label(control, text="MirrorBallBot Controls", bg=self.colors['bg'],
                               fg=self.colors['fg'], font=('Arial', 20, 'bold'))
        title_label.grid(row=0, pady=(3,3))
        
        # page container
        self.page_container = StaticFrame(control, bg=self.colors['bg'])
        self.page_container.grid(row=1, sticky="nsew", pady=2)
        self.page_container.grid_columnconfigure(0, weight=1)
        self.page_container.grid_rowconfigure(0, weight=1)
        
        # navigation buttons - 3 buttons (main, paths, calibration)
        nav = StaticFrame(control, bg=self.colors['bg'])
        nav.grid(row=2, sticky="ew", pady=5)
        
        nav_buttons = [("MAIN", 0), ("PATHS", 1), ("CALIBRATION", 2)]
        for i, (text, col) in enumerate(nav_buttons):
            nav.grid_columnconfigure(i, weight=1)
            btn = tk.Button(nav, text=text, bg=self.colors['button'], fg=self.colors['fg'],
                           font=('Arial', 11, 'bold'), relief=tk.RAISED, bd=1,
                           command=lambda p=text.lower(): self.show_page(p))
            btn.grid(row=0, column=i, sticky="ew", padx=1)
        
        # emergency stop
        self.estop_btn = tk.Button(control, text="EMERGENCY STOP", bg=self.colors['error'],
                                   fg='white', font=('Arial', 12, 'bold'), height=1,
                                   command=self.emergency_stop)
        self.estop_btn.grid(row=3, sticky="ew", pady=2)
        
        # status bar
        status = StaticFrame(control, bg=self.colors['info_bar'], height=25)
        status.grid(row=4, sticky="ew")
        status.grid_columnconfigure(0, weight=1)
        status.grid_columnconfigure(1, weight=1)
        status.grid_columnconfigure(2, weight=1)
        
        self.system_status = tk.Label(status, text="System: READY", bg=self.colors['info_bar'],
                                      fg=self.colors['success'], font=('Arial', 8))
        self.system_status.grid(row=0, column=0, sticky="w", padx=5)
        
        self.balance_status = tk.Label(status, text="Balance: OFF", bg=self.colors['info_bar'],
                                       fg=self.colors['fg'], font=('Arial', 8))
        self.balance_status.grid(row=0, column=1, sticky="w")
        
        self.ball_status = tk.Label(status, text="Ball: --", bg=self.colors['info_bar'],
                                    fg=self.colors['fg'], font=('Arial', 8))
        self.ball_status.grid(row=0, column=2, sticky="e", padx=5)
        
        self.show_page("main")
    
    
    
    def show_page(self, page_name):
        """Switch between pages."""
        if self.is_closing:
            return
        
        # at every page change reset the pid accumulators (integral and derivative)
        if page_name != self.current_page and page_name not in ["calibration", "motor"]:
            self.system.controller.reset_pid()
        
        # level platform when leaving PLAT page
        if self.current_page == "plat" and page_name != "plat":
            self.after(100, self.level_platform_with_shifts())
            
        # when leaving PATHS page, stop any running path
        if self.current_page == "paths" and page_name != "paths":
            if self.current_path_page:
                self.current_path_page.destroy()
                self.current_path_page = None
            self.stop_current_path()
            self.after(100, self.center_target())
        
        if self.current_page == "motor" and page_name != "motor":
            self.after(100, self.center_target)
        
        # clean up temporary path config page if exists
        if self.current_path_page:
            self.current_path_page.destroy()
            self.current_path_page = None
        
        # hide all pages
        for page in [self.main_page, self.paths_page, self.calibration_page,
                     self.ball_page, self.motor_page, self.homing_page,
                     self.plat_page, self.pid_page]:
            if page:
                page.grid_remove()
        
        self.current_page = page_name
        
        # show or create the requested page
        if page_name == "main":
            if self.main_page is None:
                self.show_main_page()
            else:
                self.main_page.grid()
        
        elif page_name == "paths":
            if self.paths_page is None:
                self.show_paths_selection_page()
            else:
                self.paths_page.grid()
        
        elif page_name == "calibration":
            if self.calibration_page is None:
                self.show_calibration_page()
            else:
                self.calibration_page.grid()
        
        elif page_name == "ball":
            if self.ball_page is None:
                self.show_ball_page()
            else:
                self.ball_page.grid()
        
        elif page_name == "motor":
            if self.motor_page is None:
                self.show_motor_page()
            else:
                self.motor_page.grid()
        
        elif page_name == "homing":
            if self.homing_page is None:
                self.show_homing_page()
            else:
                self.homing_page.grid()
        
        elif page_name == "plat":
            if self.plat_page is None:
                self.show_plat_page()
            else:
                self.plat_page.grid()
        
        elif page_name == "pid":
            if self.pid_page is None:
                self.show_pid_page()
            else:
                self.pid_page.grid()
    
    
    
    
    # ==================== MAIN PAGE ====================
    
    def show_main_page(self):
        """Create main page (called only once)."""
        
        self.main_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.main_page.grid(row=0, column=0, sticky="nsew")
        self.main_page.grid_columnconfigure(0, weight=1)
        
        # auto-balance checkbox
        self.auto_balance_var = tk.BooleanVar(value=False)
        self.auto_balance_check = tk.Checkbutton(self.main_page,
                                                 text="Enable Auto-Balance  ",
                                                 variable=self.auto_balance_var,
                                                 bg=self.colors['bg'],
                                                 fg=self.colors['fg'],
                                                 selectcolor=self.colors['accent'],
                                                 activebackground=self.colors['accent'],
                                                 highlightthickness=1,
                                                 font=('Arial', 14, 'bold'),
                                                 command=self.toggle_auto_balance)
        self.auto_balance_check.grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.remove_highlight(self.auto_balance_check, highlightthickness=1)
        
        # target position
        target_frame = tk.LabelFrame(self.main_page,
                                     text="Target Position",
                                     bg=self.colors['bg'],
                                     fg=self.colors['fg'],
                                     font=('Arial', 12, 'bold'))
        target_frame.grid(row=1, sticky="ew", padx=5, pady=5)
        target_frame.grid_columnconfigure(1, weight=1)
        
        tk.Label(target_frame,
                 text="X (mm):",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 12)
                 ).grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.target_x = tk.IntVar(value=0)
        tk.Spinbox(target_frame,
                   from_=-100,
                   to=100,
                   textvariable=self.target_x,
                   width=6,
                   font=('Arial', 20),
                   command=self.update_target
                   ).grid(row=0, column=1, sticky="w", padx=2, pady=2)
        
        tk.Label(target_frame,
                 text="Y (mm):",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 12)
                 ).grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.target_y = tk.IntVar(value=0)
        tk.Spinbox(target_frame,
                   from_=-100,
                   to=100,
                   textvariable=self.target_y,
                   width=6,
                   font=('Arial', 20),
                   command=self.update_target
                   ).grid(row=1, column=1, sticky="w", padx=2, pady=5)
        
        center_btn = tk.Button(target_frame,
                  text="Set (0,0)",
                  bg=self.colors['button'],
                  fg=self.colors['fg'],
                  font=('Arial', 12),
                  command=self.center_target
                  )
        center_btn.grid(row=0, column=2, rowspan=2, padx=5, pady=10, sticky="ns")
        self.remove_highlight(center_btn)
        
        # status area
        status_container = StaticFrame(self.main_page, bg=self.colors['bg'])
        status_container.grid(row=2, sticky="nsew", padx=5, pady=5)
        status_container.grid_columnconfigure(0, weight=1)
        status_container.grid_columnconfigure(1, weight=2)
        
        # left column - motor angles
        left = tk.LabelFrame(status_container,
                             text="Status",
                             bg=self.colors['bg'],
                             fg=self.colors['fg'],
                             font=('Arial', 12, 'bold'))
        left.grid(row=0, column=0, sticky="nsew", padx=2)
        
        self.motor_a = tk.StringVar(value="Motor A: ----°")
        self.motor_b = tk.StringVar(value="Motor B: ----°")
        self.motor_c = tk.StringVar(value="Motor C: ----°")
        self.pos_label = tk.StringVar(value="Pos: ----, ----")
        self.fps_label = tk.StringVar(value="FPS: ---")
        
        # use fixed width labels with monospaced-like formatting
        label_width = 15
        
        tk.Label(left,
                 textvariable=self.motor_a,
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Liberation Mono', 11),
                 width=label_width,
                 ).grid(row=0, sticky="w", padx=3, pady=2)
        
        tk.Label(left,
                 textvariable=self.motor_b,
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Liberation Mono', 11),
                 width=label_width,
                 ).grid(row=1, sticky="w", padx=3, pady=2)
        
        tk.Label(left,
                 textvariable=self.motor_c,
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Liberation Mono', 11),
                 width=label_width,
                 ).grid(row=2, sticky="w", padx=3, pady=2)
        
        tk.Frame(left,
                 bg=self.colors['fg'],
                 height=1
                 ).grid(row=3, sticky="ew", padx=3, pady=3)
        
        tk.Label(left,
                 textvariable=self.pos_label,
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Liberation Mono', 11),
                 ).grid(row=4, sticky="w", padx=3, pady=2)
        
        tk.Label(left,
                 textvariable=self.fps_label,
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Liberation Mono', 14),
                 ).grid(row=5, sticky="w", padx=3, pady=2)
        
        # right column - motor controls
        right = tk.LabelFrame(status_container,
                              text="Motor Controls",
                              bg=self.colors['bg'],
                              fg=self.colors['fg'],
                              font=('Arial', 12, 'bold')
                              )
        right.grid(row=0, column=1, sticky="nsew", padx=2)
        right.grid_columnconfigure(0, weight=1)
        
        home_btn = tk.Button(right,
                  text="Home Motors",
                  bg=self.colors['button'],
                  fg=self.colors['fg'],
                  font=('Arial', 12, 'bold'),
                  height=2,
                  command=self.home_motors
                  )
        home_btn.grid(row=0, sticky="nsew", padx=10, pady=5)
        self.remove_highlight(home_btn)
        
        balance_btn = tk.Button(right,
                  text="Balance Plat.",
                  bg=self.colors['button'],
                  fg=self.colors['fg'],
                  font=('Arial', 12, 'bold'),
                  height=2,
                  command=self.balance_platform
                  )
        balance_btn.grid(row=1, sticky="nsew", padx=10, pady=5)
        self.remove_highlight(balance_btn)
        
        # configure row weights for proper spacing
        self.main_page.grid_rowconfigure(0, weight=0)  # checkbox - fixed
        self.main_page.grid_rowconfigure(1, weight=0)  # target frame - fixed
        self.main_page.grid_rowconfigure(2, weight=1)  # status area - expands
    
    
    
    
    # ==================== PATHS SELECTION PAGE ====================
    
    def show_paths_selection_page(self):
        """Show the path selection menu."""
        self.paths_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.paths_page.grid(row=0, column=0, sticky="nsew")
        self.paths_page.grid_columnconfigure(0, weight=1)
        self.paths_page.grid_rowconfigure(0, weight=1)
        
        # define paths with icon only
        paths = [
            ("□", "square", self.colors['accent']),
            ("○", "circle", self.colors['accent']),
            ("∞", "infinity", self.colors['accent']),
            ("△", "triangle", self.colors['accent']),
            ("─", "line", self.colors['accent']),
            ("✋", "free", self.colors['warning'])
        ]
        
        # create 2x3 grid
        for i, (icon, path_type, color) in enumerate(paths):
            row = i // 2
            col = i % 2
            
            btn = tk.Button(self.paths_page,
                            text=icon,
                            bg=color,
                            fg='white',
                            font=('Arial', 60, 'bold'),
                            command=lambda pt=path_type: self.show_path_config_page(pt))
            btn.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)
            
            self.paths_page.grid_columnconfigure(col, weight=1)
            self.paths_page.grid_rowconfigure(row, weight=1)
            self.remove_highlight(btn)
    
    
    
    def show_path_config_page(self, path_type):
        """Show configuration page for selected path."""
        if self.current_path_page:
            self.current_path_page.destroy()
        
        if path_type == "square":
            self.current_path_page = SquarePathPage(self.page_container, self, 
                                                    lambda: self.show_page("paths"))
        elif path_type == "circle":
            self.current_path_page = CirclePathPage(self.page_container, self,
                                                    lambda: self.show_page("paths"))
        elif path_type == "infinity":
            self.current_path_page = InfinityPathPage(self.page_container, self,
                                                      lambda: self.show_page("paths"))
        elif path_type == "triangle":
            self.current_path_page = TrianglePathPage(self.page_container, self,
                                                      lambda: self.show_page("paths"))
        elif path_type == "line":
            self.current_path_page = LinePathPage(self.page_container, self,
                                                  lambda: self.show_page("paths"))
        elif path_type == "free":
            self.current_path_page = FreePathPage(self.page_container, self,
                                                  lambda: self.show_page("paths"))
        
        if self.current_path_page:
            self.current_path_page.create()
    
    
    
    
    # ==================== CALIBRATION PAGE (parent) ====================
    
    def show_calibration_page(self):
        """Show the calibration menu with ball and motor options."""
        self.calibration_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.calibration_page.grid(row=0, column=0, sticky="nsew")
        self.calibration_page.grid_columnconfigure(0, weight=1)
        self.calibration_page.grid_rowconfigure(0, weight=1)
        
        # 2x1 grid
        for i, (text, page_name, color) in enumerate([("BALL", "ball", self.colors['accent']),
                                                       ("ROBOT", "motor", self.colors['accent'])]):
            btn = tk.Button(self.calibration_page,
                            text=text,
                            bg=color,
                            fg='white',
                            font=('Arial', 40, 'bold'), #60
                            command=lambda p=page_name: self.show_page(p))
            btn.grid(row=i, column=0, sticky="nsew", padx=20, pady=20)  # pads 10
            self.calibration_page.grid_rowconfigure(i, weight=1)
            self.remove_highlight(btn)
    
    
    
    
    # ==================== MOTOR PAGE (parent) ====================
    
    def show_motor_page(self):
        """Show the motor settings menu with homing, plat, pid options."""
        self.motor_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.motor_page.grid(row=0, column=0, sticky="nsew")
        self.motor_page.grid_columnconfigure(0, weight=1)
        self.motor_page.grid_rowconfigure(0, weight=0)  # back button
        self.motor_page.grid_rowconfigure(1, weight=0)  # title
        self.motor_page.grid_rowconfigure(2, weight=1)  # buttons
        self.motor_page.grid_rowconfigure(3, weight=0)  # spacer
        
        # back button (using container frame like ball page)
        back_container = tk.Frame(self.motor_page, bg=self.colors['bg'])
        back_container.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        back_container.grid_columnconfigure(0, weight=1)
        
        back_btn = tk.Button(back_container, text="◄ BACK", bg=self.colors['button'],
                            fg=self.colors['fg'], font=('Arial', 14, 'bold'),
                            command=lambda: self.show_page("calibration"))
        back_btn.grid(row=0, column=0, sticky="w")
        self.remove_highlight(back_btn)
        
        # title
        title_label = tk.Label(self.motor_page, text="Robot Settings", bg=self.colors['bg'],
                               fg=self.colors['fg'], font=('Arial', 18, 'bold'))
        title_label.grid(row=1, pady=(10, 0))
        
        # 3x1 grid for buttons
        buttons_frame = tk.Frame(self.motor_page, bg=self.colors['bg'])
        buttons_frame.grid(row=2, sticky="nsew", padx=20, pady=10)
        buttons_frame.grid_columnconfigure(0, weight=1)
        
        for i, (text, page_name, color) in enumerate([("HOMING", "homing", self.colors['button']), #self.colors['warning']
                                                      ("PLATFORM", "plat", self.colors['button']),
                                                      ("PID", "pid", self.colors['button'])]):
            btn = tk.Button(buttons_frame,
                            text=text,
                            bg=color,
                            fg='white',
                            font=('Arial', 24, 'bold'),
                            height=2,
                            command=lambda p=page_name: self.show_page(p))
            btn.grid(row=i, column=0, sticky="ew", pady=8)
            buttons_frame.grid_rowconfigure(i, weight=1)
            self.remove_highlight(btn)
        
        # spacer
        spacer = tk.Frame(self.motor_page, bg=self.colors['bg'])
        spacer.grid(row=3, sticky="nsew")
    
    
    
    # ==================== HOMING PAGE ====================
    
    def show_homing_page(self):
        """Show sensorless homing parameters page."""
        self.homing_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.homing_page.grid(row=0, column=0, sticky="nsew")
        self.homing_page.grid_columnconfigure(0, weight=1)
        self.homing_page.grid_rowconfigure(3, weight=1)  # spacer
        
        # header with back button and info button
        header_container = tk.Frame(self.homing_page, bg=self.colors['bg'])
        header_container.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        header_container.grid_columnconfigure(0, weight=1)
        header_container.grid_columnconfigure(1, weight=0)
        
        back_btn = tk.Button(header_container, text="◄ BACK", bg=self.colors['button'],
                            fg=self.colors['fg'], font=('Arial', 14, 'bold'),
                            command=lambda: self.show_page("motor"))
        back_btn.grid(row=0, column=0, sticky="w")
        self.remove_highlight(back_btn)
        
        info_btn = tk.Button(header_container, text="INFO", bg=self.colors['accent'],
                            fg='white', font=('Arial', 16, 'bold'), width=4,
                            command=self.show_homing_info)
        info_btn.grid(row=0, column=1, sticky="e")
        self.remove_highlight(info_btn)
        
        # title
        title_label = tk.Label(self.homing_page, text="Sensorless Homing", bg=self.colors['bg'],
                               fg=self.colors['fg'], font=('Arial', 18, 'bold'))
        title_label.grid(row=1, pady=(5, 5))
        
        # stallguard reference value
        sg_frame = tk.LabelFrame(self.homing_page,
                                 text="StallGuard Sensitivity",
                                 bg=self.colors['bg'],
                                 fg=self.colors['fg'],
                                 font=('Arial', 12, 'bold'))
        sg_frame.grid(row=2, sticky="ew", padx=5, pady=5)
        sg_frame.grid_columnconfigure(1, weight=1)
        
        tk.Label(sg_frame,
                 text="SG Factor:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 12)
                 ).grid(row=0, column=0, sticky="w", padx=10, pady=5)
        
        self.sg_factor = tk.IntVar(value=self.system.controller.HOMING_SG_K_FACTOR)
        sg_spinbox = tk.Spinbox(sg_frame,
                                from_=0,
                                to=255,
                                textvariable=self.sg_factor,
                                width=6,
                                font=('Arial', 28),
                                justify='center',
                                command=self.apply_sg_factor
                                )
        sg_spinbox.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        
        # stallguard shift values (triangular layout)
        sg_shift_frame = tk.LabelFrame(self.homing_page,
                                       text="StallGuard Shift (per motor)",
                                       bg=self.colors['bg'],
                                       fg=self.colors['fg'],
                                       font=('Arial', 12, 'bold'))
        sg_shift_frame.grid(row=3, sticky="ew", padx=5, pady=5)
        sg_shift_frame.grid_columnconfigure(0, weight=1)
        sg_shift_frame.grid_columnconfigure(1, weight=1)
        sg_shift_frame.grid_columnconfigure(2, weight=1)
        
        # motor a (center, top position)
        a_frame = tk.Frame(sg_shift_frame, bg=self.colors['bg'])
        a_frame.grid(row=0, column=0, columnspan=3, pady=5)
        
        tk.Label(a_frame,
                 text="A:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10))
        
        self.sg_shift_a = tk.IntVar(value=self.system.controller.HOMING_SG_SHIFTS.get('A', 0))
        shift_a_spin = tk.Spinbox(a_frame,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.sg_shift_a,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_sg_shift('A')
                                  )
        shift_a_spin.pack(side=tk.LEFT)
        
        # motors b and c (bottom row, side by side)
        bc_frame = tk.Frame(sg_shift_frame, bg=self.colors['bg'])
        bc_frame.grid(row=1, column=0, columnspan=3, pady=(5, 5))
        
        # motor b (left)
        b_container = tk.Frame(bc_frame, bg=self.colors['bg'])
        b_container.pack(side=tk.LEFT, expand=True, padx=10, pady=(10, 5))
        
        tk.Label(b_container,
                 text="B:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10), pady=(10, 5))
        
        self.sg_shift_b = tk.IntVar(value=self.system.controller.HOMING_SG_SHIFTS.get('B', 0))
        shift_b_spin = tk.Spinbox(b_container,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.sg_shift_b,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_sg_shift('B')
                                  )
        shift_b_spin.pack(side=tk.LEFT)
        
        # motor c (right)
        c_container = tk.Frame(bc_frame, bg=self.colors['bg'])
        c_container.pack(side=tk.RIGHT, expand=True, padx=10, pady=(10, 5))
        
        tk.Label(c_container,
                 text="C:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10))
        
        self.sg_shift_c = tk.IntVar(value=self.system.controller.HOMING_SG_SHIFTS.get('C', 0))
        shift_c_spin = tk.Spinbox(c_container,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.sg_shift_c,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_sg_shift('C')
                                  )
        shift_c_spin.pack(side=tk.LEFT)
        
        # spacer
        spacer = tk.Frame(self.homing_page, bg=self.colors['bg'])
        spacer.grid(row=4, sticky="nsew")
        self.homing_page.grid_rowconfigure(4, weight=1)
    
    
    
    def show_homing_info(self):
        """Show information window about sensorless homing."""
        info_window = tk.Toplevel(self)
        info_window.title("Sensorless Homing Guide")
        info_window.configure(bg=self.colors['bg'])
        
        # set size and position
        width = 700
        height = 400
        screen_width = info_window.winfo_screenwidth()
        screen_height = info_window.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2
        info_window.geometry(f"{width}x{height}+{x}+{y}")
        
        # make it modal (stay on top of main window)
        info_window.transient(self)
        info_window.grab_set()
        
        # container
        main_frame = tk.Frame(info_window, bg=self.colors['bg'])
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # scrollable text area
        text_frame = tk.Frame(main_frame, bg=self.colors['bg'])
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        # create scrollbar first
        scrollbar = tk.Scrollbar(text_frame, bg=self.colors['button'], width=25)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # create text widget with scrollbar connection
        text_widget = tk.Text(text_frame, bg=self.colors['info_bar'], fg=self.colors['fg'],
                              font=('Arial', 12), wrap=tk.WORD, padx=15, pady=15,
                              relief=tk.SUNKEN, bd=1,
                              yscrollcommand=scrollbar.set)  # connect to scrollbar here
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # configure scrollbar to control text widget
        scrollbar.config(command=text_widget.yview)
        
        # information content (remove leading spaces that cause odd formatting)
        info_text = """\
        Sensorless homing uses the StallGuard feature of the TMC2209 stepper drivers
        to detect when a motor hits a mechanical stop, without needing physical end
        stop switches.

        SG FACTOR (StallGuard Sensitivity)
        ----------------------------------
        The SG Factor controls how sensitive the StallGuard detection is:
        - Lower values  = less sensitive (requires more torque to detect)
        - Higher values = more sensitive (detects stalls earlier)

        Start with a value around 45 and adjust if:
        - Homing stops too early (false detection): decrease the value
        - Motors bounce back at the stop (skip steps): increase the value

        SG SHIFT (Per-Motor Fine Tuning)
        --------------------------------
        Due to mechanical differences between motors (friction, alignment, etc.),
        each motor may trigger StallGuard at slightly different positions.
        The SG Shift values allow per-motor adjustment:
        - Positive shift: motor stops earlier (needs less torque)
        - Negative shift: motor stops later (needs more torque)

        Use these to synchronize all three motors at the same mechanical home
        position. Watch the platform during homing, the platform support should be
        pulled against the motors yet the motor should not skip steps (bounce back).
        """
        
        text_widget.insert(tk.END, info_text)
        text_widget.config(state=tk.DISABLED)  # read-only
        
#         # close button
#         close_btn = tk.Button(main_frame, text="CLOSE", bg=self.colors['button'],
#                              fg=self.colors['fg'], font=('Arial', 12, 'bold'),
#                              command=info_window.destroy)
#         close_btn.pack(pady=(10, 0))
        self.remove_highlight(close_btn)
    
    
    
    def apply_sg_factor(self):
        """Apply stallguard factor value to the controller."""
        if self.is_closing:
            return
        self.system.controller.HOMING_SG_K_FACTOR = self.sg_factor.get()
        self.system.save_homing_settings()
        print(f"StallGuard factor set to {self.sg_factor.get()}")
    
    
    
    def apply_sg_shift(self, motor):
        """Apply stallguard shift value for a specific motor."""
        if self.is_closing:
            return
        
        shifts = self.system.controller.HOMING_SG_SHIFTS.copy()
        if motor == 'A':
            shifts['A'] = self.sg_shift_a.get()
        elif motor == 'B':
            shifts['B'] = self.sg_shift_b.get()
        elif motor == 'C':
            shifts['C'] = self.sg_shift_c.get()
        
        self.system.controller.HOMING_SG_SHIFTS = shifts
        self.system.save_homing_settings()
        print(f"Motor {motor} StallGuard shift set to {shifts[motor]}")
    
    
    
    
    # ==================== BALL CALIBRATION PAGE ====================
    
    def show_ball_page(self):
        """Show ball calibration page."""
        self.ball_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.ball_page.grid(row=0, column=0, sticky="nsew")
        self.ball_page.grid_columnconfigure(0, weight=1)
        self.ball_page.grid_rowconfigure(3, weight=1)  # spacer at bottom
        
        # back button (aligned left, same as motor page)
        back_container = tk.Frame(self.ball_page, bg=self.colors['bg'])
        back_container.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        back_container.grid_columnconfigure(0, weight=1)
        
        back_btn = tk.Button(back_container, text="◄ BACK", bg=self.colors['button'],
                            fg=self.colors['fg'], font=('Arial', 14, 'bold'),
                            command=lambda: self.show_page("calibration"))
        back_btn.grid(row=0, column=0, sticky="w")
        self.remove_highlight(back_btn)
        
        # title
        title_label = tk.Label(self.ball_page, text="Ball Settings", bg=self.colors['bg'],
                               fg=self.colors['fg'], font=('Arial', 18, 'bold'))
        title_label.grid(row=1, pady=(10, 0))
        
        # ball diameter (moved down one row)
        diameter_frame = tk.LabelFrame(self.ball_page,
                                       text="Ball diameter (mm)",
                                       bg=self.colors['bg'],
                                       fg=self.colors['fg'],
                                       font=('Arial', 14, 'bold')
                                       )
        diameter_frame.grid(row=2, sticky="ew", padx=5, pady=(10, 5))
        diameter_frame.grid_columnconfigure(0, weight=1)
        
        self.ball_diam = tk.IntVar(value=self.system.camera.ball_mm)
        spinbox_ball_mm = tk.Spinbox(diameter_frame,
                             from_=10,
                             to=80,
                             textvariable=self.ball_diam,
                             width=4,
                             font=('Arial', 28),
                             justify='center',
                             command=lambda: self.apply_ball_mm()
                             )
        spinbox_ball_mm.grid(sticky="n", pady=10)
        
        # ball color calibration (position maintained)
        color = tk.LabelFrame(self.ball_page,
                              text="Ball Color Calibration (HSV)",
                              bg=self.colors['bg'],
                              fg=self.colors['fg'],
                              font=('Arial', 14, 'bold')
                              )
        color.grid(row=3, sticky="ew", padx=5, pady=(5,10))
        color.grid_columnconfigure(0, weight=1)
        
        self.hsv_display = tk.Text(color,
                                   height=2,
                                   bg=self.colors['bg'],
                                   fg=self.colors['fg'],
                                   font=('Courier', 20),
                                   relief=tk.FLAT,
                                   highlightthickness=0
                                   )
        self.hsv_display.grid(row=0, column=0, columnspan=2, padx=10, pady=7)
        
        auto_btn = tk.Button(color,
                             text="AUTO",
                             bg=self.colors['accent'],
                             fg='white',
                             font=('Arial', 12, 'bold'),
                             width=11,
                             command=self.auto_calibrate
                             )
        auto_btn.grid(row=1, column=0, pady=5, sticky="ew", padx=10)
        self.remove_highlight(auto_btn)
        
        manual_btn = tk.Button(color,
                               text="MANUAL",
                               bg=self.colors['button'],
                               fg=self.colors['fg'],
                               font=('Arial', 12, 'bold'),
                               width=13,
                               command=self.manual_calibrate
                               )
        manual_btn.grid(row=1, column=1, pady=5, sticky="ew", padx=10)
        self.remove_highlight(manual_btn)
        
        self.update_hsv_display()
    
    
    
    
    # ==================== PLATFORM CALIBRATION PAGE ====================
    
    def show_plat_page(self):
        """Show platform calibration page with motor shift adjustments and manual tilt control."""
        self.plat_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.plat_page.grid(row=0, column=0, sticky="nsew")
        self.plat_page.grid_columnconfigure(0, weight=1)
        self.plat_page.grid_rowconfigure(2, weight=1)
        
        # motor shift calibration - triangular layout
        shift_frame = tk.LabelFrame(self.plat_page,
                                    text="Platform levelling (steps)",
                                    bg=self.colors['bg'],
                                    fg=self.colors['fg'],
                                    font=('Arial', 12, 'bold')
                                    )
        shift_frame.grid(row=0, sticky="ew", padx=5, pady=5)
        shift_frame.grid_columnconfigure(0, weight=1)
        shift_frame.grid_columnconfigure(1, weight=1)
        shift_frame.grid_columnconfigure(2, weight=1)

        # row 0: Motor A (center, top position in triangle)
        a_frame = tk.Frame(shift_frame, bg=self.colors['bg'])
        a_frame.grid(row=0, column=0, columnspan=3, pady=5)
        
        tk.Label(a_frame,
                 text="A:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10))
        
        self.shift_a = tk.IntVar(value=self.system.controller.BA_SHIFT_A)
        shift_a_spin = tk.Spinbox(a_frame,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.shift_a,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_motor_shift_delta('A')
                                  )
        shift_a_spin.pack(side=tk.LEFT)
        
        # row 1: Motors B and C (bottom row, side by side)
        bc_frame = tk.Frame(shift_frame, bg=self.colors['bg'])
        bc_frame.grid(row=1, column=0, columnspan=3, pady=(5,5))
        
        # motor B (left)
        b_container = tk.Frame(bc_frame, bg=self.colors['bg'])
        b_container.pack(side=tk.LEFT, expand=True, padx=10, pady=(10, 5))
        
        tk.Label(b_container,
                 text="B:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10), pady=(10, 5))
        
        self.shift_b = tk.IntVar(value=self.system.controller.BA_SHIFT_B)
        shift_b_spin = tk.Spinbox(b_container,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.shift_b,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_motor_shift_delta('B')
                                  )
        shift_b_spin.pack(side=tk.LEFT)
        
        # motor C (right)
        c_container = tk.Frame(bc_frame, bg=self.colors['bg'])
        c_container.pack(side=tk.RIGHT, expand=True, padx=10, pady=(10, 5))
        
        tk.Label(c_container,
                 text="C:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 16, 'bold')
                 ).pack(side=tk.LEFT, padx=(0, 10))
        
        self.shift_c = tk.IntVar(value=self.system.controller.BA_SHIFT_C)
        shift_c_spin = tk.Spinbox(c_container,
                                  from_=-50,
                                  to=50,
                                  textvariable=self.shift_c,
                                  width=4,
                                  font=('Arial', 28),
                                  justify='center',
                                  command=lambda: self.apply_motor_shift_delta('C')
                                  )
        shift_c_spin.pack(side=tk.LEFT)
        
        # manual Tilt Control
        tilt = tk.LabelFrame(self.plat_page,
                             text="Manual Tilt Control (degrees)",
                             bg=self.colors['bg'],
                             fg=self.colors['fg'],
                             font=('Arial', 12, 'bold')
                             )
        tilt.grid(row=1, sticky="ew", padx=5, pady=5)
        tilt.grid_columnconfigure(1, weight=1)
        
        max_tilt = self.system.controller.MAX_TILT_TENTHS / 10
        
        # direction slider
        tk.Label(tilt,
                 text="Direction:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 12)
                 ).grid(row=0, column=0, sticky="w", padx=10, pady=5)
        
        self.dir_slider = tk.Scale(tilt,
                                   from_=0,
                                   to=180,
                                   orient=tk.HORIZONTAL,
                                   bg=self.colors['bg'],
                                   fg=self.colors['fg'],
                                   troughcolor=self.colors['button'],
                                   sliderlength=25,
                                   length=200,
                                   font=('Arial', 16),
                                   command=self.on_dir_change
                                   )
        self.dir_slider.set(90)
        self.dir_slider.grid(row=0, column=1, sticky="ew", padx=10, pady=5)
        
        # tilt angle slider
        tk.Label(tilt,
                 text="Tilt Angle:",
                 bg=self.colors['bg'],
                 fg=self.colors['fg'],
                 font=('Arial', 12)
                 ).grid(row=1, column=0, sticky="w", padx=10, pady=5)
        
        self.tilt_slider = tk.Scale(tilt,
                                    from_=-max_tilt,
                                    to=max_tilt,
                                    orient=tk.HORIZONTAL,
                                    bg=self.colors['bg'],
                                    fg=self.colors['fg'],
                                    troughcolor=self.colors['button'],
                                    sliderlength=25,
                                    length=200,
                                    font=('Arial', 16),
                                    command=self.on_tilt_change
                                    )
        self.tilt_slider.set(0)
        self.tilt_slider.grid(row=1, column=1, sticky="ew", padx=10, pady=5)
        
        # level platform button
        level_btn = tk.Button(tilt,
                              text="⟳ LEVEL PLATFORM",
                              bg=self.colors['warning'],
                              fg='black',
                              font=('Arial', 12, 'bold'),
                              command=self.level_platform_with_shifts
                              )
        level_btn.grid(row=2, columnspan=2, sticky="ew", padx=10, pady=(10, 5))
        self.remove_highlight(level_btn)
        
        # spacer to push content up
        spacer = tk.Frame(self.plat_page, bg=self.colors['bg'])
        spacer.grid(row=2, sticky="nsew")
        self.plat_page.grid_rowconfigure(2, weight=1)
    
    
    
    
    # ==================== PID TUNING PAGE ====================
    
    def show_pid_page(self):
        """Show PID tuning page with integral parameters in 2x2 grid."""
        self.pid_page = StaticFrame(self.page_container, bg=self.colors['bg'])
        self.pid_page.grid(row=0, column=0, sticky="nsew")
        self.pid_page.grid_columnconfigure(0, weight=1)
        self.pid_page.grid_rowconfigure(0, weight=0)  # PID sliders - fixed
        self.pid_page.grid_rowconfigure(1, weight=0)  # integral params - fixed
        self.pid_page.grid_rowconfigure(2, weight=1)  # spacer
        
        # ==================== PID SLIDERS SECTION ====================
        pid_frame = tk.LabelFrame(self.pid_page,
                                  text="PID Gains",
                                  bg=self.colors['bg'],
                                  fg=self.colors['fg'],
                                  font=('Arial', 12, 'bold'))
        pid_frame.grid(row=0, sticky="ew", padx=5, pady=(0,5))
        pid_frame.grid_columnconfigure(1, weight=1)
        
        # Kp
        tk.Label(pid_frame, text="Kp:", bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 12)).grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.kp = tk.DoubleVar(value=self.system.controller.k_p)
        tk.Scale(pid_frame, from_=0.0, to=3.0, resolution=0.1,
                 orient=tk.HORIZONTAL, variable=self.kp,
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 length=200,
                 command=lambda v: setattr(self.system.controller, 'k_p', self.kp.get())
                 ).grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        
        # Ki
        tk.Label(pid_frame, text="Ki:", bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 12)).grid(row=1, column=0, sticky="w", padx=10, pady=5)
        self.ki = tk.DoubleVar(value=self.system.controller.k_i)
        tk.Scale(pid_frame, from_=0.0, to=2.0, resolution=0.1,
                 orient=tk.HORIZONTAL, variable=self.ki,
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 length=200,
                 command=lambda v: setattr(self.system.controller, 'k_i', self.ki.get())
                 ).grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        
        # Kd
        tk.Label(pid_frame, text="Kd:", bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 12)).grid(row=2, column=0, sticky="w", padx=10, pady=5)
        self.kd = tk.DoubleVar(value=self.system.controller.k_d)
        tk.Scale(pid_frame, from_=0.0, to=2.0, resolution=0.1,
                 orient=tk.HORIZONTAL, variable=self.kd,
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 length=200,
                 command=lambda v: setattr(self.system.controller, 'k_d', self.kd.get())
                 ).grid(row=2, column=1, sticky="ew", padx=5, pady=(5,2))
        
        # ==================== INTEGRAL PARAMETERS SECTION (2x2 GRID) ====================
        integral_frame = tk.LabelFrame(self.pid_page,
                                       text="Integral Control ( when Ki > 0 )",
                                       bg=self.colors['bg'],
                                       fg=self.colors['fg'],
                                       font=('Arial', 12, 'bold'))
        integral_frame.grid(row=1, sticky="ew", padx=5, pady=(5,5))
        
        # configure 2x2 grid
        for i in range(2):
            integral_frame.grid_columnconfigure(i, weight=1)
        
        # parameter 1: Integral Zone (ActiveZone) - top-left
        zone_frame = tk.Frame(integral_frame, bg=self.colors['bg'])
        zone_frame.grid(row=0, column=0, padx=5, pady=2, sticky="nsew")

        tk.Label(zone_frame, text="ActiveZone (mm)",
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 11, 'bold')).pack()

        self.integral_zone = tk.IntVar(value=self.system.controller.INTEGRAL_ZONE_TENTHS // 10)
        zone_spinbox = tk.Spinbox(zone_frame, from_=10, to=100,
                                  textvariable=self.integral_zone,
                                  width=6, font=('Arial', 28), justify='center')
        zone_spinbox.pack(pady=2)
        
        # parameter 2: Deadzone Distance - top-right
        dz_frame = tk.Frame(integral_frame, bg=self.colors['bg'])
        dz_frame.grid(row=0, column=1, padx=5, pady=2, sticky="nsew")

        tk.Label(dz_frame, text="DeadZone (mm)",
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 11, 'bold')).pack()

        self.deadzone = tk.IntVar(value=self.system.controller.DEADZONE_TENTHS // 10)
        dz_spinbox = tk.Spinbox(dz_frame, from_=0, to=30,
                                textvariable=self.deadzone,
                                width=6, font=('Arial', 28), justify='center')
        dz_spinbox.pack(pady=2)
        
        # parameter 3: Max Integral (Windup Limit) - bottom-left
        windup_frame = tk.Frame(integral_frame, bg=self.colors['bg'])
        windup_frame.grid(row=1, column=0, padx=5, pady=2, sticky="nsew")

        tk.Label(windup_frame, text="Max Integral (#)",
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 11, 'bold')).pack()

        self.max_integral = tk.IntVar(value=self.system.controller.MAX_INTEGRAL)
        windup_spinbox = tk.Spinbox(windup_frame, from_=500, to=10000, increment=100,
                                    textvariable=self.max_integral,
                                    width=6, font=('Arial', 28), justify='center')
        windup_spinbox.pack(pady=2)

        # parameter 4: Deadzone Counter - bottom-right
        counter_frame = tk.Frame(integral_frame, bg=self.colors['bg'])
        counter_frame.grid(row=1, column=1, padx=5, pady=2, sticky="nsew")

        tk.Label(counter_frame, text="DZ Counter (#)",
                 bg=self.colors['bg'], fg=self.colors['fg'],
                 font=('Arial', 11, 'bold')).pack()

        self.deadzone_counter = tk.IntVar(value=self.system.controller.DEADZONE_COUNTER_THR)
        counter_spinbox = tk.Spinbox(counter_frame, from_=5, to=50,
                                     textvariable=self.deadzone_counter,
                                     width=6, font=('Arial', 28), justify='center')
        counter_spinbox.pack(pady=2)
        
        
        
        # update controller when spinbox changes
        def update_integral_zone(*args):
            self.system.controller.INTEGRAL_ZONE_TENTHS = self.integral_zone.get() * 10
        self.integral_zone.trace_add('write', update_integral_zone)
        
        
        
        def update_deadzone(*args):
            self.system.controller.DEADZONE_TENTHS = self.deadzone.get() * 10
        self.deadzone.trace_add('write', update_deadzone)
        
        
        
        def update_max_integral(*args):
            self.system.controller.MAX_INTEGRAL = self.max_integral.get()
        self.max_integral.trace_add('write', update_max_integral)
        
        
        
        def update_deadzone_counter(*args):
            self.system.controller.DEADZONE_COUNTER_THR = self.deadzone_counter.get()
        self.deadzone_counter.trace_add('write', update_deadzone_counter)
        
        spacer = tk.Frame(self.pid_page, bg=self.colors['bg'])
        spacer.grid(row=2, sticky="nsew")
    
    
    
    
    # ==================== VIDEO AND STATUS ====================
    
    def update_video(self):
        """
        Update video feed with optimized flicker-free method.
        Noted the flicker is not solved when sharing images to the VNC Viewer.
        """
        if not self.video_running or self.is_closing:
            return
        
        self.frame_counter += 1
        
        try:
            if self.frame_counter > self.frame_skip:
                frame = self.system.camera.get_annotated_frame() if hasattr(self.system.camera, 'get_annotated_frame') else self.system.camera.latest_frame
                self.frame_counter = 0
                
                if frame is not None:
                    if self.use_optimized_canvas and isinstance(self.video_display, OptimizedCanvas):
                        self.video_display.update_image(frame)
                    else:
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        img = Image.fromarray(frame_rgb)
                        
                        if hasattr(self.video_display, 'winfo_width'):
                            w = self.video_display.winfo_width()
                            h = self.video_display.winfo_height()
                            if w > 10 and h > 10:
                                img = img.resize((w, h), Image.Resampling.LANCZOS)
                        
                        imgtk = ImageTk.PhotoImage(image=img)
                        self.video_display.imgtk = imgtk
                        self.video_display.configure(image=imgtk)
        
        except Exception as e:
            pass
        
        if not self.is_closing:
            self.after(self.video_update_interval_ms, self.update_video)
    
    
    
    def update_cpu_temperature(self):
        """Update CPU temperature display."""
        if self.is_closing:
            return
        
        try:
            # read CPU temperature from sysfs
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_raw = f.read().strip()
                temp_c = round(float(temp_raw) / 1000)
            
            # format with one decimal place
            temp_text = f"{temp_c}°C"
            
            # color code based on temperature
            if temp_c < 65:
                color = self.colors['success']  # green - cool
            elif temp_c < 70:
                color = self.colors['warning']  # yellow - warm
            else:
                color = self.colors['error']    # red - hot
            
            self.temp_value.config(text=temp_text, fg=color)
            
        except FileNotFoundError:
            # not on raspberry pi or thermal zone not available
            self.temp_value.config(text="N/A", fg=self.colors['fg'])
        except Exception as e:
            # other errors - just show error indicator
            self.temp_value.config(text="ERR", fg=self.colors['error'])
        
        # schedule next update if not closing
        if not self.is_closing:
            self.after(self.temp_update_interval_ms, self.update_cpu_temperature)
    
    
    
    def update_status(self):
        """Update status displays."""
        if self.is_closing:
            return
            
        try:
            self.fps_label.set(f"FPS: {self.system.camera.fps}")
            
            x, y = self.system.camera.get_ball_position()
            if x != -9999 and y != -9999:
                self.ball_status.config(text="Ball: DETECTED", fg=self.colors['success'])
                self.pos_label.set(f"Pos: {x:3d}, {y:3d}")
            else:
                self.ball_status.config(text="Ball: LOST", fg=self.colors['fg'])
                self.pos_label.set("Pos: ----, ----")
            
            angles = self.system.controller.current_angles
            
            # always 5 chars total to keep the column width fixed
            self.motor_a.set(f"Motor A: {angles.A/10:>5.1f}°")
            self.motor_b.set(f"Motor B: {angles.B/10:>5.1f}°")
            self.motor_c.set(f"Motor C: {angles.C/10:>5.1f}°")
            
            # system status update (bottom row)
            self.refresh_system_status()
            
        except Exception as e:
            pass
        
        if not self.is_closing:
            self.after(self.status_update_interval_ms, self.update_status)
    
    
    
    def update_hsv_display(self):
        """Update HSV display."""
        if hasattr(self, 'hsv_display') and not self.is_closing:
            lower = self.system.camera.lower_color
            upper = self.system.camera.upper_color
            self.hsv_display.delete(1.0, tk.END)
            self.hsv_display.insert(1.0, f"Lower: [{lower[0]:3d}, {lower[1]:3d}, {lower[2]:3d}]\nUpper: [{upper[0]:3d}, {upper[1]:3d}, {upper[2]:3d}]")
    
    
    
    def refresh_system_status(self):
        """Refresh system status based on actual state."""
        if self.is_closing:
            return
        
        # check motor state first
        try:
            if not self.system.mc.motors_enabled:
                self.system_status.config(text="System: MOTORS OFF", fg=self.colors['warning'])
                return
        except:
            pass
        
        # check balance state
        if hasattr(self, 'auto_balance_var') and self.auto_balance_var.get():
            if hasattr(self.system.controller, 'at_balance') and self.system.controller.at_balance:
                self.system_status.config(text="System: BALANCED", fg=self.colors['success'])
            else:
                self.system_status.config(text="System: BALANCING", fg=self.colors['accent'])
        else:
            self.system_status.config(text="System: READY", fg=self.colors['success'])
    
    
    
    # ==================== HELPER FUNCTIONS ====================

    def _ensure_auto_balance(self):
        """Enable auto-balance if it's currently off."""
        if not self.auto_balance_var.get():
            if self.verbose:
                print("Auto-balance was OFF - enabling now...")
            self.auto_balance_var.set(True)
            self.toggle_auto_balance()
            # small delay to let auto-balance stabilize
            sleep(0.5)
    
    
    
    # ==================== PATH CONTROL FUNCTIONS ====================
    
    def start_square_path(self, side, repeats):
        def run():
            self.system.square_path(side_mm=side, repeats=repeats)
        self._ensure_auto_balance()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def start_circle_path(self, radius, repeats, direction):
        def run():
            self.system.circle_path(radius_mm=radius, repeats=repeats, direction=direction)
        self._ensure_auto_balance()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def start_infinity_path(self, size, speed, repeats):
        def run():
            self.system.infinity_path(size_mm=size, speed_factor=speed, repeats=repeats)
        self._ensure_auto_balance()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def start_triangle_path(self, side, orientation, repeats):
        def run():
            self.system.triangle_path(side_mm=side, orientation=orientation, repeats=repeats,
                                      tolerance_mm=40, settle_time_ms=400)
        self._ensure_auto_balance()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def start_line_path(self, length, angle, repeats):
        def run():
            self.system.line_path(length_mm=length, angle_deg=angle, repeats=repeats,
                                  tolerance_mm=40, settle_time_ms=400)
        self._ensure_auto_balance()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def start_free_path(self, path_points=None, mode="continuous"):
        if mode == "continuous":
            # continuous finger tracking mode - handled by on_touch callback
            self._ensure_auto_balance()
            pass
        elif mode == "playback" and path_points:
            def run():
                for x, y in path_points:
                    self.update_target(x, y)
                    sleep(0.05)  # adjust speed as needed
            self._ensure_auto_balance()
            threading.Thread(target=run, daemon=True).start()
    
    
    
    def stop_current_path(self):
        """Stop the currently running path."""
        if hasattr(self.system, 'stop_current_path') or \
           (hasattr(self, 'auto_balance_var') and not self.auto_balance_var.get()):
            
            self.system.stop_current_path()
            # force clear trajectory to show quadrants
            if hasattr(self.system, 'camera'):
                self.system.camera.clear_trajectory()
                self.system.camera.clear_target()
                # explicitly set trajectory_type to None
                self.system.camera.trajectory_type = None
        self.system_status.config(text="System: PATH STOPPED", fg=self.colors['warning'])
    
    
    
    # ==================== CONTROL FUNCTIONS ====================
    
    def toggle_fullscreen(self, event=None):
        """Toggle between fullscreen and windowed mode."""
        if self.is_closing:
            return
        
        # force update of window state to ensure correct status
        self.update_idletasks()
        
        if self.is_fullscreen:
            # exit fullscreen - return to windowed mode
            self.attributes('-fullscreen', False)
            # force geometry update before setting windowed mode
            self.update_idletasks()
            self._set_windowed_geometry()
            self.fullscreen_btn.config(text="\u25B2")
        else:
            # enter fullscreen
            self._set_fullscreen_geometry()
            self.update_idletasks()
            self.fullscreen_btn.config(text="\u25BC")
        
        # force canvas to recalculate centering by updating its dimensions directly
        if hasattr(self, 'video_canvas') and self.video_canvas.is_alive:
            # get the current canvas dimensions
            try:
                canvas_width = self.video_canvas.winfo_width()
                canvas_height = self.video_canvas.winfo_height()
                if canvas_width > 0 and canvas_height > 0:
                    # update canvas dimensions manually
                    self.video_canvas.canvas_width = canvas_width
                    self.video_canvas.canvas_height = canvas_height
                    # re-center the image
                    if self.video_canvas.original_width > 0 and self.video_canvas.original_height > 0:
                        self.video_canvas._center_image()
                    # force a redraw by updating the image if it exists
                    if hasattr(self.video_canvas, 'current_photo') and self.video_canvas.current_photo:
                        # get the last frame from camera
                        frame = self.system.camera.get_annotated_frame() if hasattr(self.system.camera, 'get_annotated_frame') else self.system.camera.latest_frame
                        if frame is not None:
                            self.video_canvas.update_image(frame)
            except Exception as e:
                pass
        
        self.update_idletasks()
    
    
    
    def toggle_auto_balance(self):
        if self.is_closing:
            return
        if hasattr(self, 'auto_balance_var'):
            if self.auto_balance_var.get():
                self.system.controller.reset_pid()
                self.system.start_auto_balance()
                self.balance_status.config(text="Balance: ON", fg=self.colors['success'])
                self.system.controller.reset_pid()
            else:
                self.system.stop_auto_balance()
                self.auto_balance_var.set(False)
                self.balance_status.config(text="Balance: OFF", fg=self.colors['fg'])
            self.update()
    
    
    
    def update_target(self, x_mm=None, y_mm=None):
        """Update target position with circular boundary enforcement."""
        if self.is_closing:
            return
        
        # get values
        if x_mm is not None and y_mm is not None:
            x = max(-110, min(110, int(x_mm)))
            y = max(-110, min(110, int(y_mm)))
            self.target_x.set(x)
            self.target_y.set(y)
        else:
            x = self.target_x.get()
            y = self.target_y.get()
        
        # use a slightly larger radius for calculation to prevent getting stuck
        MAX_RADIUS = 85
        ALLOWANCE = 2  # small allowance to prevent getting stuck
        
        radius = sqrt(x*x + y*y)
        
        if radius > MAX_RADIUS + ALLOWANCE:
            # scale down to fit within boundary
            scale = MAX_RADIUS / radius
            x = int(round(x * scale))
            y = int(round(y * scale))
            self.target_x.set(x)
            self.target_y.set(y)
            self.system_status.config(text=f"Boundary: ({x}, {y}) mm", fg=self.colors['warning'])
            self.after(1500, self.refresh_system_status)
        
        # send to robot
        self.system.controller.target_x = x * 10
        self.system.controller.target_y = y * 10
        self.system.camera.set_target_position(x, y)
    
    
    
    def center_target(self):
        """Reset target to center."""
        if self.is_closing:
            return
        self.update_target(0, 0)
    
    
    
    def home_motors(self):
        """Home motor: Moves the platform toward home, and applies sensorless homing."""
        if self.is_closing:
            return
        def run():
            self.auto_balance_var.set(False)
            self.system.controller.auto_balance = False
            self.system.home_platform()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def balance_platform(self):
        """Balance platform: Moves the platform to the balancing position."""
        if self.is_closing:
            return
        def run():
            self.system.balance_platform()
        threading.Thread(target=run, daemon=True).start()
    
    
    
    def emergency_stop(self):
        """Emergency stop: Disable motors and update UI."""
        if self.is_closing:
            return
        
        try:
            # disable motors
            self.system.mc.disable_motors()
            self.system.controller.auto_balance = False
            self.auto_balance_var.set(False)
            
            # update status displays
            self.balance_status.config(text="Balance: OFF", fg=self.colors['fg'])
            self.system_status.config(text="System: MOTORS OFF", fg=self.colors['error'])
            
            # update button - force color change
            self.estop_btn.config(
                text="ACTIVATE MOTORS", 
                bg=self.colors['success'], 
                fg='white',
                command=self.activate_motors,
                state=tk.NORMAL
            )
            
            # force UI update
            self.estop_btn.update_idletasks()
            self.update()
            
        except Exception as e:
            print(f"Emergency stop error: {e}")
            self.system_status.config(text="System: ERROR", fg=self.colors['error'])
    
    
    
    def activate_motors(self):
        """Activate motors after emergency stop."""
        if self.is_closing:
            return
        
        try:
            # enable motors
            self.system.mc.enable_motors()
            
            # update button - force color change
            self.estop_btn.config(
                text="EMERGENCY STOP", 
                bg=self.colors['error'], 
                fg='white',
                command=self.emergency_stop,
                state=tk.NORMAL
            )
            
            # update status
            self.system_status.config(text="System: READY", fg=self.colors['success'])
            
            # force UI update
            self.estop_btn.update_idletasks()
            self.update()
            
        except Exception as e:
            print(f"Motor activation error: {e}")
            self.system_status.config(text="System: MOTOR ERROR", fg=self.colors['error'])
    
    
    
    def apply_motor_shift_delta(self, motor):
        """Apply individual motor shift delta immediately without full homing."""
        if self.is_closing:
            return
        
        # get current value from spinbox
        if motor == 'A':
            new_shift_a = self.shift_a.get()
            if new_shift_a != self.prev_shift_a:
                self.stop_auto_balance()
                self.old_shift_a = self.prev_shift_a
                delta_steps = new_shift_a - self.old_shift_a
                self.prev_shift_a = new_shift_a
                self.system.controller.BA_SHIFT_A = new_shift_a
                print(f"Motor {motor}: adjusting by {delta_steps:+d} steps (now {new_shift_a})")
            else:
                return
        
        elif motor == 'B':
            new_shift_b = self.shift_b.get()
            if new_shift_b != self.prev_shift_b:
                self.stop_auto_balance()
                self.old_shift_b = self.prev_shift_b
                delta_steps = new_shift_b - self.old_shift_b
                self.prev_shift_b = new_shift_b
                self.system.controller.BA_SHIFT_B = new_shift_b
                print(f"Motor {motor}: adjusting by {delta_steps:+d} steps (now {new_shift_b})")
            else:
                return
                
        elif motor == 'C':
            new_shift_c = self.shift_c.get()
            if new_shift_c != self.prev_shift_c:
                self.stop_auto_balance()
                self.old_shift_c = self.prev_shift_c
                delta_steps = new_shift_c - self.old_shift_c
                self.prev_shift_c = new_shift_c
                self.system.controller.BA_SHIFT_C = new_shift_c
                print(f"Motor {motor}: adjusting by {delta_steps:+d} steps (now {new_shift_c})")
            else:
                return
                
        else:
            return
        
        if delta_steps == 0:
            return
        
        # re-level platform, by applying just the delta movement
        def run():
            # stop auto-balance temporarily
            was_auto_balance = False
            if hasattr(self, 'auto_balance_var') and self.auto_balance_var.get():
                was_auto_balance = True
                self.stop_auto_balance()
            
            # calculate the delta angle in tenths of degree
            delta_tenths = int(delta_steps * self.system.controller.STEP_ANGLE_TENTHS)
            
            # apply delta to current motor angle
            if motor == 'A':
                current_angle = self.system.controller.current_angles.A
                new_angle = current_angle + delta_tenths
                
                # clamp to safe limits (30° to 150°)
                new_angle = max(300, min(1500, new_angle))
                target_angles = MotorAngles(
                    A=new_angle,
                    B=self.system.controller.current_angles.B,
                    C=self.system.controller.current_angles.C
                )
            elif motor == 'B':
                current_angle = self.system.controller.current_angles.B
                new_angle = current_angle + delta_tenths
                new_angle = max(300, min(1500, new_angle))
                target_angles = MotorAngles(
                    A=self.system.controller.current_angles.A,
                    B=new_angle,
                    C=self.system.controller.current_angles.C
                )
            else:  # motor C
                current_angle = self.system.controller.current_angles.C
                new_angle = current_angle + delta_tenths
                new_angle = max(300, min(1500, new_angle))
                target_angles = MotorAngles(
                    A=self.system.controller.current_angles.A,
                    B=self.system.controller.current_angles.B,
                    C=new_angle
                )
            
            # move just that motor
            movement_time_ms = self.system.controller.move_to_angles(target_angles)
            sleep(movement_time_ms / 1000 + 0.05)
            
            # restart auto-balance if it was on
            if was_auto_balance:
                self.auto_balance_var.set(True)
                self.toggle_auto_balance()
        
        threading.Thread(target=run, daemon=True).start()
        
        self.system.save_balance_shifts()
    
    
    
    def stop_auto_balance(self):
        if self.system.controller.auto_balance:
            self.system.stop_auto_balance()
            self.auto_balance_var.set(False)
            sleep(0.05)
    
    
    
    def on_dir_change(self, v):
        if not self.is_closing:
            if self.dir_slider.get() != 90:
                self.stop_auto_balance()
                self.update_tilt()
    
    
    
    def on_tilt_change(self, v):
        if not self.is_closing:
            if self.tilt_slider.get() != 0:
                self.stop_auto_balance()
                self.update_tilt()
    
    
    
    def update_tilt(self):
        if self.is_closing:
            return
        dir_deg = self.dir_slider.get()
        tilt_deg = self.tilt_slider.get()
        rad = radians(dir_deg)
        self.apply_tilt(tilt_deg * cos(rad), tilt_deg * sin(rad))
    
    
    
    def apply_tilt(self, x, y):
        if self.is_closing:
            return
            
        if self.system.controller.at_balance:
            
            tilt_x_rad = radians(x)
            tilt_y_rad = radians(y)
            
            motor_pos = [radians(90), radians(210), radians(330)]
            deltas = []
            for pos in motor_pos:
                delta = (-cos(pos) * tilt_x_rad - sin(pos) * tilt_y_rad)
                deltas.append(int(round(degrees(delta) * 10)))
            
            targets = MotorAngles(900 + deltas[0], 900 + deltas[1], 900 + deltas[2])
            targets.A = max(300, min(targets.A, 1500))
            targets.B = max(300, min(targets.B, 1500))
            targets.C = max(300, min(targets.C, 1500))
            
            movement_time_ms = self.system.controller.move_to_angles(targets)
            self.system.controller.move_to_angles(targets)
            
            sleep(movement_time_ms/1000)
    
    
    
    def level_platform_with_shifts(self):
        """Level the platform using current spinbox shift values (if changed)."""
        if self.is_closing:
            return
        
        # check the sliders values
        new_shift_a = self.shift_a.get()
        new_shift_b = self.shift_b.get()
        new_shift_c = self.shift_c.get()
        
        # case there are variations at the motor shift values vs previous values
        if new_shift_a != self.old_shift_a or new_shift_b != self.old_shift_b or new_shift_c != self.old_shift_c:
            
            # update reference
            self.old_shift_a = new_shift_a
            self.old_shift_b = new_shift_b
            self.old_shift_c = new_shift_c
            
            # update controller with current spinbox values
            self.system.controller.BA_SHIFT_A = new_shift_a
            self.system.controller.BA_SHIFT_B = new_shift_b
            self.system.controller.BA_SHIFT_C = new_shift_c
            
            print(f"Leveling platform with shifts: A={new_shift_a}, B={new_shift_b}, C={new_shift_c}")
            
            self.system.save_balance_shifts()
            
            def run():
                # stop auto-balance if running
                was_auto_balance = False
                if hasattr(self, 'auto_balance_var') and self.auto_balance_var.get():
                    was_auto_balance = True
                    self.auto_balance_var.set(False)
                    self.system.stop_auto_balance()
                    sleep(0.05)
                
                # home and balance with current shifts
                self.system.home_platform()
                sleep(0.5)
                self.system.balance_platform()
                
                # reset tilt sliders to zero
                self.dir_slider.set(90)
                self.tilt_slider.set(0)
                
                # update status
                self.system_status.config(text="Platform leveled", fg=self.colors['success'])
                self.after(1500, self.refresh_system_status)
            
            threading.Thread(target=run, daemon=True).start()
        
        # case there are no variations at the motor shift values vs previous values
        else:
            self.dir_slider.set(90)
            self.tilt_slider.set(0)
            self.apply_tilt(0, 0)
    
    
    
    def auto_calibrate(self):
        """Auto calibration for ball color: Shows progress windows."""
        if self.is_closing:
            return

        # stop auto-balance temporarily
        if hasattr(self, 'auto_balance_var'):
            self.auto_balance_var.set(False)
            self.toggle_auto_balance()
            self.update_idletasks()
        
        # call the auto calibration method (handles windows internally)
        # in GUI mode, this runs silently (no OpenCV windows)
        ret, lower, upper = self.system.camera.auto_calibrate_with_windows()
        
        if ret:
            self.update_hsv_display()
            print(f"Auto-calibration successful: Lower={lower}, Upper={upper}")
        else:
            print("Auto-calibration failed")
        
        # reset the PID accumulators, remove accumulated errors
        # while calibrating, if ball out of center
        self.system.controller.reset_pid()
        
        # restart auto-balance
        if hasattr(self, 'auto_balance_var'):
            self.auto_balance_var.set(True)
            self.toggle_auto_balance()
            self.update_idletasks()
        
        # reset the PID accumulators, remove accumulated errors
        # while calibrating, if ball out of center
        self.system.controller.reset_pid()
    
    
    
    def manual_calibrate(self):
        """Applies the manual_hsv function for the ball colour."""
        if self.is_closing:
            return
        
        if hasattr(self, 'auto_balance_var'):
            self.auto_balance_var.set(False)
            self.toggle_auto_balance()
            self.update_idletasks()
        
        # call the manual_hsv function, at the camera class
        self.system.camera.manual_hsv_adjust()
        
        # refresh the page
        self.update_hsv_display()
        
        # reset the PID accumulators, remove accumulated errors
        # while calibrating, if ball out of center
        self.system.controller.reset_pid()
        
        if hasattr(self, 'auto_balance_var'):
            self.auto_balance_var.set(True)
            self.toggle_auto_balance()
            self.update_idletasks()
        
        # reset the PID accumulators, remove accumulated errors
        # while calibrating, if ball out of center
        self.system.controller.reset_pid()
    
    
    
    def save_calibration(self):
        if self.is_closing:
            return
        if self.system.camera.save_calibration():
            messagebox.showinfo("Success", "HSV calibration saved")
        else:
            messagebox.showerror("Error", "Failed to save")
    
    
    
    def apply_ball_mm(self):
        """Apply ball diameter change."""
        if self.is_closing:
            return
        
        self.system.camera.ball_mm = self.ball_diam.get()
    
    
    
    def on_closing(self):
        """Handle window closing with proper cleanup."""
        if self.is_closing:
            return
        
        self.is_closing = True
        
        try:
            if self.winfo_exists():
                if messagebox.askokcancel("Quit", "Do you want to quit?"):
                    self.video_running = False
                    self.stop_auto_balance()
                    self.system.cleanup()
                    self.destroy()
                else:
                    self.is_closing = False
        except Exception as e:
            print(f"Error during closing: {e}")
            try:
                self.video_running = False
                if hasattr(self, 'system'):
                    if hasattr(self.system, 'controller') and self.system.controller.auto_balance:
                        self.system.stop_auto_balance()
                    self.system.cleanup()
            except:
                pass
            self.destroy()




# ============================================================================
# path configuration pages
# ============================================================================

class PathConfigPage:
    """Base class for path configuration pages."""
    
    def __init__(self, parent, gui, path_name, back_callback):
        self.parent = parent
        self.gui = gui
        self.path_name = path_name
        self.back_callback = back_callback
        self.frame = None
        self.repeats = None
    
    
    
    def create(self):
        """Create the configuration page."""
        self.frame = StaticFrame(self.parent, bg=self.gui.colors['bg'])
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame.grid_columnconfigure(0, weight=1)
        self.frame.grid_rowconfigure(1, weight=1)
        
        # header with back button
        header = tk.Frame(self.frame, bg=self.gui.colors['bg'])
        header.grid(row=0, sticky="ew", pady=5)
        header.grid_columnconfigure(0, weight=1)
        
        back_btn = tk.Button(header, text="◄ BACK", bg=self.gui.colors['button'],
                            fg=self.gui.colors['fg'], font=('Arial', 12, 'bold'),
                            command=self.back_callback)
        back_btn.grid(row=0, column=0, sticky="w", padx=19)
        self.gui.remove_highlight(back_btn)
        
        title = tk.Label(header, text=f"{self.path_name} Path Config   ",
                        bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                        font=('Arial', 17, 'normal'))
        title.grid(row=0, column=1, sticky="e", padx=(0,6))
        
        # content area
        self.content = tk.Frame(self.frame, bg=self.gui.colors['bg'])
        self.content.grid(row=1, sticky="nsew", padx=20, pady=10)
        self.content.grid_columnconfigure(0, weight=1)
        
        return self.content
    
    
    
    def add_repeats_and_start(self, start_command):
        """Add repeats spinbox and start button side by side using grid."""
        # container for bottom section
        bottom_frame = tk.Frame(self.content, bg=self.gui.colors['bg'])
        bottom_frame.grid(row=99, column=0, sticky="ew", pady=15)
        
        # configure columns
        bottom_frame.grid_columnconfigure(0, weight=0)
        bottom_frame.grid_columnconfigure(1, weight=1)
        bottom_frame.grid_columnconfigure(2, weight=0)
        
        # repeats section (left aligned)
        repeats_frame = tk.Frame(bottom_frame, bg=self.gui.colors['bg'])
        repeats_frame.grid(row=0, column=0, sticky="w")
        
        title_label = tk.Label(repeats_frame, text="Repeats",
                               bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                               font=('Arial', 16, 'normal'))
        title_label.pack(anchor='w')
        
        self.repeats = tk.IntVar(value=3)
        spinbox = tk.Spinbox(repeats_frame, from_=1, to=100, textvariable=self.repeats,
                            width=5, font=('Arial', 28), justify='left')
        spinbox.pack(pady=10)
        
        # start button (right aligned)
        start_btn = tk.Button(bottom_frame, text=f"START\n{self.path_name.upper()}\nPATH",
                             bg=self.gui.colors['accent'], fg='white',
                             font=('Arial', 15, 'bold'), height=3,
                             command=start_command)
        start_btn.grid(row=0, column=2, sticky="e")
        self.gui.remove_highlight(start_btn)
    
    
    
    def destroy(self):
        """Clean up the path configuration page."""
        if self.frame:
            self.frame.destroy()
            self.frame = None
        self.repeats = None






class SquarePathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Square", back_callback)
    
    
    
    def create(self):
        content = super().create()
        row = 0
        
        # side length
        tk.Label(content, text="Side Length (mm):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 12)).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.side = tk.IntVar(value=100)
        tk.Scale(content, from_=50, to=140, orient=tk.HORIZONTAL,
                variable=self.side, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # add repeats and start button
        self.add_repeats_and_start(self.start_path)
    
    
    
    def start_path(self):
        self.gui.start_square_path(self.side.get(), self.repeats.get())





class CirclePathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Circle", back_callback)
    
    
    def create(self):
        content = super().create()
        row = 0
        
        # radius
        tk.Label(content, text="Radius (mm):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 12)).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.radius = tk.IntVar(value=70)
        tk.Scale(content, from_=40, to=100, orient=tk.HORIZONTAL,
                variable=self.radius, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # direction
        tk.Label(content, text="Direction:", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 16, 'normal')).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        # create a frame that spans the full width
        dir_frame = tk.Frame(content, bg=self.gui.colors['bg'])
        dir_frame.grid(row=row, column=0, sticky="ew", pady=5)
        dir_frame.grid_columnconfigure(0, weight=1)
        dir_frame.grid_columnconfigure(1, weight=1)
        
        self.direction = tk.StringVar(value="cw")
        
        # left-aligned radiobutton (clockwise)
        cw_frame = tk.Frame(dir_frame, bg=self.gui.colors['bg'])
        cw_frame.grid(row=0, column=0, sticky="w")
        tk.Radiobutton(cw_frame, text="  ↻   ", variable=self.direction,
                      value="cw", bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                      selectcolor=self.gui.colors['accent'],
                      font=('Arial', 30)).pack(side=tk.LEFT)
        
        # right-aligned radiobutton (counter-clockwise)
        ccw_frame = tk.Frame(dir_frame, bg=self.gui.colors['bg'])
        ccw_frame.grid(row=0, column=1, sticky="e")
        tk.Radiobutton(ccw_frame, text="  ↺   ", variable=self.direction,
                      value="ccw", bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                      selectcolor=self.gui.colors['accent'],
                      font=('Arial', 30)).pack(side=tk.LEFT)
        row += 1
        
        # add repeats and start button
        self.add_repeats_and_start(self.start_path)
    
    
    def start_path(self):
        self.gui.start_circle_path(self.radius.get(), self.repeats.get(), self.direction.get())





class InfinityPathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Infinity", back_callback)
    
    
    def create(self):
        content = super().create()
        row = 0
        
        # size
        tk.Label(content, text="Size (mm):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 16, 'normal')).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.size = tk.IntVar(value=100)
        tk.Scale(content, from_=140, to=160, orient=tk.HORIZONTAL,
                variable=self.size, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # speed factor
        tk.Label(content, text="Speed Factor:", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 16, 'normal')).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.speed = tk.DoubleVar(value=1.0)
        tk.Scale(content, from_=0.7, to=1.2, resolution=0.1, orient=tk.HORIZONTAL,
                variable=self.speed, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # add repeats and start button
        self.add_repeats_and_start(self.start_path)
    
    
    def start_path(self):
        self.gui.start_infinity_path(self.size.get(), self.speed.get(), self.repeats.get())





class TrianglePathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Triangle", back_callback)
    
    
    
    def create(self):
        content = super().create()
        row = 0
        
        # side length
        tk.Label(content, text="Side Length (mm):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 12)).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.side = tk.IntVar(value=100)
        tk.Scale(content, from_=50, to=160, orient=tk.HORIZONTAL,
                variable=self.side, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # orientation
        tk.Label(content, text="Orientation:", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 16, 'normal')).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        # create a frame that spans the full width
        orient_frame = tk.Frame(content, bg=self.gui.colors['bg'])
        orient_frame.grid(row=row, column=0, sticky="ew", pady=5)
        orient_frame.grid_columnconfigure(0, weight=1)
        orient_frame.grid_columnconfigure(1, weight=1)
        
        self.orientation = tk.StringVar(value="point_up")
        
        # left-aligned radiobutton (point up)
        up_frame = tk.Frame(orient_frame, bg=self.gui.colors['bg'])
        up_frame.grid(row=0, column=0, sticky="w")
        tk.Radiobutton(up_frame, text="  △   ", variable=self.orientation,
                      value="point_up", bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                      selectcolor=self.gui.colors['accent'],
                      font=('Arial', 30)).pack(side=tk.LEFT)
        
        # right-aligned radiobutton (point down)
        down_frame = tk.Frame(orient_frame, bg=self.gui.colors['bg'])
        down_frame.grid(row=0, column=1, sticky="e")
        tk.Radiobutton(down_frame, text="  ▽   ", variable=self.orientation,
                      value="point_down", bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                      selectcolor=self.gui.colors['accent'],
                      font=('Arial', 30)).pack(side=tk.LEFT)
        row += 1
        
        # add repeats and start button
        self.add_repeats_and_start(self.start_path)
    
    
    def start_path(self):
        self.gui.start_triangle_path(self.side.get(), self.orientation.get(), self.repeats.get())





class LinePathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Line", back_callback)
    
    
    def create(self):
        content = super().create()
        row = 0
        
        # length
        tk.Label(content, text="Line Length (mm):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 12)).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.length = tk.IntVar(value=120)
        tk.Scale(content, from_=60, to=160, orient=tk.HORIZONTAL,
                variable=self.length, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # direction angle
        tk.Label(content, text="Direction (degrees):", bg=self.gui.colors['bg'],
                fg=self.gui.colors['fg'], font=('Arial', 16, 'normal')).grid(row=row, column=0, sticky="w", pady=5)
        row += 1
        
        self.angle = tk.IntVar(value=0)
        tk.Scale(content, from_=0, to=90, orient=tk.HORIZONTAL,
                variable=self.angle, bg=self.gui.colors['bg'], font=("Arial", 16),
                fg=self.gui.colors['fg'], length=300).grid(row=row, column=0, sticky="ew", pady=5)
        row += 1
        
        # add repeats and start button
        self.add_repeats_and_start(self.start_path)
    
    
    def start_path(self):
        self.gui.start_line_path(self.length.get(), self.angle.get(), self.repeats.get())





class FreePathPage(PathConfigPage):
    def __init__(self, parent, gui, back_callback):
        super().__init__(parent, gui, "Free", back_callback)
        self.tracking_active = False
        self.recording = False
        self.recorded_path = []
        self.recording_points = []
        self.timeout_id = None
        self.recording_update_id = None
        self.last_point_count = 0
        self.TIMEOUT_MS = 2000
    
    
    def create(self):
        content = super().create()
        
        # configure content grid rows for proper spacing
        content.grid_rowconfigure(0, weight=0)  # instructions - fixed
        content.grid_rowconfigure(1, weight=0)  # control frame - fixed
        content.grid_rowconfigure(2, weight=0)  # status label - fixed
        content.grid_rowconfigure(3, weight=1)  # spacer - takes all extra space
        content.grid_rowconfigure(4, weight=0)  # stop button - fixed at bottom
        content.grid_columnconfigure(0, weight=1)
        
        row = 0
        
        # instructions
        instructions = tk.Label(content, 
            text="Touch and drag on the video area!\nThe ball follows your finger.\nRelease to stop.",
            bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
            font=('Arial', 12), justify=tk.CENTER)
        instructions.grid(row=row, column=0, pady=10, sticky="ew")
        row += 1
        
        # recording controls frame
        control_frame = tk.Frame(content, bg=self.gui.colors['bg'])
        control_frame.grid(row=row, column=0, pady=10, sticky="ew")
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)
        row += 1
        
        self.record_btn = tk.Button(control_frame, text="START REC",
                                    bg=self.gui.colors['warning'], fg='black',
                                    font=('Arial', 12, 'bold'),
                                    command=self.toggle_recording)
        self.record_btn.grid(row=0, column=0, padx=10, sticky="ew")
        
        self.play_btn = tk.Button(control_frame, text="PLAY PATH",
                                  bg=self.gui.colors['accent'], fg='white',
                                  font=('Arial', 12, 'bold'), state=tk.DISABLED,
                                  command=self.play_recorded_path)
        self.play_btn.grid(row=0, column=1, padx=10, sticky="ew")
        self.gui.remove_highlight(self.play_btn)
        row += 1
        
        # status label
        self.status_label = tk.Label(content, text="Ready\nTouch the screen to move the ball",
                                     bg=self.gui.colors['bg'], fg=self.gui.colors['fg'],
                                     font=('Arial', 16)) # HERE 13
        self.status_label.grid(row=row, column=0, pady=10)
        row += 1
        
        # spacer (pushes stop button to bottom)
        spacer = tk.Frame(content, bg=self.gui.colors['bg'])
        spacer.grid(row=row, column=0, sticky="nsew")
        row += 1
        
        # stop path button (fixed at bottom)
        stop_btn = tk.Button(content, text="STOP PATH",
                             bg=self.gui.colors['error'], fg='white',
                             font=('Arial', 12, 'bold'),
                             command=self.stop_path)
        stop_btn.grid(row=row, column=0, pady=10, sticky="ew")
        self.gui.remove_highlight(stop_btn)
        
        # enable touch tracking on video canvas
        if hasattr(self.gui, 'video_canvas'):
            self.gui.video_canvas.set_touch_callback(self.on_touch)
        
        # start continuous mode automatically when page is created
        self.start_continuous_mode()
    
    
    
    def start_continuous_mode(self):
        """Start continuous mode automatically, enabling auto-balance if needed."""
        if not self.tracking_active:
            # Enable auto-balance if needed
            self.gui._ensure_auto_balance()
                
            # HERE
#             if not self.gui.auto_balance_var.get():
#                 self.gui.auto_balance_var.set(True)
#                 self.gui.toggle_auto_balance()
#                 sleep(0.5)
            
            self.tracking_active = True
            self.gui.update_target(0, 0)
            self.gui.system.free_path_continuous()
            self.status_label.config(text="Continuous mode active\nTouch the screen to move the ball")
    
    
    
    def stop_path(self):
        """Stop the current free path."""
        self._cancel_timeout()
        self.tracking_active = False
        self.gui.stop_current_path()
        if "Playing" in self.status_label.cget("text"):
            self.status_label.config(text="Path stopped")
            self.gui.after(2000, self._revert_status_message)
    
    
    
    def toggle_recording(self):
        if not self.recording:
            # start recording
            self.recording = True
            self.recording_points = []
            self.last_point_count = 0
            
            # stop continuous mode temporarily
            if self.tracking_active:
                self.tracking_active = False
                self.gui.stop_current_path()
            
            # start recording in robot
            self.gui.system.free_path_start_recording()
            
            # update UI
            self.record_btn.config(
                text="STOP REC",
                bg=self.gui.colors['error'],
                activebackground=self.gui.colors['error'],
                activeforeground=self.record_btn.cget('fg'),
                highlightthickness=0,
                relief=tk.RAISED
                )

            self.status_label.config(text="Recording: 0 points")
            self.play_btn.config(state=tk.DISABLED)
            self.gui.update_idletasks()
            
            # start periodic count updates
            self._update_recording_count()
            
        else:
            # stop recording
            self.recording = False
            
            # cancel periodic count updates
            if self.recording_update_id:
                self.gui.after_cancel(self.recording_update_id)
                self.recording_update_id = None
            
            # stop recording in robot
            self.gui.system.free_path_stop_recording()
            
            # get final recorded points
            self.gui.after(200, self._get_recorded_points)
            
            # update UI
            self.record_btn.config(
                text="START REC",
                bg=self.gui.colors['warning'],
                activebackground=self.gui.colors['warning'],
                activeforeground=self.record_btn.cget('fg'),
                highlightthickness=0,
                relief=tk.RAISED
                )
            self.gui.update_idletasks()
            
            # restart continuous mode
            self.start_continuous_mode()
    
    
    
    def _update_recording_count(self):
        """Periodically update the recording point count from local storage."""
        if self.recording:
            current_points = len(self.recording_points)
            if current_points != self.last_point_count:
                self.last_point_count = current_points
                self.status_label.config(text=f"Recording: {current_points} points")
                self.status_label.update_idletasks()
            self.recording_update_id = self.gui.after(50, self._update_recording_count)
    
    
    
    def _cancel_timeout(self):
        """Cancel any pending timeout."""
        if self.timeout_id:
            self.gui.after_cancel(self.timeout_id)
            self.timeout_id = None
    
    
    
    def _reset_to_center(self):
        """Reset target to center."""
        if self.tracking_active:
            self.gui.update_target(0, 0)
            self.status_label.config(text="Finger removed\nBall at center")
            self.gui.after(1500, self._revert_status_message)
        self._cancel_timeout()
    
    
    
    def _revert_status_message(self):
        """Revert status label to normal message."""
        if self.tracking_active and not self.recording:
            self.status_label.config(text="Continuous mode active\nTouch the screen to move the ball")
        elif not self.tracking_active and not self.recording:
            self.status_label.config(text="Ready\nTouch the screen to move the ball")
    
    
    
    def _get_recorded_points(self):
        """Get recorded points from robot (final points)."""
        self.recorded_path = self.gui.system.get_recorded_points()
        if not self.recorded_path and self.recording_points:
            self.recorded_path = self.recording_points
        if self.recorded_path:
            self.play_btn.config(state=tk.NORMAL)
            self.gui.update_target(0, 0)
        self.status_label.config(text=f"Path recorded: {len(self.recorded_path)} points")
        self.gui.after(2000, self._revert_status_message)
    
    
    
    def play_recorded_path(self):
        if self.recorded_path:
            if self.tracking_active:
                self.tracking_active = False
                self.gui.stop_current_path()
            self.status_label.config(text="Playing recorded path...")
            self.gui.system.free_path_playback(self.recorded_path, completion_callback=self._on_playback_complete)
    
    
    
    def _on_playback_complete(self):
        """Called when playback completes."""
        print("Playback complete callback triggered")
        def update_ui():
            try:
                if self.status_label and self.status_label.winfo_exists():
                    self.status_label.config(text="Playback complete\nRestarting continuous mode")
                    self.gui.after(100, self.start_continuous_mode)
                    self.gui.after(2000, self._revert_status_message)
            except (tk.TclError, AttributeError, RuntimeError):
                print("Widget no longer exists, skipping UI update")
        self.gui.after(0, update_ui)
    
    
    
    def on_touch(self, action, x_mm, y_mm):
        """
        Process touch/drag events from the video canvas.
        - in continuous mode: moves the ball to follow finger.
        - in recording mode: additionally captures path points for playback.
        """
        if self.tracking_active:
            if action == 'start' or action == 'move':
                self._cancel_timeout()
                self.gui.update_target(x_mm, y_mm)
                self.status_label.config(text=f"Following: ({x_mm:.1f}, {y_mm:.1f}) mm")
            elif action == 'end':
                self._cancel_timeout()
                self.timeout_id = self.gui.after(self.TIMEOUT_MS, self._reset_to_center)
                self.status_label.config(text="Finger removed\nReturning to center in 2s")
                
        elif self.recording:
            if action == 'start' or action == 'move':
                # send to robot
                self.gui.update_target(x_mm, y_mm)
                
                # store point locally for counting
                self.recording_points.append((x_mm, y_mm))
                
                # also update count immediately for instant feedback
                current_count = len(self.recording_points)
                if current_count != self.last_point_count:
                    self.last_point_count = current_count
                    self.status_label.config(text=f"Recording: {current_count} points")
                    self.status_label.update_idletasks()
    
    
    
    def destroy(self):
        """Clean up the free path page."""
        if self.recording_update_id:
            self.gui.after_cancel(self.recording_update_id)
            self.recording_update_id = None
        self._cancel_timeout()
        if self.tracking_active:
            self.gui.stop_current_path()
        if hasattr(self.gui, 'video_canvas'):
            self.gui.video_canvas.set_touch_callback(None)
        super().destroy()





def main(verbose=False):
    """Main program entry point for the GUI."""
    
    print("\n" + "="*60)
    print("MIRRORBALLBOT TOUCHSCREEN GUI, by Andrea Favero")
    print(f"Version: {__version__}")
    print("="*60 + "\n")
    
    # create a setting manager instance
    settings_mgr = SettingsManager(verbose=VERBOSE)
    
    try:
        app = BallBalancingGUI(verbose=verbose, settings_mgr=settings_mgr)
        app.mainloop()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        print("\nGUI terminated")


if __name__ == "__main__":
    main(verbose=VERBOSE)
    