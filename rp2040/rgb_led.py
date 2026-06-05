"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


MIT License
Copyright (c) 2026 Andrea Favero
"""

from shared_variables import shared_variables
from machine import Pin
from neopixel import NeoPixel
import _thread, time

class RgbLed:
    
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            print("\n[core 0] Uploading rgb_led ...")
            cls._instance = super().__new__(cls)
            cls._instance._init()
        return cls._instance
    
    
    def _init(self):
        self.np = NeoPixel(Pin(shared_variables.LED_ONBOARD_PIN, Pin.OUT), 1)  # 1=NeoPixel number 1 (the only one on the board)
        self._lock = _thread.allocate_lock()


    def _get_rgb_color(self, color, bright):    # Adjust brightness for the specified color
        if color == 'red':
            return (int(bright * 255), 0, 0)
        elif color == 'green':
            return (0, int(bright * 255), 0)
        elif color == 'blue':
            return (0, 0, int(bright * 255))
        else:
            raise ValueError("[core 0] Invalid color. Choose 'red', 'green', or 'blue'.")

    
    
    def _validate_args(self, color, bright, times, time_s):  # Validate parameters
        if not (0 <= bright <= 1):
            raise ValueError("[core 0] Brightness 'bright' must be between 0 and 1.")
        if not (isinstance(times, int) and times > 0):
            raise ValueError("[core 0] 'times' must be a positive integer.")
        if not (isinstance(time_s, (int, float)) and time_s >= 0):
            raise ValueError("[core 0] 'time_s' must be a >= 0.")



    def flash_color(self, color, bright=1, times=1, time_s=0.01):
        self._validate_args(color, bright, times, time_s)
        rgb_color = self._get_rgb_color(color, bright)
        
        # Flash the specified color
        with self._lock:
            for _ in range(times):
                self.np[0] = rgb_color
                self.np.write()
                time.sleep(time_s)
                self.np[0] = (0, 0, 0)
                self.np.write()
                time.sleep(time_s)
    
    
    def fast_flash_red(self, ticks=1):
        self.np[0] = (255, 0, 0)
        self.np.write()
        for i in range(ticks):
            continue
        self.np[0] = (0, 0, 0)
        self.np.write()
        
    
    def fast_flash_green(self, ticks=1):
        self.np[0] = (0, 255, 0)
        self.np.write()
        for i in range(ticks):
            continue
        self.np[0] = (0, 0, 0)
        self.np.write()
    
    
    def fast_flash_blue(self, ticks=1):
        self.np[0] = (0, 0, 255)
        self.np.write()
        for i in range(ticks):
            continue
        self.np[0] = (0, 0, 0)
        self.np.write()


    def heart_beat(self, n=10,delay=0):
        self.flash_color('red', bright=0.06, times=n, time_s=0.05)
        time.sleep(delay/2)
        self.flash_color('green', bright=0.04, times=n, time_s=0.05)
        time.sleep(delay/2)
        self.flash_color('blue', bright=0.20, times=n, time_s=0.05)
        return True

    
    


rgb_led = RgbLed()