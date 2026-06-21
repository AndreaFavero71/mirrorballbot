"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


MIT License
Copyright (c) 2026 Andrea Favero
"""

from machine import mem16
import _thread

class SharedMemory:
    def __init__(self, address):
        self.address = address
        self._lock = _thread.allocate_lock()
    
    def read(self):
        with self._lock:
            return mem16[self.address]
    
    def write(self, value):
        with self._lock:
            mem16[self.address] = value
#         print("Value just written:", self.read())
