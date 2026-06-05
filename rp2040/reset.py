"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


Helper code to stops the core 1 tasks

MIT License
Copyright (c) 2026 Andrea Favero
"""

import time, _thread                         # micropython modules

def core1():
    """ Funtion with imports and functions running in Core1."""
    print("Core 1 is active ...")
    time.sleep(0.1)

def stop_code():
    time.sleep(0.1)                          # give core1 time to stop safely
    print("Program stopped")                 # feedback is printed to the terminal


print("Core 0 is active ...")
_thread.start_new_thread(core1, ())          # a new thread is started, with callback to core1 function

t_max = 0.5                                  # timeout to break the infinite loop
t_start = time.time()                        # reference time


try:
    while True:                              # infinite loop                           
        if time.time() - t_start > t_max:    # case the elapsed time is bigger than timeout
            break                            # infinite loop is interrupted
        time.sleep(0.1)                      # sleep for another tenth of second

except KeyboardInterrupt:                    # keyboard interrupts
    print("\nCtrl+C detected! Stopping Core 1...")   # feedback is printed to the terminal
    
except Exception as e:                       # error 
    print(f"\nAn error occured: {e}")        # feedback is printed to the terminal

finally:                                     # closing the try loop
    stop_code()                              # stop_code is called, to stop the PIOs and Timers
    