"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


MIT License
Copyright (c) 2026 Andrea Favero
"""

from machine import Timer                           # RP2040 (RP2350) specific modules
import _thread, time                                # micropython modules

# over-clocking
from machine import freq
freq(150000000) #default is 125000000 for RP2040
# freq(180000000) #default is 150000000 for RP2350


def import_led():
    print("[core 0] waiting time to eventually stop the code before further imports ...")
    from rgb_led import rgb_led                     # Singleton for RGB led handling
    ret = False
    ret = rgb_led.heart_beat(n=10, delay=1)         # allow Thonny to connect and eventually stop the RP2040

    while not ret:
        time. sleep(0.1)


def import_libraries():
    from shared_variables import shared_variables   # Singleton for global variables
    time.sleep(0.5)
    from stepper_controller import stepper_controller as motor   # Singleton for motor handling
    from i2c_handler import I2CHandler              # Class with the i2c data handling
    return shared_variables, motor, I2CHandler



def core1(fields):
    """ Funtion with imports and functions running in Core1."""
    i2c = I2CHandler(fields=fields)          # create the singleton instance of the I2CHandler class
    dummy_i2c = False                        # flag to use a dummy function mimicking I2C data arrival
    if not dummy_i2c:                        # case dummy_i2c is set False
        i2c.run()                            # starts I2C function
    else:                                    # case dummy_i2c is set True
        time.sleep(1.2)                      # sleep time letting other functions to complete their start up
        delay_us = 500                       # sleep time between each individual 8bit i2c data generation
        runs = 200                           # counter to print the total time to procees the runs
        i2c.dummy_i2c(delay_us, runs)        # calls the dummy I2C function





def drive_stepper(timer, shared_variables, motor):
    global prev_i2c_counter #, shared_variables, motor
    i2c_counter = shared_variables.i2c_counter.read()   # check if new I2C data is available
    
    if i2c_counter > prev_i2c_counter:         # case there is new I2C data available
        prev_i2c_counter = i2c_counter         # store the new I2C counter value     
        speed = shared_variables.field1.read() # read speed from the mem16 address
        steps = shared_variables.field2.read() # read steps from the mem16 address
        motor.run_stepper(speed, steps)  # call the run_stepper function with the new speed and steps values
        


def stop_code():
    if 'shared_variables' in locals():       # case shared_variables has been imported
        print("[core 0] Halt signal sent")   # feedback is printed to the terminal
        print("[core 0] Stopping Core 1...") # feedback is printed to the terminal
        shared_variables.halt.write(1)       # flag stopping the core1 tasks is written to the mem16 address
    if 'motor' in locals() :                 # case motor has been imported
        motor.stop_stepper()                 # motor gets stopped (PIO steps generation)
        motor.disable_stepper()              # flag to stop the motor from new requests
        print("[core 0] Motor is stopped")   # feedback is printed to the terminal
    if 'timer' in locals():                  # case timer has been imported
        timer.deinit()                       # timer get deinit
    if 'timer2' in globals():                # case timer2 has been imported
        timer2.deinit()                      # timer2 get deinit

    import gc
    gc.collect()
    print("\n[core 0] Free RAM:", gc.mem_free())
    time.sleep(0.5)                          # give core1 time to stop safely
    print("\n[core 0] Closing the program ...") # feedback is printed to the terminal
    


try:
    print("\n[core 0] MirrorBallBot code at RP2040")
    import_led()
    shared_variables, motor, I2CHandler = import_libraries()

    fields = 2               # number of data fields, max 4 (1 for speed, 1 for steps, the other 2 are not used)
    prev_i2c_counter = 0     # local varial with the latest i2c counter

    # start core1 thread
    _thread.start_new_thread(core1, (fields,))  # a new thread is started, with callback to core1 function

    # Timer setup for motor driving (in Core0)
    timer = Timer()                          # timer instance is created
    timer.init(period=5,                     # time interval in ms (below 5ms it might not be ready yet)
               mode=Timer.PERIODIC,          # periodic timer
               callback=lambda t: drive_stepper(t, shared_variables, motor)) # callback function and args

    while True:                              # infinite loop                           
        time.sleep(0.1)                      # sleep for another little while


except KeyboardInterrupt:                    # keyboard interrupts
    print("\nCtrl+C detected!")              # feedback is printed to the terminal
    
except Exception as e:                       # error 
    print(f"\nAn error occured: {e}")        # feedback is printed to the terminal

finally:                                     # closing the try loop
    stop_code()                              # stop_code is called, to stop the PIOs and Timers



