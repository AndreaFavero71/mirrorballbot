"""
Andrea Favero 20260524

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the motors test
Arguments can be passed to limit the test to 1 or more motors; This could be useful
to track the with wire is connected to which motor, once the wires are mixed up.




MIT License

Copyright (c) 2026 Andrea Favero

qaPermission is hereby granted, free of charge, to any person obtaining a copy
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



__version__ = "0.0.1"

from math import sqrt, radians, degrees, cos, sin, gcd
from gpiozero import OutputDevice, PWMOutputDevice
from dataclasses import dataclass
from time import time, sleep
from smbus2 import SMBus
import numpy as np
import argparse
import sys


# MirrorBallBot custom modules
from mbb_settings_mgr import SettingsManager


# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class MotorController:
    """Handles all I2C communication with stepper motors."""
    
    # status bit definitions (mirror RP2040 definitions)
    STATUS_BIT_ENABLED   = 0  # 1 << 0 = 1
    STATUS_BIT_MOVING    = 1  # 1 << 1 = 2
    STATUS_BIT_HOMED     = 2  # 1 << 2 = 4
    STATUS_BIT_BUSY      = 3  # 1 << 3 = 8
    STATUS_BIT_STALL     = 4  # 1 << 4 = 16
    STATUS_BIT_ERROR     = 5  # 1 << 5 = 32
    STATUS_BIT_HOMING    = 6  # 1 << 6 = 64
    STATUS_BIT_RETRACT   = 7  # 1 << 7 = 128
    
    # predefined masks for common conditions
    STATUS_MASK_HOMING = (1 << STATUS_BIT_HOMING) | (1 << STATUS_BIT_RETRACT)
    STATUS_MASK_BUSY   = (1 << STATUS_BIT_BUSY)   | (1 << STATUS_BIT_MOVING)
    STATUS_MASK_READY  = (1 << STATUS_BIT_ENABLED)  # must also check not busy
    
    # special I2C response values
    I2C_RESPONSE_NO_DATA        = 254
    I2C_RESPONSE_CHECKSUM_ERROR = 255
    
    
    def __init__(self, motors_list=None, verbose=False, settings_mgr=None):
        """
        Handles all I2C communication with stepper motors.
        
        Args:
            settings_mgr: SettingsManager instance
            verbose: If True, print debug information
        """
        
        # verbose mode
        self.verbose = verbose
        
        self.settings_mgr = settings_mgr
        
        # load motor settings if available
        if self.settings_mgr:
            settings = self.settings_mgr.get()
            self.GPIO_ENABLE_PIN = settings.gpio.motors_drv_enable
            self.GPIO_MOT_A = settings.gpio.motor_a
            self.GPIO_MOT_B = settings.gpio.motor_b
            self.GPIO_MOT_C = settings.gpio.motor_c
            self.REVERSE    = settings.motors.reverse
            self.MAX_SPEED  = settings.motors.max_speed
            self.MIN_SPEED  = settings.motors.min_speed
            self.BASE_SPEED = settings.motors.base_speed
        else:
            # fallback defaults
            self.GPIO_ENABLE_PIN =  4
            self.GPIO_MOT_A = 18
            self.GPIO_MOT_B = 20
            self.GPIO_MOT_C = 21
            self.REVERSE    = False
            self.MAX_SPEED  = 65480
            self.MIN_SPEED  = 5000
            self.BASE_SPEED = 55000
        

        # I2C setup
        self.bus = SMBus(1)
        
        if motors_list is None:
            motors_list = ['A', 'B', 'C']
        else:
            motors_list = motors_list
        
        self.motors = {}
        if 'A' in motors_list:
            self.motors['A'] = 0x41
        if 'B' in motors_list:
            self.motors['B'] = 0x42
        if 'C' in motors_list:
            self.motors['C'] = 0x43
        
        print()
        for motor in self.motors:
            print(f"motor {motor} @ I2C {hex(self.motors[motor])}", end=";  ")
        print()
        
        # communication constants
        self.STX = 0x02
        self.ETX = 0x03
        
        # hard reset the RP2040 boards
        self.reset_pico()
        
        # Pin object to enable/disable the stepper drivers
        self.motors_enable_pin = OutputDevice(self.GPIO_ENABLE_PIN)
        
        # disable the motors (no current to their coils)
        # motors will be enabled when positioning th platform the first time
        self.disable_motors()
        
        # variable to track the motor enable status
        self.motors_enabled = False
        
        # dict of Pins objects to syncronize the motors
        self.motors_sync_pins = {
            'A': OutputDevice(self.GPIO_MOT_A),
            'B': OutputDevice(self.GPIO_MOT_B),
            'C': OutputDevice(self.GPIO_MOT_C)
            }
        
        # set the motors syncronization pins to low
        # when data at the PIO buffer, the stepper start at rising edge of the sync_pin
        for pin in self.motors_sync_pins.values():
            pin.off()
        
        # stepper direction as per default or reverse it
        self.reverse_motors(self.REVERSE)
        
        # PIO parameters
        self.PIO_FIX = 8
        self.PIO_VAR = 4
        self.PIO_FREQUENCY = 5_000_000
        self.PIO_MULTIPLIER = 8              # factor increasing the received speed via I2C to the one used by the PIO
        
        # speed control
        self.MAX_NORMALIZED_SPEED = 100_000
        self.MIN_DELAY = 100
        self.MAX_DELAY = 32767
        self.STOP_WORD = 32768
                
        self.f_min = self.PIO_FREQUENCY // (self.PIO_MULTIPLIER * self.MAX_DELAY * self.PIO_VAR + self.PIO_FIX)
        self.f_max = self.PIO_FREQUENCY // (self.PIO_MULTIPLIER * self.MIN_DELAY * self.PIO_VAR + self.PIO_FIX)
        self.f_range = self.f_max - self.f_min
        
        if self.verbose:
            print("\nMotorController initialized")
    
    
    
    
    
    def normalized_speed_to_delay(self, norm_speed: int) -> tuple[int, int]:
        """Convert normalized speed to PIO delay value."""
        norm_speed = max(-self.MAX_NORMALIZED_SPEED, min(self.MAX_NORMALIZED_SPEED, norm_speed))
        
        if abs(norm_speed) < 1:
            return 0, 0
        
        f_target = self.f_min + self.f_range * abs(norm_speed) // self.MAX_NORMALIZED_SPEED
        
        if f_target == 0:
            return 0, 0
            
        pio_delay = (self.PIO_FREQUENCY // f_target - self.PIO_FIX) // (self.PIO_MULTIPLIER * self.PIO_VAR)
        pio_delay = max(self.MIN_DELAY, min(self.MAX_DELAY, pio_delay))
        
        # determine stepper direction, based on speed >0 and self.REVERSE
        needs_inversion = (norm_speed > 0) == self.REVERSE
        
        # define the speed_word to send to the RP2040
        speed_word = pio_delay if needs_inversion else 65535 - pio_delay
        
        return pio_delay, speed_word
    
    
    
    def calculate_movement_time(self, pio_delay: int, steps: int) -> int:
        """Calculate movement time in milliseconds."""
        if pio_delay == 0:
            return 10         # 10 ms
        
        if steps <= 0:
            return 10         # 10 ms
        
        elif steps < 4:
            step_time_us = (self.PIO_MULTIPLIER * pio_delay * self.PIO_VAR + self.PIO_FIX) * 1000000 // self.PIO_FREQUENCY
            return (steps * step_time_us) // 1000
        
        # long sequence of is cases, turned out to be faster than several other approaches
        elif steps >= 32:
            steps_ramp = 8
        elif steps >= 28:
            steps_ramp = 7
        elif steps >= 24:
            steps_ramp = 6
        elif steps >= 20:
            steps_ramp = 5
        elif steps >= 16:
            steps_ramp = 4
        elif steps >= 12:
            steps_ramp = 3
        elif steps >= 8:
            steps_ramp = 2
        elif steps >= 4:
            steps_ramp = 1

        speed_reduction = 1.5
        delay_ramp = min(int(speed_reduction * pio_delay), 262143)
        step_time_ramp_us = (self.PIO_MULTIPLIER * delay_ramp * self.PIO_VAR + self.PIO_FIX) * 1000000 // self.PIO_FREQUENCY
        step_time_us = (self.PIO_MULTIPLIER * pio_delay * self.PIO_VAR + self.PIO_FIX) * 1000000 // self.PIO_FREQUENCY
        movement_time_us = (2 * steps_ramp * step_time_ramp_us)
        movement_time_us += ((steps - 2 * steps_ramp) * (step_time_us))
        return movement_time_us//1000

    
    def i2c_test(self, num=3):
        """Send a I2C query command to all motors (RP2040)."""
        responses = 0
        for motor in self.motors:
            # send motor status query commands (special command 7)
            ret = self.send_command(motor, 7, 1)
            if ret is not None:
                responses += 1
                sleep(0.01)
                if self.verbose:
                    print(f"Motor {motor} status is: {ret}")
        if responses == num:
            return True
        else:
            return False
    
    
    def calculate_checksum(self, dataframe):
        return sum(dataframe) & 0xFF
    
    
    def escape_data(self, dataframe):
        escaped_data = []
        for byte in dataframe:
            if byte in [self.STX, self.ETX, 0x5C]:
                escaped_data.append(0x5C)
            escaped_data.append(byte)
        return escaped_data
    
    
    def send_command(self, motor: str, speed_word: int, steps: int):
        """Send command to a single motor using I2C speed_word."""
        
        # special handling for status request (command 7)
        if speed_word == 7:
            # just read status, no need to send data
            # the I2C handler will return status on read
            return True
        
        data_frame = [self.STX]
        for value in [speed_word, steps]:
            field_bytes = value.to_bytes(2, byteorder='big')
            data_frame += list(field_bytes)
        
        checksum = self.calculate_checksum(data_frame)
        data_frame.append(checksum)
        escaped_data = self.escape_data(data_frame)[1:] + [self.ETX]
        
        address = self.motors[motor]
        try:
            self.bus.write_i2c_block_data(address, 0, escaped_data)
            return True
        except Exception as e:
            print(f"I2C error motor {motor}: {e}")
            return False
    
    
    def read_response(self, motor: str):
        """Read response from motor."""
        address = self.motors[motor]
        try:
            response = self.bus.read_byte(address)
            return response
        except Exception as e:
            print(f"I2C read error motor {motor}: {e}")
            return self.I2C_RESPONSE_CHECKSUM_ERROR  # Return checksum error on exception
    
    
    def sync_motors(self, motors: str):
        """
        Sync multiple motors.
        The PIO stepper generator waits for a GPIO pin to be high to start.
        This is used to activate the 3 stepper in parallel, after sending them the
        instruction in sequence via the I2C.
        Not tested if really needed...but it sounds a robust approach.
        """
    
        # get the pin objects for the requested motors
        active_pins = [self.motors_sync_pins[m] for m in motors if m in self.motors_sync_pins]
        
        # set the pin on
        for pin in active_pins:
            pin.on()
        
        # short wait time
        sleep(0.01)
        
        # set the pin off again
        for pin in active_pins:
            pin.off()
    
    
    
    def enable_motors(self):    
        """Enable all motors (EN pin of the stepper driver)."""
        self.motors_enable_pin.off()
        
        # send enable commands (special command 3)
        for motor in self.motors:
            self.send_command(motor, 3, 1)
            sleep(0.1)
        self.motors_enabled = True
    
    
    
    def disable_motors(self):
        """Disable all motors (EN pin of the stepper driver)."""
        self.motors_enable_pin.on()
        
        # send enable commands (special command 2)
        for motor in self.motors:
            self.send_command(motor, 2, 1)
            sleep(0.1)
        self.motors_enabled = False
    
    
    
    def reverse_motors(self, reverse):
        """
        Maintain or reverse the direction of the stepper motors
        0 (or False) maintains the default direction (my MBB)
        1 (or True) reverses the default direction (compared to my MBB)
        """
        
        # send reverse command (special command 8)
        for motor in self.motors:
            self.send_command(motor, 8, int(reverse))
            sleep(0.1)

    
    
    def reset_pico(self):
        """
        Hard reset of the RP2040 or RP2350, via I2C command.
        Necessary to wait at least 6 seconds for the RP2040 to reboot,
        prior to send other commands
        """
        
        print("\nHard reset of the RP2040 (wait 6 seconds to reboot)") 
        # send reset command to the board (special command 9)
        for motor in self.motors:
            self.send_command(motor, 9, 1)
            if self.verbose:
                print(f"Reset of the RP board for motor {motor}")
        sleep(6)
        print()
    
    
    
    def decode_status(self, status_byte):
        """Decode status byte into human-readable string."""
        if status_byte == self.I2C_RESPONSE_NO_DATA:
            return "NO_DATA"
        if status_byte == self.I2C_RESPONSE_CHECKSUM_ERROR:
            return "CHECKSUM_ERROR"
        
        states = []
        if status_byte & (1 << self.STATUS_BIT_ENABLED): states.append("ENABLED")
        if status_byte & (1 << self.STATUS_BIT_MOVING):  states.append("MOVING")
        if status_byte & (1 << self.STATUS_BIT_HOMED):   states.append("HOMED")
        if status_byte & (1 << self.STATUS_BIT_BUSY):    states.append("BUSY")
        if status_byte & (1 << self.STATUS_BIT_STALL):   states.append("STALL")
        if status_byte & (1 << self.STATUS_BIT_ERROR):   states.append("ERROR")
        if status_byte & (1 << self.STATUS_BIT_HOMING):  states.append("HOMING")
        if status_byte & (1 << self.STATUS_BIT_RETRACT): states.append("RETRACT")
        
        return " | ".join(states) if states else "IDLE"
    
    
    
    def read_status(self, motor):
        """Read motor status."""
        try:
            status = self.bus.read_byte(self.motors[motor])
            return status
        except Exception as e:
            print(f"Read error: {e}")
            return self.I2C_RESPONSE_CHECKSUM_ERROR

    
    
    def request_status_update(self, motor):
        """Request a status update from RP2040 (command 7)."""
        self.send_command(motor, 7, 1)
        sleep(0.01)
        return self.read_status(motor)
    
    
    
    def wait_for_status(self, motor, expected_bits=None, clear_bits=None, timeout_s=2, interval_s=0.05):
        """
        Wait for motor status to meet conditions.
        
        Args:
            motor: Motor identifier
            expected_bits: Bits that must be set (bitmask)
            clear_bits: Bits that must be cleared (bitmask)
            timeout_s: Maximum time to wait in seconds
            interval_s: Time between status checks
        
        Returns:
            tuple: (success, final_status)
        """
        start_time = time()
        
        while (time() - start_time) < timeout_s:
            # request fresh status
            status = self.request_status_update(motor)
            
            # skip invalid responses
            if status in (self.I2C_RESPONSE_NO_DATA, self.I2C_RESPONSE_CHECKSUM_ERROR):
                sleep(interval_s)
                continue
            
            # check conditions
            conditions_met = True
            if expected_bits is not None:
                if (status & expected_bits) != expected_bits:
                    conditions_met = False
            if clear_bits is not None:
                if (status & clear_bits) != 0:
                    conditions_met = False
            
            if conditions_met:
                return True, status
            
            sleep(interval_s)
        
        return False, self.request_status_update(motor)






def main(motors_list=3):
    """Motors test"""

    # create a setting manager instance
    settings_mgr = SettingsManager()
     
    # load motor settings
    if settings_mgr:
        settings = settings_mgr.get()
        FULL_STEPS_PER_REV = settings.motors.full_steps_per_rev
        MICROSTEPS = settings.motors.microsteps
        REVERSE    = settings.motors.reverse
        MAX_SPEED  = settings.motors.max_speed
        MIN_SPEED  = settings.motors.min_speed
        BASE_SPEED = settings.motors.base_speed
    else:
        # fallback defaults
        FULL_STEPS_PER_REV = 200
        REVERSE    = False
        MAX_SPEED  = 65480
        MIN_SPEED  = 5000
        BASE_SPEED = 55000
    
    full_rev = int(MICROSTEPS * FULL_STEPS_PER_REV)
    
    # create an instance to the motor controller (I2C handler with the motor)
    mc = MotorController(motors_list=motors_list, verbose=True, settings_mgr=settings_mgr)

    
    
    def low_speed_CCW(steps):
        slow_speed_pio, min_speed_word = mc.normalized_speed_to_delay(-MIN_SPEED)
        slow_rotation_time_s = mc.calculate_movement_time(slow_speed_pio, steps)/1000
        
        # enable the motors
        mc.enable_motors()
        sleep(0.1)
        
        # send the speed and number od steps
        for motor in mc.motors:
            success = mc.send_command(motor, min_speed_word, steps)
            sleep(0.01)
        
        # start the motors
        for motor in mc.motors:
            mc.sync_motors(motor)
        
        print(f"\nMotors {motors_list} rotating CCW at low speed (ca {round(slow_rotation_time_s,1)} seconds)")
        print("[note 1] Change 'reverse' setting at mbb_settings.json if CW rotation.")
        print("[note 2] Check 'microsteps' and 'full_steps_per_rev' settings if large time difference.")
        
        # wait for the slow speed totation time
        sleep(0.1 + slow_rotation_time_s)
        
    
    
    def high_speed_CW(steps):
        high_speed_pio, high_speed_word = mc.normalized_speed_to_delay(MAX_SPEED)
        fast_rotation_time_s = mc.calculate_movement_time(high_speed_pio, steps)/1000
        
        # enable the motors
        mc.enable_motors()
        sleep(0.1)
        
        # send the speed and number od steps
        for motor in mc.motors:
            success = mc.send_command(motor, high_speed_word, steps)
            sleep(0.01)
        
        # start the motors
        for motor in mc.motors:
            mc.sync_motors(motor)
        
        print(f"\nMotors {motors_list} rotating CW at high speed (ca {round(fast_rotation_time_s,1)} seconds)")
        print("[note] Change 'reverse' setting at mbb_settings.json if CCW rotation")
        
        # wait for the slow speed totation time
        sleep(0.1 + fast_rotation_time_s)
        

            
    try:
        # number of motors to test
        motors_numbers = len(motors_list)
        
        # check the I2C communication
        if mc.i2c_test(num=motors_numbers):
            print("OK: All boards responded\n")
        else:
            print("\nWarning: Not all RP2040 boards responded")
        
        
        pause = 2
        test_runs = 5
        for test_run in range(test_runs):
            
            print()
            print(f"####################    Test run {test_run + 1} out of {test_runs}   ####################")
            
            if not mc.motors_enabled:
                # enable the motors
                mc.enable_motors()
                print("Motors are enabled: Axes are blocked for 5 seconds")
                sleep(5)
            
            if mc.motors_enabled:
                # disable the motors
                mc.disable_motors()
                print("Motors are disabled: Axes are free for 5 seconds")
                sleep(5)
            
            # call the low speed test function
            low_speed_CCW(full_rev)
            
            # disable the motors
            mc.disable_motors()
            print(f"\nMotors {motors_list} are disabled: Axes are free for {pause} seconds")
            sleep(5)
            
            # call the high speed test function
            high_speed_CW(full_rev)
            
            # disable the motors
            mc.disable_motors()
            print(f"\nMotors {motors_list} are disabled: Axes are free for {pause} seconds")
            sleep(5)
            print("\n" * 3)
        
        print("\nTest completed.\n\n")
    
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    
    finally:
        mc.disable_motors()
        sleep(0.1)
    



if __name__ == "__main__":
    
    print("\n" + "="*60)
    print("MOTORS TEST")
    print("[Note] Clockwise (CW) and counterclockwise (CCW) considers")
    print("       the motors'axes pointing to the observer")
    print("="*60)
    
    motors_list=['A','B','C']
    
    # argument parser object creation
    description =f"It is possible to limit the test to specific motors (1 or more motors). If not specified, the 3 motors will be tested\n"
    parser = argparse.ArgumentParser(description=description)
    
    parser.add_argument("--A", action='store_true', help="Test motor A (motor connected to header M1)") # argument is added to the parser
    parser.add_argument("--B", action='store_true', help="Test motor B (motor connected to header M2)") # argument is added to the parser
    parser.add_argument("--C", action='store_true', help="Test motor C (motor connected to header M3)") # argument is added to the parser
    args = parser.parse_args()       # argument parsed assignement
    
    if args.A or args.B or args.C:
        motors_list = []
        if args.A == True:           # case of 'A' argument
            motors_list.append('A')  # motor 'A' is appended to the list of motors
        if args.B == True:           # case of 'B' argument
            motors_list.append('B')  # motor 'B' is appended to the list of motors
        if args.C == True:           # case of 'C' argument
            motors_list.append('C')  # motor 'C' is appended to the list of motors
        print(f"\nTest is going to be applied to motors {motors_list}")
    
    main(motors_list=motors_list)
        
    
    
    