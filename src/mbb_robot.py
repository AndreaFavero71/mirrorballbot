"""
Andrea Favero 20260607

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the robot:
   - interract with the camera (get the ball position)
   - steer the steppers via I2C communication to the RP2040
   - interract with the Paths
   - control the active cooling (fans)




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
# BALL BALANCING ROBOT by ANDREA FAVERO
# ============================================================================

__version__ = "0.0.2"

from math import sqrt, radians, degrees, cos, sin, gcd
from gpiozero import OutputDevice, PWMOutputDevice
from dataclasses import dataclass
from time import time, sleep
from smbus2 import SMBus
import numpy as np
import threading
import queue
import sys


# MirrorBallBot custom modules
from mbb_display_mgr import DisplayManager
from mbb_settings_mgr import SettingsManager
from mbb_paths import PathExecutor
import mbb_camera


# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class MotorController:
    """Handles all I2C communication with stepper motors."""
    
    # GPIO pins
#     GPIO_ENABLE_PIN =  4
#     GPIO_MOT_A      = 18
#     GPIO_MOT_B      = 20
#     GPIO_MOT_C      = 21
    
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
    
    
    
    def __init__(self, verbose=False, settings_mgr=None):
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
        self.motors = {'A': 0x41, 'B': 0x42, 'C': 0x43}
        
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

    
    def i2c_test(self):
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
        if responses == 3:
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
                print(f"Reset of the RP2040 board for motor {motor}")
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
    
    
    
    def are_motors_busy(self, timeout_s=5.0):
        """
        Check if any motor is busy (moving, homing, etc.)
        Returns (is_busy, status_message, busy_motors_list)
        """
        start_time = time()
        busy_motors = []
        
        while (time() - start_time) < timeout_s:
            busy_motors = []
            
            for motor in self.motors.keys():  # A, B, C
                # request fresh status
                status = self.request_status_update(motor)
                
                # skip invalid responses
                if status in (self.I2C_RESPONSE_NO_DATA, self.I2C_RESPONSE_CHECKSUM_ERROR):
                    busy_motors.append(f"{motor}(ERR)")
                    continue
                
                # check busy bits: BUSY, MOVING, HOMING, RETRACT
                busy_bits = (1 << self.STATUS_BIT_BUSY) | \
                            (1 << self.STATUS_BIT_MOVING) | \
                            (1 << self.STATUS_BIT_HOMING) | \
                            (1 << self.STATUS_BIT_RETRACT)
                
                if status & busy_bits:
                    busy_motors.append(motor)
            
            if not busy_motors:
                return False, "READY", []
            
            # still busy, wait and retry
            sleep(0.1)
        
        # timeout - still busy
        if busy_motors:
            return True, f"BUSY: motors {', '.join(busy_motors)}", busy_motors
        else:
            return True, "BUSY (unknown)", []




# ============================================================================
# COOLING FANS CONTROLLER
# ============================================================================

class CoolingFans:
    """Control for the fans according to the RPI temperature"""

    def __init__(self, verbose=False, settings_mgr=None):
        
        self.verbose = verbose
        self.settings_mgr = settings_mgr
        
        # load fans settings if available
        if self.settings_mgr:
            settings = self.settings_mgr.get()
            self.FANS_GPIO_PIN = settings.gpio.fans_gpio_pin
            self.FANS_PWM_FREQ = settings.fans.pwm_frequency
            self.FANS_PWM_DUTY_CYCLE = settings.fans.pwm_duty_cycle
            self.CPU_TEMP_FANS_ON = settings.fans.temp_on_celsius
            self.CPU_TEMP_FANS_HYST = settings.fans.temp_hysteresis_celsius
            self.CPU_TEMP_CHECK_INTERVAL_S = settings.fans.check_interval_s
                
        else:
            # fallback defaults
            self.FANS_GPIO_PIN = 19              # gpio pin controlling the pwm to the transistor's base
            self.FANS_PWM_FREQ = 100             # pwm frequency (higher frequency are more noisy)
            self.FANS_PWM_DUTY_CYCLE = 0.75      # pwm for the fans, to get enouf cooling yet limiting air noise
            self.CPU_TEMP_FANS_ON = 65.0         # temperature (Celsius) to swith the fans on
            self.CPU_TEMP_FANS_HYST = 5.0        # temperature histeresys (Celsius), to swith the fans off
            self.CPU_TEMP_CHECK_INTERVAL_S = 15  # period to check the RPI cpu temperature


        if self.verbose:
            print(f"Thermal control activated (Threshold: {self.CPU_TEMP_FANS_ON}°C)")
        
        self.fan = PWMOutputDevice(self.FANS_GPIO_PIN, frequency=self.FANS_PWM_FREQ)
        self.last_cpu_temp_check = time()
        self.fans_control()
    

    def get_cpu_temp(self):
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            # the file hold the CPU temp in millidegrees (ie 45000 = 45.0 degrees)
            return float(f.read()) / 1000.0
    
    
    def fans_control(self):
        now = time()
        if now - self.last_cpu_temp_check > self.CPU_TEMP_CHECK_INTERVAL_S:
            
            temp = self.get_cpu_temp()
            if temp > self.CPU_TEMP_FANS_ON:
                if self.fan.value == 0:
                    if self.verbose:
                        print(f"CPU Temp: {temp:.1f}°C  swith-on the fans")
                    self.fan.value = self.FANS_PWM_DUTY_CYCLE
            
            elif temp < self.CPU_TEMP_FANS_ON - self.CPU_TEMP_FANS_HYST:
                if self.fan.value > 0:
                    if self.verbose:
                        print(f"CPU Temp: {temp:.1f}°C  swith-off the fans")
                self.fan.value = 0
            
            self.last_cpu_temp_check = now







# ============================================================================
# BALANCE CONTROLLER (All Integers)
# ============================================================================

@dataclass
class MotorAngles:
    """Holds the three motor angles as integers (tenths of degrees)."""
    A: int  # angle in tenths of degrees (i.e. 900 = 90.0°)
    B: int
    C: int

# ============================================================================
# BALANCE CONTROLLER ( PID close to center otherwise PD )
# ============================================================================

class BalanceController:
    """Main balancing controller using integer math."""
    
    def __init__(self,
                 fans: CoolingFans,
                 motor_controller: MotorController,
                 camera: mbb_camera.Camera = None,
                 verbose = False,
                 settings_mgr = None
                 ):
        
        """
        Main balancing controller (uses integer math).
        
        Args:
            fans:             CoolingFans instance
            motor_controller: MotorController instance
            camera:           Camera instance
            verbose:          If True, print debug information
            settings_mgr:     SettingsManager instance
        
        Note for PID the Integral term: The Integral only works:
          - if k_i > 0 
          - if within an area defined by radius INTEGRAL_ZONE_TENTHS from target (activation zone, in tenths of mm)
          - if outside a smaller area defined by radius DEADZONE_TENTHS from target (to prevent useless jitter),
            unless the counter self.deadzone_couter exceeds the threshold DEADZONE_COUNTER_THR
        The integral windup reset once the ball moves outside the INTEGRAL_ZONE_TENTHS
        """
            
        self.fans = fans
        self.mc = motor_controller
        self.camera = camera
        self.verbose = verbose
        self.settings_mgr = settings_mgr
        
        # pass verbose to the components
        self.fans.verbose = verbose
        self.mc.verbose = verbose
        
        # load settings
        if self.settings_mgr:
            settings = self.settings_mgr.get()
            
            # motor specs, used to discretize movements and platform position tracking
            self.FULL_STEPS_PER_REV = settings.motors.full_steps_per_rev
            self.MICROSTEPS         = settings.motors.microsteps
            self.STEP_ANGLE_TENTHS  = settings.motors.step_angle_tenths
            
            # motor balance shifts, for fine tuning levelling of the platform
            self.BA_SHIFT_A = settings.motors.balance.shifts["A"]
            self.BA_SHIFT_B = settings.motors.balance.shifts["B"]
            self.BA_SHIFT_C = settings.motors.balance.shifts["C"]
            
            # balance position steps
            self.BALANCE_STEPS_FROM_HOME = settings.motors.balance.steps_from_home
            
            # controller settings
            self.TARGET_CYCLE_TIME_MS = settings.controller.target_cycle_time_ms
            self.BALANCE_ANGLE        = settings.controller.balance_angle_tenths
            self.MAX_TILT_TENTHS      = settings.controller.max_tilt_tenths
            self.DEADZONE_TENTHS      = settings.controller.deadzone_tenths
            self.DEADZONE_COUNTER_THR = settings.controller.deadzone_counter_thr
            self.INTEGRAL_ZONE_TENTHS = settings.controller.integral_zone_tenths
            self.MAX_INTEGRAL         = settings.controller.max_integral
            
            # PID gains
            self.k_p = settings.pid.kp
            self.k_i = settings.pid.ki
            self.k_d = settings.pid.kd 
            
            # speed control
            self.BASE_SPEED =        settings.motors.base_speed
            self.MAX_SPEED =         settings.motors.max_speed
            self.MIN_SPEED =         settings.motors.min_speed
            self.POSITIONING_SPEED = settings.motors.positioning_speed
            
            # homing settings
            self.HOMING_SPEED_FREQ = settings.motors.homing.speed_freq
            self.HOMING_SG_K_FACTOR = settings.motors.homing.sg_k_factor
            self.HOMING_SG_SHIFTS = settings.motors.homing.sg_shifts
            
        else:
            print("Warning: File settings_manager.py not found. Using hardcoded settings")
            
            # fallback defaults (hardcoded values)
            self.FULL_STEPS_PER_REV = 200
            self.MICROSTEPS = 8
            self.BALANCE_STEPS_FROM_HOME = 220
            self.BA_SHIFT_A = 6
            self.BA_SHIFT_B = 10
            self.BA_SHIFT_C = -6
            self.MAX_TILT_TENTHS = 400
            self.TARGET_CYCLE_TIME_MS = 60
            self.BALANCE_ANGLE = 900
            self.DEADZONE_TENTHS = 50
            self.DEADZONE_COUNTER_THR = 15
            self.INTEGRAL_ZONE_TENTHS = 400
            self.MAX_INTEGRAL = 3000
            self.k_p = 1.3
            self.k_i = 0.0
            self.k_d = 1.0
            self.STEP_ANGLE_TENTHS = 2.25
            self.BASE_SPEED = 55000
            self.MAX_SPEED = 65480
            self.MIN_SPEED = 5000
            self.HOMING_SPEED_FREQ = 800
            self.HOMING_SG_K_FACTOR = 45
            self.HOMING_SG_SHIFTS = {'A': -5, 'B': -5, 'C': 0}
            
        
        self.current_angles = MotorAngles(0, 0, 0)  # arbitrary value is assigned at the start
        
        # state Tracking
        self.at_home = False           # physical end stops, reached via sensorless homing of the steppers
        self.at_balance = False        # at operating position
        
        # target position (center)
        self.target_x = 0
        self.target_y = 0        
        
        # PID error accumulation
        self.integral_x = 0
        self.integral_y = 0
        self.last_error_x = 0
        self.last_error_y = 0
        
        
        self.last_t_check = time()
        
        # camera conversion from pixels to tenth of mm
        # the platform visible diameter is 220 mm
        if self.camera:
            self.pixels_to_tenths_mm = round(10 * self.camera.pixels_to_mm)
            self.tenths_mm_to_pixels = round(0.1 * 1/self.camera.pixels_to_mm)
        else:
            self.pixels_to_tenths_mm = 10 * 220 / self.camera.width if self.camera else 6  # fallback
            self.tenths_mm_to_pixels = 0
        
        
        # stepepr angle per single step, by using integeres (numerator and denominator)
        self.steps_scaling_numerator, self.steps_scaling_denominator = self.get_step_scaling(self.FULL_STEPS_PER_REV,
                                                                                             self.MICROSTEPS)

        # counter for consecutive cycles in deadzone
        self.deadzone_couter = 0
        
        # auto-balancing control
        self.auto_balance = False
        self.movement_in_progress = False

        # verbosity control
        self.print_interval = 1.0  # seconds between status prints
        self.last_status_print = time()
        
        if self.verbose:
            print(f"\n" + "="*60)
            print("BALANCE CONTROLLER")
            print("="*60)
            print(f"Vision system: {'ENABLED' if camera else 'DISABLED'}")
            print(f"PID gains: k_p={self.k_p}, k_i={self.k_i}, k_d={self.k_d}")
            print(f"Max tilt: {self.MAX_TILT_TENTHS/10:.1f}°")
            print(f"Integral zone: {self.INTEGRAL_ZONE_TENTHS/10:.1f}mm")
            print(f"Deadzone: {self.DEADZONE_TENTHS/10:.1f}mm")
            print(f"Speed range: {self.MIN_SPEED:,} to {self.MAX_SPEED:,}")
            print(f"[Init] Stepper resolution = {self.STEP_ANGLE_TENTHS:.3f} tenths per microstep")
            print("="*60)
    
    
    
    
    def get_step_scaling(self, steps_per_rev, microstepping):
        """
        Calculates numerator and denominator to get the motor angle (tenth of degrees)
        per each step of the stepper motor
        
        Example:
        Motor specs 200 steps per revolution and 1/8 microstepping
        motor = 1600 steps per revolution
        In degrees_tenths: 2.25 tenths per step
        Steps = degrees_tenths/2.25 = degrees_tenths * 100/225 = degrees_tenths * 4/9
        """
        numerator = steps_per_rev * microstepping
        denominator = 3600

        g = gcd(numerator, denominator)
        
#         self.steps_scaling_numerator = num // g
#         self.steps_scaling_denominator = den // g
        
        return numerator//g, denominator//g
    
    
    
    def pixel_to_tenths_mm(self, pixel_x: int, pixel_y: int):
        """Convert pixel coordinates to tenths of millimeters."""
        if pixel_x == -9999 or pixel_y == -9999:
            return 0, 0
            
        # simple multiplication
        x_tenths_mm = pixel_x * self.pixels_to_tenths_mm
        y_tenths_mm = pixel_y * self.pixels_to_tenths_mm
            
        return x_tenths_mm, y_tenths_mm
    
    
    def reset_pid(self, last_error=True):
        """Reset the PID integral derivative terms"""
        self.integral_x = 0
        self.integral_y = 0
        if last_error:
            self.last_error_x = 0
            self.last_error_y = 0

    def calculate_target_angles(self, ball_x_tenths_mm: int, ball_y_tenths_mm: int, ball_visible: bool = True) -> MotorAngles:
        """
        Calculate target motor angles using PID control.
        Integral term is only accumulated when ball is near center.
        """
        
        t_check = time()
        dt = t_check - self.last_t_check
        
        # handle invalid time difference
        if dt <= 0:
            dt = 0.01  # default to 10ms
        
        # handle ball visibility
        if not ball_visible:
            # ball not visible - reset integral terms
            self.reset_pid()
            return MotorAngles(self.BALANCE_ANGLE, self.BALANCE_ANGLE, self.BALANCE_ANGLE)
        
        # calculate error relative to target (center)
        error_x = ball_x_tenths_mm - self.target_x
        error_y = ball_y_tenths_mm - self.target_y
        
        # skip processing if within deadzone (too small to care)
        if self.k_i > 0:
            if abs(error_x) < self.DEADZONE_TENTHS and abs(error_y) < self.DEADZONE_TENTHS:
                self.deadzone_couter += 1
                if self.deadzone_couter > self.DEADZONE_COUNTER_THR:
                    # small gradual reduction of integral when near target
                    if abs(self.integral_x) > 10 or abs(self.integral_y) > 10:
                        self.integral_x = int(self.integral_x * 0.95)
                        self.integral_y = int(self.integral_y * 0.95)
            else:
                # reset self.deadzone_couter when not in deadzone range
                self.deadzone_couter = 0
        
        if self.verbose:
            print(f"\nCycle time: {round(dt,3)}s, Error: ({error_x/10:.1f}, {error_y/10:.1f})mm")
        
        # calculate derivative (rate of change of error)
        if dt > 0 and hasattr(self, 'last_position_x') and self.last_position_x is not None:
            derivative_x = (ball_x_tenths_mm - self.last_position_x) / dt
            derivative_y = (ball_y_tenths_mm - self.last_position_y) / dt
        else:
            derivative_x = 0
            derivative_y = 0
            if not hasattr(self, 'last_position_x') or self.last_position_x is None:
                self.last_position_x = ball_x_tenths_mm
                self.last_position_y = ball_y_tenths_mm
        
        # store current position for next derivative calculation
        self.last_position_x = ball_x_tenths_mm
        self.last_position_y = ball_y_tenths_mm
    
    
        # case the Integral term is > zero
        if self.k_i > 0:
            use_integral = True
            # only accumulate integral when error is small
            if (abs(error_x) < self.INTEGRAL_ZONE_TENTHS and 
                abs(error_y) < self.INTEGRAL_ZONE_TENTHS):
                
                # ball is near center - accumulate integral for precise centering
                self.integral_x += error_x * dt
                self.integral_y += error_y * dt
                
                # limit integral terms to prevent windup
                self.integral_x = max(-self.MAX_INTEGRAL, min(self.MAX_INTEGRAL, self.integral_x))
                self.integral_y = max(-self.MAX_INTEGRAL, min(self.MAX_INTEGRAL, self.integral_y))
                
                if self.verbose:
                    integral_status = "ACTIVE (near center)"
            else:
                # ball is far from center - reset integral to prevent fighting disturbances
                self.reset_pid(last_error=False)
                use_integral = False
                if self.verbose:
                    integral_status = "INACTIVE (far from center)"
        else:
            use_integral = False
        
        # PID calculation
        if use_integral:
            # full PID with integral (for precise centering)
            tilt_x = -(self.k_p * error_x + self.k_i * self.integral_x + self.k_d * derivative_x) // 10
            tilt_y = -(self.k_p * error_y + self.k_i * self.integral_y + self.k_d * derivative_y) // 10
        else:
            # PD only (no integral) for disturbance rejection
            tilt_x = -(self.k_p * error_x + self.k_d * derivative_x) // 10
            tilt_y = -(self.k_p * error_y + self.k_d * derivative_y) // 10
        
        # limit total tilt to prevent excessive platform angles
        total_tilt_squared = tilt_x * tilt_x + tilt_y * tilt_y
        max_tilt_squared = self.MAX_TILT_TENTHS * self.MAX_TILT_TENTHS
        
        if total_tilt_squared > max_tilt_squared:
            # scale down tilt to stay within limits
            total_tilt = int(sqrt(total_tilt_squared))
            if total_tilt > 0:
                scale = (self.MAX_TILT_TENTHS * 100) // total_tilt
                tilt_x = (tilt_x * scale) // 100
                tilt_y = (tilt_y * scale) // 100
                # also reduce integral to prevent windup
                self.integral_x = (self.integral_x * scale) // 100
                self.integral_y = (self.integral_y * scale) // 100
        
        if self.verbose:
            if use_integral:
                print(f"Integral: {integral_status}")
                print(f"PID: P_x={self.k_p*error_x/10:.1f}, I_x={self.k_i*self.integral_x/10:.1f}, D_x={self.k_d*derivative_x/10:.1f}")
                print(f"PID: P_y={self.k_p*error_y/10:.1f}, I_y={self.k_i*self.integral_y/10:.1f}, D_y={self.k_d*derivative_y/10:.1f}")
            else:
                print(f"PD only: P_x={self.k_p*error_x/10:.1f}, D_x={self.k_d*derivative_x/10:.1f}")
                print(f"PD only: P_y={self.k_p*error_y/10:.1f}, D_y={self.k_d*derivative_y/10:.1f}")
        
            print(f"CONTROL: Tilt=({tilt_x/10:.1f},{tilt_y/10:.1f})°")
        
        # convert tilt to motor angles
        if tilt_x == 0 and tilt_y == 0:
            return MotorAngles(self.BALANCE_ANGLE, self.BALANCE_ANGLE, self.BALANCE_ANGLE)
        
        # use floats for trigonometric calculations
        tilt_x_deg = tilt_x / 10.0
        tilt_y_deg = tilt_y / 10.0
        
        tilt_x_rad = radians(tilt_x_deg)
        tilt_y_rad = radians(tilt_y_deg)
        
        # motor positions at 90°, 210°, 330° (120° apart)
        motor_positions_rad = [
            radians(90),   # A: back
            radians(210),  # B: front-left
            radians(330)   # C: front-right
        ]
        
        # calculate angle delta for each motor based on tilt
        deltas = []
        for pos in motor_positions_rad:
            delta_rad = (-cos(pos) * tilt_x_rad - sin(pos) * tilt_y_rad)
            delta_deg = degrees(delta_rad)
            deltas.append(int(round(delta_deg * 10)))  # convert to tenths of degrees
        
        # calculate target angles for each motor
        targets = MotorAngles(
            A = self.BALANCE_ANGLE + deltas[0],  # start from 90.0° (center)
            B = self.BALANCE_ANGLE + deltas[1],
            C = self.BALANCE_ANGLE + deltas[2]
        )

        
        # safety limits to prevent excessive motor angles
        targets.A = max(300, min(targets.A, 1500))  # limit to 30.0° - 150.0°
        targets.B = max(300, min(targets.B, 1500))
        targets.C = max(300, min(targets.C, 1500))
        
        if self.verbose:
            print(f"CONTROL: Targets: A={targets.A/10:.1f}°, B={targets.B/10:.1f}°, C={targets.C/10:.1f}°")
        
        # store errors for next derivative calculation
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_t_check = t_check
        
        return targets
    
    
    def set_angle(self, angle_A=None, angle_B=None, angle_C=None):
        """Set current motor angles, defaulting to BALANCE_ANGLE."""
        if angle_A is None:
            angle_A = self.BALANCE_ANGLE
        if angle_B is None:
            angle_B = self.BALANCE_ANGLE
        if angle_C is None:
            angle_C = self.BALANCE_ANGLE
        
        self.current_angles = MotorAngles(angle_A, angle_B, angle_C)
    
    
    def quantize_angle(self, angle_tenths: float) -> float:
        """Round a motor angle (in tenths of degree) to the nearest reachable microstep."""
        return round(angle_tenths / self.STEP_ANGLE_TENTHS) * self.STEP_ANGLE_TENTHS
    
    
    
    def wait_for_motors_ready(self, timeout_s=5.0):
        """Wait for motors to be ready for new commands."""
        return self.mc.are_motors_busy(timeout_s=timeout_s)
    
    
    
    def move_to_angles(self, target_angles: MotorAngles):
        """Move motors to target angles with speed proportional to step count."""
        if self.verbose:
            print(f"Current: A={self.current_angles.A/10:.1f}°, B={self.current_angles.B/10:.1f}°, C={self.current_angles.C/10:.1f}°")
            print(f"Target:  A={target_angles.A/10:.1f}°, B={target_angles.B/10:.1f}°, C={target_angles.C/10:.1f}°")
        
        # quantize target angles to nearest microstep
        target_angles = MotorAngles(
            A=self.quantize_angle(target_angles.A),
            B=self.quantize_angle(target_angles.B),
            C=self.quantize_angle(target_angles.C)
        )
        
        # calculate angle differences
        diff_A = target_angles.A - self.current_angles.A
        diff_B = target_angles.B - self.current_angles.B
        diff_C = target_angles.C - self.current_angles.C
        
        if self.verbose:
            print(f"Diffs: ΔA={diff_A/10:.1f}°, ΔB={diff_B/10:.1f}°, ΔC={diff_C/10:.1f}°")
        
        self.movement_in_progress = True
        
        # calculate steps needed for each motor
        steps_A = int((abs(diff_A) * self.steps_scaling_numerator) // self.steps_scaling_denominator)
        steps_B = int((abs(diff_B) * self.steps_scaling_numerator) // self.steps_scaling_denominator)
        steps_C = int((abs(diff_C) * self.steps_scaling_numerator) // self.steps_scaling_denominator)
        
        # update current angles to targets
        self.current_angles = target_angles
        
        max_steps = max(steps_A, steps_B, steps_C)
        
        if max_steps == 0:
            self.movement_in_progress = False
            if self.verbose:
                print("No steps to move")
            return 0
        
        def calculate_speed_for_cycle_time(steps, diff, max_steps, target_time_ms=60):
            """Calculate speed to complete movement within target """
            if steps == 0:
                return 0
            
            # base speed calculation
            if steps == max_steps:
                # this motor determines the movement time
                base_speed = self.BASE_SPEED
            else:
                # other motors need to move faster to finish together
                speed_factor = (max_steps * 100) // steps
                base_speed = min(self.BASE_SPEED * speed_factor // 100, self.MAX_SPEED)
            
            # adjust speed based on target cycle time
            if steps > 0:
                # calculate current expected time for this speed
                pio_delay, _ = self.mc.normalized_speed_to_delay(base_speed)
                expected_time_ms = self.mc.calculate_movement_time(pio_delay, steps)
                
                # adjust speed to meet target time
                if expected_time_ms > target_time_ms:
                    # need to go faster
                    speed_factor = (expected_time_ms * 100) // target_time_ms
                    adjusted_speed = min(base_speed * speed_factor // 100, self.MAX_SPEED)
                else:
                    # can go slower for smoother movement
                    # but don't go below minimum speed for very small movements
                    if steps < 10: #5:  # very small movement
                        adjusted_speed = self.MIN_SPEED
                    else:
                        speed_factor = (target_time_ms * 100) // max(expected_time_ms, 1)
                        adjusted_speed = max(self.MIN_SPEED, base_speed * 100 // speed_factor)
            else:
                adjusted_speed = base_speed
            
            return adjusted_speed if diff > 0 else -adjusted_speed
        
        # calculate speeds for each motor
        norm_speed_A = calculate_speed_for_cycle_time(steps_A, diff_A, max_steps, self.TARGET_CYCLE_TIME_MS)
        norm_speed_B = calculate_speed_for_cycle_time(steps_B, diff_B, max_steps, self.TARGET_CYCLE_TIME_MS)
        norm_speed_C = calculate_speed_for_cycle_time(steps_C, diff_C, max_steps, self.TARGET_CYCLE_TIME_MS)
        
        if self.verbose:
            print(f"Steps: A={steps_A}, B={steps_B}, C={steps_C}")
            print(f"Speeds: A={norm_speed_A:,}, B={norm_speed_B:,}, C={norm_speed_C:,}")
        
        # send commands to motors
        motors_to_sync = ""
        movement_time_ms = 0
        
        motors = [
            ('A', norm_speed_A, steps_A),
            ('B', norm_speed_B, steps_B),
            ('C', norm_speed_C, steps_C)
        ]
        
        for motor, norm_speed, steps in motors:
            if norm_speed == 0 or steps == 0:
                if self.verbose:
                    print(f"Motor {motor}: speed 0 or steps 0, skipping")
                continue
            
            pio_delay, speed_word = self.mc.normalized_speed_to_delay(norm_speed)
            
            # ensure speed_word is valid
            if speed_word == 0 or speed_word == self.mc.STOP_WORD:
                if self.verbose:
                    print(f"Motor {motor}: invalid speed_word {speed_word}, skipping")
                continue
            
            if self.verbose:
                print(f"Motor {motor}: speed={norm_speed:,}, delay={pio_delay}, word={speed_word}, steps={steps}")
            
            success = self.mc.send_command(motor, speed_word, steps)
            
            if success:
                motor_time = self.mc.calculate_movement_time(pio_delay, steps)
                movement_time_ms = max(movement_time_ms, motor_time)
                motors_to_sync += motor
                if self.verbose:
                    print(f"Motor {motor} command sent, est time: {motor_time}ms")
            else:
                if self.verbose:
                    print(f"Motor {motor}: send failed")
        
        # sync and wait for movement completion
        if motors_to_sync:
            if self.verbose:
                print(f"Syncing motors: {motors_to_sync}")
            self.mc.sync_motors(motors_to_sync)
            
            # add small safety margin
            wait_time_ms = movement_time_ms #+ 5
            wait_time_sec = wait_time_ms / 1000.0
            
            if self.verbose:
                print(f"Waiting {wait_time_ms}ms ({wait_time_sec:.3f}s) for movement")
            
            # wait for movement to complete
            sleep(wait_time_sec)
            
            if self.verbose:
                print(f"Movement complete in {movement_time_ms}ms")
        else:
            if self.verbose:
                print("No motors to sync")
        
        self.movement_in_progress = False
        return movement_time_ms
    
    
    def auto_balance_loop(self, stop_event: threading.Event = None):
        """Automatic balancing loop with ball visibility check."""
        if self.verbose:
            print("\n\n\n" + "="*60)
            print("STARTING AUTO-BALANCE MODE")
            print("="*60 + "\n")
        
        self.auto_balance = True
        last_print_time = time()
        status_interval_s = 3               # time interval (s) to print the status
        last_cpu_temp_check = time()
        cycle_count = 0
        movement_count = 0
        last_cycle_count = 0
        consecutive_missed_frames = 0
        MAX_MISSED_FRAMES = 12             # reset integral after max missed frames
        
        # reset PID accumulators
        self.reset_pid()
        
        t_ref = time()
        while self.auto_balance and (stop_event is None or not stop_event.is_set()):
            try:
                t_now = time()
                
                if t_now - t_ref > 0.075:
                    t_ref = t_now
                    cycle_count += 1
                    
                    # don't process if movement is in progress
                    if self.movement_in_progress:
                        sleep(0.002)
                        continue
                    
                    # get ball position directly from camera
                    if self.camera:
                        x, y = self.camera.get_ball_position(block=False)
                        ball_visible = (x != -9999 and y != -9999)
                        fps = self.camera.fps
                    else:
                        x, y = -9999, -9999
                        ball_visible = False
                        fps = 0
                    
                    # track consecutive missed frames
                    if ball_visible:
                        consecutive_missed_frames = 0
                    else:
                        consecutive_missed_frames += 1
                    
                    if ball_visible:
                        # convert to tenths of mm
                        x_tenths_mm, y_tenths_mm = self.pixel_to_tenths_mm(x, y)
                        
                        if not hasattr(self, 'last_position_x') or self.last_position_x is None:
                            self.last_position_x = x_tenths_mm
                            self.last_position_y = y_tenths_mm
                        
                        # calculate target angles
                        target_angles = self.calculate_target_angles(x_tenths_mm, y_tenths_mm, ball_visible=True)
                        
                        # only move if there is a significant target
                        if abs(target_angles.A - self.current_angles.A) >= 10 or \
                           abs(target_angles.B - self.current_angles.B) >= 10 or \
                           abs(target_angles.C - self.current_angles.C) >= 10:
                            # move to target angles
                            movement_time_ms = self.move_to_angles(target_angles)
                            
                            if movement_time_ms > 0:
                                movement_count += 1
                    else:
                        # ball not visible - reset integral if we've missed many frames
                        if consecutive_missed_frames == MAX_MISSED_FRAMES:
                            self.reset_pid(last_error=False)
                            if self.verbose:
                                print(f"Ball lost for {consecutive_missed_frames} frames - resetting integral")
                                print("Ball lost - returning to center")
                            movement_time_ms = self.center_platform()
                            sleep(movement_time_ms / 1000)
                    
                    if self.verbose:
                        # print status every status_interval_s seconds
                        current_time = time()
                        if current_time - last_print_time >= status_interval_s:
                            
                            # calculate cycles per second
                            cycles_per_sec = (cycle_count - last_cycle_count) / (current_time - last_print_time)
                            
                            # format display
                            if ball_visible:
                                x_display = f"{x:4d}"
                                y_display = f"{y:4d}"
                            else:
                                x_display = " N/A"
                                y_display = " N/A"
                            
                            # convert angles back to degrees for display
                            angle_A = self.current_angles.A / 10.0
                            angle_B = self.current_angles.B / 10.0
                            angle_C = self.current_angles.C / 10.0
                            
                            if self.verbose:
                                print(f"FPS: {fps:3d} | "
                                      f"Cycles: {cycle_count:4d} ({cycles_per_sec:4.0f}/s) | "
                                      f"Movements: {movement_count:3d} | "
                                      f"Ball: ({x_display}, {y_display}) | "
                                      f"Integral: ({self.integral_x/10:.1f}, {self.integral_y/10:.1f}) | "
                                      f"Angles: A={angle_A:5.1f}° B={angle_B:5.1f}° C={angle_C:5.1f}°")
                        
                        # reset counters
                        last_cycle_count = cycle_count
                        movement_count = 0
                        last_print_time = current_time
                    
                else:
                    # small sleep
                    sleep(0.005)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Auto-balance error: {e}")
                import traceback
                traceback.print_exc()
                sleep(0.1)
        
        if self.verbose:
            print("\nAuto-balance mode stopped")
        
        self.auto_balance = False  
    
    
    
    
    def go_home(self, speed_freq=None, sg_k_factor=None, verbose=False):
        """
        Perform sensorless homing of stepper motor.
        
        Args:
            speed_freq:  Homing frequency in Hz (uses self.HOMING_SPEED_FREQ if None)
            sg_k_factor: Sensitivity factor (uses self.HOMING_SG_K_FACTOR if None)
            verbose:     If True, print detailed output
        Returns:
            bool: True if homing successful, False otherwise
        """
        
        if self.at_home:
            if self.verbose:
                print("Already at HOME position")
            return True
        
        if not self.mc.motors_enabled:
            if self.verbose:
                print("Motors must be enabled before moving to HOME")
            return False
        
        # use instance variables if parameters not provided
        if speed_freq is None:
            speed_freq = self.HOMING_SPEED_FREQ
        
        if sg_k_factor is None:
            sg_k_factor = self.HOMING_SG_K_FACTOR
        
        if self.verbose:
            print(f"Moving to HOME position (Settings: frequency={speed_freq}Hz, sg_k_factor={sg_k_factor})...")
        
        self.at_home = False
        self.at_balance = False
        
        # step 1: set homing frequency (special command 5)
        for motor in self.mc.motors:
            self.mc.send_command(motor, 5, speed_freq)
        sleep(0.05)
        
        # use the shifts loaded from settings
        sg_k_factor_shift_A = self.HOMING_SG_SHIFTS.get('A', 0)
        sg_k_factor_shift_B = self.HOMING_SG_SHIFTS.get('B', 0)
        sg_k_factor_shift_C = self.HOMING_SG_SHIFTS.get('C', 0)
        
        # homing is repeated twice: first time brings platform at same distance from home for the 3 steppers
        for i in range(2):
            
            # step 2: start homing procedure (special command 6)
            for motor in self.mc.motors:
                if motor == 'A':
                    sg_k_factor_motor = sg_k_factor + sg_k_factor_shift_A
                elif motor == 'B':
                    sg_k_factor_motor = sg_k_factor + sg_k_factor_shift_B
                elif motor == 'C':
                    sg_k_factor_motor = sg_k_factor + sg_k_factor_shift_C
                self.mc.send_command(motor, 6, sg_k_factor_motor)
            sleep(0.05)
            
            # step 3: enable motor
            self.mc.enable_motors()
            sleep(0.05)
            
            # step 4: start motors (sync pin triggers the sequence)
            self.mc.sync_motors('ABC')
            
            # step 5: monitor retraction phase (HOMING + RETRACT bits)
            for motor in self.mc.motors:
                success, status = self.mc.wait_for_status(
                    motor,
                    expected_bits=(1 << self.mc.STATUS_BIT_HOMING) | (1 << self.mc.STATUS_BIT_RETRACT),
                    timeout_s=3,
                    interval_s=0.05
                    )
            
#             sleep(0.1)
            
            # wait for retraction to complete (RETRACT bit clears)
            for motor in self.mc.motors:
                success, status = self.mc.wait_for_status(
                    motor,
                    expected_bits=(1 << self.mc.STATUS_BIT_HOMING),  # still homing, but retract cleared
                    clear_bits=(1 << self.mc.STATUS_BIT_RETRACT),
                    timeout_s=1,
                    interval_s=0.05
                    )
            
            
#             sleep(0.05)
            
            # step 6: monitor homing search phase
            for motor in self.mc.motors:
                success, status = self.mc.wait_for_status(
                    motor,
                    expected_bits=(1 << self.mc.STATUS_BIT_HOMING),
                    clear_bits=(1 << self.mc.STATUS_BIT_RETRACT),
                    timeout_s=2,
                    interval_s=0.05
                    )
            

            # step 7: wait for homing to complete
            for motor in self.mc.motors:
                success = False
                success, final_status = self.mc.wait_for_status(
                    motor,
                    clear_bits=(1 << self.mc.STATUS_BIT_HOMING),
                    timeout_s=1.5,
                    interval_s=0.05
                    )

                
                # decode final status
                is_homed = bool(final_status & (1 << self.mc.STATUS_BIT_HOMED))
                is_stalled = bool(final_status & (1 << self.mc.STATUS_BIT_STALL))
                is_enabled = bool(final_status & (1 << self.mc.STATUS_BIT_ENABLED))
                
                if verbose:
                    print(f"\n{'='*60}")
                    print("FINAL STATUS:")
                    print(f"  Raw: 0x{final_status:02X} ({final_status:3d})")
                    print(f"  Decoded: {decode_status(final_status)}")
                    print(f"  HOMED bit: {'YES' if is_homed else 'NO'}")
                    print(f"  STALL bit: {'YES' if is_stalled else 'NO'}")
                    print(f"  ENABLED bit: {'YES' if is_enabled else 'NO'}")
                    print(f"{'='*60}")
                
        
        if is_homed:
            self.at_home = True
            if self.verbose:
                print("Platform at HOME position")
            return True
        
        elif is_stalled:
            if verbose:
                print(f"\n MOTOR {motor} HOMING FAILED - Stall detected")
                print("  Try increasing sg_k_factor (less aggressive) or reducing speed")
            return False
        
        else:
            if verbose:
                print(f"\n MOTOR {motor} HOMING FAILED - Unknown reason")
                print(f"  Final status: {decode_status(final_status)}")
            return False
    
    
    
    def go_to_balance(self):
        """Move from HOME to BALANCE position."""
        
        if not self.at_home:
            if self.verbose:
                print("Motors must be at HOME before moving to BALANCE")
            return False
        
        if not self.mc.motors_enabled:
            if self.verbose:
                print("Motors must be enabled before moving to BALANCE")
            return False
        
        if self.verbose:
            print(f"Moving to BALANCE position ({self.BALANCE_STEPS_FROM_HOME} steps)...")
        
        # calculate the parameters to send to the RP2040
        pio_delay, speed_word = self.mc.normalized_speed_to_delay(self.POSITIONING_SPEED)
        
        if self.verbose:
            print(f"  Speed: {self.POSITIONING_SPEED}, PIO delay: {pio_delay}, Speed word: {speed_word}")
            
        # send to all motors the request to move to balance position
        for motor in self.mc.motors:
            if motor == 'A':
                steps = self.BALANCE_STEPS_FROM_HOME + self.BA_SHIFT_A
            elif motor == 'B':
                steps = self.BALANCE_STEPS_FROM_HOME + self.BA_SHIFT_B
            elif motor == 'C':
                steps = self.BALANCE_STEPS_FROM_HOME + self.BA_SHIFT_C
                
            success = self.mc.send_command(motor, speed_word, steps)
            if not success and self.verbose:
                print(f"  Failed to send to motor {motor}")
        
        # sync motors
        self.mc.sync_motors('ABC')
        
        # calculate and wait for movement
        move_time_ms = self.mc.calculate_movement_time(pio_delay, steps)
        move_time_s = move_time_ms / 1000.0
        if self.verbose:
            print(f"  Move time: {move_time_s:.2f}s")
        
        if move_time_s > 0:
            sleep(move_time_s + 0.1)

        # zero counters (special command 1)
        for motor in self.mc.motors:
            self.mc.send_command(motor, 1, 1)
            sleep(0.1)
        
        # initialize the motors arm1 as horizontal (90 degrees)
        self.set_angle(self.BALANCE_ANGLE, self.BALANCE_ANGLE, self.BALANCE_ANGLE)

        # after reaching balance:
        self.movement_in_progress = False
        self.at_balance = True
        self.at_home = False
        
        if self.verbose:
            print("Platform at BALANCE position (ready for operation)")
        
        return True
    
    
    
    def home_and_balance(self):
        """Complete sequence: home then go to balance position."""
        if self.go_home():
            return self.go_to_balance()
        return False
    
    
    def is_at_home(self):
        return self.at_home
    
    
    def is_at_balance(self):
        return self.at_balance
    
    
    
    def center_platform(self):
        """Return platform to center (90°)."""
        if self.verbose:
            print("Centering platform...")
        
        center_angles = MotorAngles(A = self.quantize_angle(self.BALANCE_ANGLE + self.BA_SHIFT_A),
                                    B = self.quantize_angle(self.BALANCE_ANGLE + self.BA_SHIFT_B),
                                    C = self.quantize_angle(self.BALANCE_ANGLE + self.BA_SHIFT_C))
        
        movement_time_ms = self.move_to_angles(center_angles)
        
        # reset PID accumulators
        self.reset_pid()
        
        if self.verbose:
            print(f"Platform centered in {movement_time_ms/1000:.2f}s")
        return movement_time_ms
    
    
    def is_leveled(self, tolerance=1.0):
        """Check if platform is level (within tolerance degrees)."""
        # check if all motors are within tolerance of 90°
        return (abs(self.current_angles.A - (self.BALANCE_ANGLE + self.BA_SHIFT_A)) <= tolerance * 10 and
                abs(self.current_angles.B - (self.BALANCE_ANGLE + self.BA_SHIFT_B)) <= tolerance * 10 and
                abs(self.current_angles.C - (self.BALANCE_ANGLE + self.BA_SHIFT_C)) <= tolerance * 10)
    
    
    def save_balance_shifts(self):
        """Save current balance shifts to centralized settings."""
        if self.settings_mgr:
            self.settings_mgr.update_balance_shifts(
                self.BA_SHIFT_A, self.BA_SHIFT_B, self.BA_SHIFT_C
            )
            if self.verbose:
                print(f"Balance shifts saved: A={self.BA_SHIFT_A}, B={self.BA_SHIFT_B}, C={self.BA_SHIFT_C}")
    
    
    
    def save_homing_settings(self):
        """Save StallGuard homing settings to centralized settings."""
        if self.settings_mgr:
            self.settings_mgr.update_homing_sg_factor(self.HOMING_SG_K_FACTOR)
            self.settings_mgr.update_homing_sg_shifts(self.HOMING_SG_SHIFTS)
            if self.verbose:
                print(f"Homing settings saved: SG_FACTOR={self.HOMING_SG_K_FACTOR}, "
                      f"SHIFTS={self.HOMING_SG_SHIFTS}")
    
    
    
    def save_pid_settings(self):
        """Save current PID gains to centralized settings."""
        if self.settings_mgr:
            self.settings_mgr.update_pid(self.k_p, self.k_i, self.k_d)
            if self.verbose:
                print(f"PID settings saved: kp={self.k_p}, ki={self.k_i}, kd={self.k_d}")
    
    
    
    def save_deadzone(self):
        """Save current deadzone to centralized settings."""
        if self.settings_mgr:
            self.settings_mgr.update_deadzone(self.DEADZONE_TENTHS)
            if self.verbose:
                print(f"Deadzone saved: {self.DEADZONE_TENTHS/10:.1f}mm")
    
    
    
    def save_all_controller_settings(self):
        """Save all controller settings to centralized settings."""
        self.save_balance_shifts()
        self.save_pid_settings()
        self.save_deadzone()
        if self.verbose:
            print("All controller settings saved")



# ============================================================================
# SYSTEM INITIALIZATION
# ============================================================================

class BallBalancingSystem:
    """Complete system with vision integration."""
    
    
    def __init__(self, gui_mode=False, verbose=False, auto_calibrate=True,
                 display_manager=None, settings_mgr=None):
        
        """
        Initialize the complete ball balancing system.
        
        Args:
            gui_mode:        If True, display_manager creates its own display windows for calibration
            verbose:         If True, print debug information
            auto_calibrate:  If True, run auto-calibration at startup
            display_manager: Optional DisplayManager instance for window handling
            settings_mgr:    Optional SettingsManager instance (created if None)
        """
        
        print(f"\nmbb_robot.py  version: {__version__}\n")
        
        # verbose mode
        self.verbose = verbose
            
        # initialize settings manager if not provided
        if settings_mgr is None:
            self.settings_mgr = SettingsManager(verbose=verbose)
        else:
            self.settings_mgr = settings_mgr
        
        
        settings = self.settings_mgr.get()
        
        if self.verbose:
            print("\n" + "="*60)
            print("BALL BALANCING SYSTEM WITH VISION INITIALIZATION")
            print("="*60)
            
            if settings is not None:
                print(f"Settings loaded from {SettingsManager.USER_SETTINGS_FILE}")
                print(f"Platform: {settings.hardware.platform_diameter_mm}mm")
                print(f"Ball: {settings.hardware.ball_diameter_mm}mm")
        
        # initialize motor control
        if self.verbose:
            print("\nInitializing motor control...")
        self.mc = MotorController(verbose=verbose, settings_mgr=self.settings_mgr)
        
        # initialize the display manager, before the camera.
        self.display_manager = display_manager
        
        # initialize camera
        if self.verbose:
            print("\nInitializing camera...")
        self.camera = mbb_camera.Camera(gui_mode=gui_mode,
                                          verbose=verbose,
                                          display_manager=display_manager)
        
        # check the camera
        if self.camera is None:
            if self.verbose:
                print("\nWarning: Camera is not available. Code exit")
            sys.exit(1)
        
        # case the ball HSV auto calibration is requested
        if auto_calibrate:
            ret, lower, upper = self.camera.auto_calibrate_with_windows()
        
        # initialize Cooling fans (must be done before initializing the BalanceController)
        if settings.fans.enabled:
            if self.verbose:
                print("\nInitializing cooling fans...")
            self.fans = CoolingFans(verbose=verbose, settings_mgr=settings_mgr)
            self.fans_thread_running = True      # start independent fan control thread
            self.start_fans_control()
        else:
            if self.verbose:
                print("\nCooling fans disabled in settings")
            self.fans = None
        
        
        # initialize balance controller with camera
        self.controller = BalanceController(self.fans, self.mc, self.camera, verbose=self.verbose, settings_mgr=self.settings_mgr)
        
        # initialize I2C
        self._initialize_i2c()
        
        # initialize the stepper current
        self._initialize_motor_current()
        
        # initialize platform
        self._initialize_platform()

        # auto-balance thread control
        self.auto_balance_thread = None
        self.stop_event = threading.Event()
        
        # initialize path executor and manager
        self.path_executor = PathExecutor(self, verbose=self.verbose)
        self.path_manager = PathManager(self)
        
        self.last_cup_temp_check = time()
        
        if self.verbose:
            print("\nSystem ready")
            print("="*60)
    
    
    
    def _initialize_i2c(self):
        """Check if the I2C communication works."""
        
        if self.verbose:
            print("\nInitializing I2C...")
        
        ret = self.mc.i2c_test()
        
        if ret:
            if self.verbose:
                print("I2C initialized\n")
        else:
            print("\nWarning: Not all RP2040 boards responded, code is stopped")
            sys.exit(1)
    
    
    
    def _initialize_motor_current(self):
        """
        Set stepper current from settings:
        - IRUN: current when moving (0-31 -> 0-100% of Imax)
        - IHOLD: current when stopped (0-31 -> 0-100% of Imax)
        - TPOWERDOWN: delay from last step to current reduction (2-255 -> ~0.5-5.6s)
        """
        if self.verbose:
            print("\nInitializing stepper current...")
        
        # get current settings from settings manager
        current_settings = self.settings_mgr.get_stepper_current()
        irun = current_settings["irun"]
        ihold = current_settings["ihold"]
        tpowerdown = current_settings["tpowerdown"]
        
        # pack IRUN and IHOLD into a single 16-bit value
        packed_irun_ihold = (irun << 8) | ihold
        
        if self.verbose:
            print(f"  Stepper current: IRUN={irun} ({int(100*irun/31)}%), IHOLD={ihold} ({int(100*ihold/31)}%)")
            print(f"  TPOWERDOWN={tpowerdown} (delay ~{(tpowerdown+1)*0.022:.2f}s)")
        
        for motor in self.mc.motors:
            # command 10: set IRUN and IHOLD
            self.mc.send_command(motor, 10, packed_irun_ihold)
            sleep(0.1)
            
            # command 11: set TPOWERDOWN
            self.mc.send_command(motor, 11, tpowerdown)
            sleep(0.1)
    
    
    
    def _initialize_platform(self):
        """Initialize the platform position to balance position."""
        
        self.mc.disable_motors()
        sleep(0.1)
        
        # send stop commands to all motors
        for motor in self.mc.motors:
            self.mc.send_command(motor, self.mc.STOP_WORD, 0)
            sleep(0.1)
        
        self.mc.enable_motors()
        sleep(0.1)
        
        # platform searches home to have a clear mechanical position
        ret = self.controller.go_home()
        
        # platforms goes to balanceing position, a fix "distance" from home
        ret = self.controller.go_to_balance()
    
    
    
    
    def square_path(self, side_mm: int, repeats=1, 
                    tolerance_mm: int = 40, settle_time_ms: int = 400):
        """Execute square path."""
        self.path_manager.start_path(
            self.path_executor.square_path,
            side_mm, repeats, tolerance_mm, settle_time_ms
        )
    
    
    def circle_path(self, radius_mm: int, repeats=1, direction='cw', 
                    speed_factor: float = 1.5):
        """
        Execute a circular path.
        
        Args:
            radius_mm: Radius of circle in mm
            repeats: Number of times to repeat
            direction: 'cw' or 'ccw'
            speed_factor: Speed multiplier (0.5 = half speed, 2.0 = double speed)
        """
        if self.path_executor:
            self.path_executor.circle_path(radius_mm, repeats, direction, speed_factor)
        else:
            print("Error: Path executor not initialized")
    
    
    def infinity_path(self, size_mm: int, speed_factor: float = 1.0, 
                      repeats: int = 1, stretch: float = 1.5):
        """Execute infinity path."""
        self.path_manager.start_path(
            self.path_executor.infinity_path,
            size_mm, speed_factor, repeats, stretch
        )
    
    def triangle_path(self, side_mm: int, orientation: str = 'point_up', repeats: int = 1,
                      tolerance_mm: int = 40, settle_time_ms: int = 400):
        """Execute triangle path."""
        self.path_manager.start_path(
            self.path_executor.triangle_path,
            side_mm, orientation, repeats, tolerance_mm, settle_time_ms
        )
    
    def line_path(self, length_mm: int, angle_deg: int, repeats: int = 1,
                  tolerance_mm: int = 40, settle_time_ms: int = 400):
        """Execute line path."""
        self.path_manager.start_path(
            self.path_executor.line_path,
            length_mm, angle_deg, repeats, tolerance_mm, settle_time_ms
        )
    
    
    def free_path_continuous(self):
        """Start continuous free path mode (follows finger)."""
        def run(stop_event=None):  # add stop_event parameter
            self.path_executor.free_path_continuous(
                stop_event=self.path_manager.stop_event
            )
        self.path_manager.start_path(run)

    
    def free_path_start_recording(self):
        """Start recording a free path."""
        # clear any existing recorded points
        self._recorded_points = []
        self._recording_stop_event = threading.Event()
        
        def run(stop_event=None):  # add stop_event parameter
            points = self.path_executor.free_path_record(
                stop_event=self._recording_stop_event
            )
            self._on_recording_complete(points)
        
        self.path_manager.start_path(run)

    
    def free_path_stop_recording(self):
        """Stop recording and return points."""
        if hasattr(self, '_recording_stop_event'):
            self._recording_stop_event.set()
            # wait a moment for recording to finish
            sleep(0.2)

    
    def _on_recording_complete(self, points):
        """Called when recording completes."""
        self._recorded_points = points
        if self.verbose:
            print(f"Recording complete: {len(points)} points")

    
    def get_recorded_points(self):
        """Get the last recorded points."""
        return getattr(self, '_recorded_points', [])

    
    def free_path_playback(self, points, completion_callback=None):
        def run(stop_event=None):
            # clear the stop_event before starting playback
            if hasattr(self, 'path_manager'):
                self.path_manager.stop_event.clear()
            self.path_executor.free_path_playback(
                points=points,
                stop_event=self.path_manager.stop_event,
                completion_callback=completion_callback
            )
        self.path_manager.start_path(run)
    
    
    
    
    def stop_current_path(self):
        """Stop the currently executing path."""
        self.path_manager.stop_current_path()
    


    def start_auto_balance(self):
        """Start auto-balancing mode."""
        
        # case the platform is at HOME the auto-balance is refused
        if self.controller.at_home:
            if self.verbose:
                print("Cannot start auto-balance: Platform is at HOME position")
            return False
        
        # case the platform is not at balance position the auto-balance is refused
        if not self.controller.at_balance:
            if self.verbose:
                print("Cannot start auto-balance: Platform not at BALANCE position")
            return False
        
        if self.auto_balance_thread and self.auto_balance_thread.is_alive():
            return
        
        self.stop_event.clear()
        self.controller.auto_balance = True
        
        self.auto_balance_thread = threading.Thread(
            target=self.controller.auto_balance_loop,
            args=(self.stop_event,),
            daemon=True
        )
        self.auto_balance_thread.start()
    
    
    def stop_auto_balance(self):
        """Stop auto-balancing mode."""
        self.controller.auto_balance = False
        self.stop_event.set()
        if self.auto_balance_thread:
            self.auto_balance_thread.join(timeout=2)
        if self.verbose:
            print("Auto-balance mode stopped")
    
    
    def auto_start(self):
        """on with pause-on-command approach."""
        if self.verbose:
            print("\n" + "="*60)
            print("STARTED AUTO-BALANCE MODE")
            print("="*60)
            print("Starting auto-balance mode...")
        
        self.start_auto_balance()
        
        if self.verbose:
            print("Auto-balance running in background.")
            print("Use the main thread for display and commands.")
            print("="*60 + "\n")
            print()
        
    
    
    
    def set_target(self, x_mm, y_mm):
        """Set target position in mm."""
        x_tenths = x_mm * 10
        y_tenths = y_mm * 10
        
        self.controller.target_x = x_tenths
        self.controller.target_y = y_tenths
        self.controller.integral_x = 0
        self.controller.integral_y = 0
        
        # update camera visualization
        self.camera.set_target_position(x_mm, y_mm)
        
        if self.verbose:
            print(f"Target set to ({x_mm}, {y_mm}) mm")

    
    
    def reset_target(self):
        """Reset target to center."""
        self.set_target(0, 0)

    
    
    def get_target(self):
        """Get current target position in mm."""
        return (self.controller.target_x // 10, self.controller.target_y // 10)
    
    
    
    
    def home_platform(self):
        """Home the platform (move to end stops)."""
        return self.controller.go_home()
    
    
    
    
    def balance_platform(self):
        """Move platform to balance position (level)."""
        
        # HERE
        print(f"DEBUG: balance_platform called, at_balance={self.controller.at_balance}")
        
        # if already in progress, just return
        if hasattr(self, '_balancing_in_progress') and self._balancing_in_progress:
            if self.verbose:
                print("Balance already in progress, ignoring")
            return False
        
        # if already balanced, just return
        if self.controller.at_balance:
            if self.verbose:
                print("Already at balance position")
            return True
        
        # if not at home, go home first
        if not self.controller.at_home:
            if self.verbose:
                print("Platform not at home, homing first...")
            if not self.home_platform():
                return False
        
        # now go to balance position
        if self.verbose:
            print("Moving to balance position...")
        success = self.controller.go_to_balance()
        
        if success:
            # update states
            self.controller.at_balance = True
            self.controller.at_home = False
        
        return success
    
    
    
    
    def save_balance_shifts(self):
        """Save balance shifts to settings."""
        self.controller.save_balance_shifts()
    
    
    
    def save_homing_settings(self):
        """Save homing settings to settings."""
        self.controller.save_homing_settings()
    
    
    
    def save_pid_settings(self):
        """Save PID settings to settings."""
        self.controller.save_pid_settings()
    
    
    
    def save_deadzone(self):
        """Save deadzone to settings."""
        self.controller.save_deadzone()
    
    
    
    
    def save_all_controller_settings(self):
        """Save all controller settings to settings."""
        self.controller.save_all_controller_settings()
    
    
    
    
    def _fans_control_loop(self):
        """Background thread that periodically checks and controls fans."""
        while self.fans_thread_running and self.fans is not None:
            self.fans.fans_control()
            sleep(10)  # check every 10 seconds (faster than the 15s internal check)
    
    
    
    
    def start_fans_control(self):
        """Start the background fan control thread."""
        if hasattr(self, 'fans_thread') and self.fans_thread and self.fans_thread.is_alive():
            return
        
        self.fans_thread_running = True
        self.fans_thread = threading.Thread(target=self._fans_control_loop, daemon=True)
        self.fans_thread.start()
        if self.verbose:
            print("\nBackground fan control thread started\n")
    
    
    
    
    def stop_fans_control(self):
        """Stop the background fan control thread."""
        if self.fans is None:
            return
        self.fans_thread_running = False
        if hasattr(self, 'fans_thread') and self.fans_thread:
            self.fans_thread.join(timeout=2)
        if self.verbose:
            print("Background fan control thread stopped")
    
    
    
    
    def cleanup(self):
        """Clean shutdown."""
        print("\n" + "="*60)
        print("SYSTEM SHUTDOWN")
        print("="*60)
        
        # signal shutdown first
        self.is_shutting_down = True
        
        # stop auto-balance
        if self.controller.auto_balance:
            self.stop_auto_balance()
        
        # center platform
        try:
            if self.verbose:
                print("Centering platform...")
            self.controller.center_platform()
            sleep(0.5)
        except:
            pass
        
        # disable motors
        try:
            self.mc.disable_motors()
            if self.verbose:
                print("Motors disabled")
        except:
            pass
        
        # stop fan control thread
        try:
            if self.fans is not None:
                self.stop_fans_control()
        except:
            pass
        
        # stop camera
        if self.camera:
            self.camera.clean_up_cam()
        
        # close I2C
        try:
            self.mc.bus.close()
            if self.verbose:
                print("I2C closed")
        except:
            pass
        
        print("Cleanup complete")
        print("="*60)





class PathManager:
    """Simple path manager to handle path interruptions."""
    
    def __init__(self, robot):
        self.robot = robot
        self.current_path_thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
    
    
    def start_path(self, path_func, *args, **kwargs):
        """Start a path, stopping any current one first."""
        with self.lock:
            self._stop_current()
            self.stop_event.clear()
            
            self.current_path_thread = threading.Thread(
                target=self._run_path,
                args=(path_func, *args),
                kwargs=kwargs,
                daemon=True
            )
            self.current_path_thread.start()
    
    
    def _stop_current(self):
        """Stop the currently running path."""
        if self.current_path_thread and self.current_path_thread.is_alive():
            self.stop_event.set()
            self.current_path_thread.join(timeout=2.0)
    
    
    def _run_path(self, path_func, *args, **kwargs):
        """Run path with stop event."""
        path_func(*args, stop_event=self.stop_event, **kwargs)
    
    
    def stop_current_path(self):
        """Public method to stop the current path."""
        with self.lock:
            self._stop_current()





# ============================================================================
# MAIN EXECUTION to test it as standalone (without the GUI interface)
# ============================================================================
    
def main(verbose = False):
    """Main program entry point for standalone robot operation."""

    # import cv2 and os only when testing without the GUI
    import cv2, os
    
    # create a display manager instance
    display_mgr = DisplayManager(verbose=verbose)
    
    # create a setting manager instance
    settings_mgr = SettingsManager()
    
    # initialize the system
    print("\n" + "="*60)
    print("BALL BALANCING ROBOT - STANDALONE MODE")
    print("="*60)
    
    # auto-calibration runs at startup by default
    system = BallBalancingSystem(gui_mode=False,
                                 verbose=verbose, 
                                 auto_calibrate=True,
                                 display_manager=display_mgr,
                                 settings_mgr=settings_mgr
                                 )
    
    print("Starting auto-balance...")
    system.auto_start()
    
    # check if auto_balance is True
    print(f"auto_balance = {system.controller.auto_balance}")
    
    
    stderr_fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)

    try:
        # settings preventing garbage prints when setting the first CV2 window
        os.dup2(devnull, 2)
        
        # create main display window
        window_name = "Ball Balance Platform"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 352, 352)
        cv2.moveWindow(window_name, 0, 30)
        cv2.waitKey(1)

    finally:
        # reset settings preventing garbage prints when setting the first CV2 window
        os.dup2(stderr_fd, 2)
        os.close(devnull)
        os.close(stderr_fd)
    
    
    print("\n" + "="*60)
    print("CONTROLS:")
    print("="*60)
    print("  't' - Set target position (then enter X Y in mm)")
    print("  's' - Show current target position")
    print("  'r' - Reset target to center")
    print("  'a' - Auto calibration (ball color)")
    print("  'm' - Manual calibration (ball color)")
    print("  'q' - Quit")
    print()
    print("NOTE 1: Select the video window prior entering a choice")
    print("NOTE 2: Select the Terminal to input values and/or Enter\n\n")
    print("="*60)
    print("  Robot is balancing in background")
    print("="*60 + "\n")
    
    # variable for target input state
    waiting_for_target = False
    
    try:
        while system.controller.auto_balance:
            # get and display frame
            frame = system.camera.get_annotated_frame()
            if frame is not None:
                cv2.imshow(window_name, frame)
            
            # handle keyboard input
            key = cv2.waitKey(33) & 0xFF
            
            if waiting_for_target:
                # collect target coordinates from console (non-blocking)
                import sys
                import select
                
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip()
                    if line:
                        if line.lower() == 'q':
                            waiting_for_target = False
                            print("Target input cancelled")
                        else:
                            parts = line.split()
                            if len(parts) == 2:
                                try:
                                    x_mm = int(parts[0])
                                    y_mm = int(parts[1])
                                    system.set_target(x_mm, y_mm)
                                    waiting_for_target = False
                                except ValueError:
                                    print("Invalid input. Enter two integers (X Y in mm)")
                            else:
                                print("Enter two numbers: X Y (in mm)")
                    else:
                        # no input, show prompt again
                        sys.stdout.write("Target (X Y mm): ")
                        sys.stdout.flush()
                continue
            
            # normal command handling
            if key == ord('q'):
                print("\nQuitting...")
                break
            
            elif key == ord('t'):
                waiting_for_target = True
                print("\n--- SET TARGET ---")
                print("Enter X Y coordinates in mm (e.g., '50 -30')")
                print("Press 'q' to cancel")
                import sys
                sys.stdout.write("Target (X Y mm): ")
                sys.stdout.flush()
            
            elif key == ord('s'):
                x_mm, y_mm = system.get_target()
                print(f"Current target: ({x_mm}, {y_mm}) mm")
            
            elif key == ord('r'):
                system.reset_target()
                print("Target reset to center")
            
            elif key == ord('a'):
                print("\n--- AUTO CALIBRATION ---")
                print("Center the ball and press Enter...")
                input()
                
                # close main window temporarily
                cv2.destroyWindow(window_name)
                cv2.waitKey(100)
                
                # run auto calibration
                ret, lower, upper = system.camera.auto_calibrate_with_windows()
                
                if ret:
                    print(f"Auto calibration successful: Lower={lower}, Upper={upper}")
                else:
                    print("Auto calibration failed!")
                
                # recreate main window
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 352, 352)
                cv2.moveWindow(window_name, 0, 30)
                cv2.waitKey(100)
                print("\nResuming normal operation...")
            
            elif key == ord('m'):
                print("\n--- MANUAL CALIBRATION ---")
                
                # close main window temporarily
                cv2.destroyWindow(window_name)
                cv2.waitKey(100)
                
                # run manual calibration
                lower, upper = system.camera.manual_hsv_adjust()
                print(f"Manual calibration: Lower={lower}, Upper={upper}")
                
                # recreate main window
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 352, 352)
                cv2.moveWindow(window_name, 0, 30)
                cv2.waitKey(100)
                print("\nResuming normal operation...")
            
            # small sleep to prevent CPU overload
            sleep(0.005)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        cv2.destroyAllWindows()
        display_mgr.destroy_all()
        if system:
            system.stop_auto_balance()
            system.cleanup()
        print("\nSystem shutdown complete.\n\n") 
    

if __name__ == "__main__":
    verbose = False  # Set to True for debug, when standalone testing
    main(verbose = verbose)
    
    