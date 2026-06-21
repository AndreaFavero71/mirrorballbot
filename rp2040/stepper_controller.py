"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


MIT License
Copyright (c) 2026 Andrea Favero
"""

from time import ticks_diff, ticks_ms, ticks_us, sleep_ms,  sleep_us
from rp2 import PIO, StateMachine, asm_pio, asm_pio_encode
from shared_variables import shared_variables
from machine import Pin, reset
from TMC_2209_driver import *



class StepperController:
    
    # Bit position constants for status bitmask
    # These allow multiple states to be represented simultaneously in a single byte
    BIT_ENABLED   = 0  # 1 << 0 = 1   - Motor driver powered on
    BIT_MOVING    = 1  # 1 << 1 = 2   - Rotor currently turning
    BIT_HOMED     = 2  # 1 << 2 = 4   - At reference position
    BIT_BUSY      = 3  # 1 << 3 = 8   - Executing command (moving OR waiting for sync)
    BIT_STALL     = 4  # 1 << 4 = 16  - Stall/overcurrent detected
    BIT_ERROR     = 5  # 1 << 5 = 32  - Error state (thermal, overcurrent, etc)
    BIT_HOMING    = 6  # 1 << 6 = 64  - Homing procedure active
    BIT_RETRACT   = 7  # 1 << 7 = 128 - Retracting from home (sub-phase of homing)
    
    # Predefined status masks for common conditions (for convenience)
    MASK_READY_FOR_COMMAND   = (1 << BIT_ENABLED)  # Enabled (not busy check must be done separately)
    MASK_ACTIVE_MOVEMENT     = (1 << BIT_MOVING) | (1 << BIT_BUSY)
    MASK_HOMING_IN_PROGRESS  = (1 << BIT_HOMING) | (1 << BIT_BUSY)
    MASK_ERROR_STATES        = (1 << BIT_STALL)  | (1 << BIT_ERROR)

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init()
        return cls._instance

    
    
    def _init(self, reverse=False, ms=0, printout=True):
        # reverse: use True to reverse the stepper direction (solving wiring issue), otherwise False
        # ms: microstepping, from 0 to 3
        
        self.reverse = reverse             # flag to reverse the stepper direction (reverse==True) if wiring issues
        self.ms = ms                       # integГЁr reppresenting the microstep (0=1/8, 1=1/32, 2=1/64 and 3=1/16)
        self.printout = printout           # flag to print some data to the terminal for debug purpose  
        print("\n[core 0] Uploading stepper_controller ...")

        # constants, from the shared_variables file
        self.frequency       = shared_variables.FREQUENCY
        self.pio_fix         = shared_variables.PIO_FIX
        self.pio_var         = shared_variables.PIO_VAR
        self.stepper_steps   = shared_variables.MOTOR_STEPS
        
        # GPIO pins, from the shared_variables file
        self.motor_steps_pin = Pin(shared_variables.MOTOR_STEPS_PIN, Pin.OUT)
        self.motor_dir       = Pin(shared_variables.MOTOR_DIR_PIN, Pin.OUT)
        self.motor_ms1       = Pin(shared_variables.MOTOR_MS1_PIN, Pin.OUT)
        self.motor_ms2       = Pin(shared_variables.MOTOR_MS2_PIN, Pin.OUT)
        self.sync_pin        = Pin(shared_variables.SYNC_PIN, Pin.IN)
        self.diag_pin        = Pin(shared_variables.DIAG_PIN, Pin.IN, Pin.PULL_DOWN)
        
        # pre-encode some of the PIO instructions  
        self.PULL_ENCODED = asm_pio_encode("pull()", 0)
        self.MOV_X_OSR_ENCODED = asm_pio_encode("mov(x, osr)", 0)
        self.PUSH_ENCODED = asm_pio_encode("push()", 0)
        self.MOV_ISR_X_ENCODED = asm_pio_encode("mov(isr, x)", 0)
    
        # initial states and variables
        self.motor_enabled = False          # flag to track whether the motor is enabled
        self._stepper_busy = False          # True when motor is moving or waiting for sync
        self._last_stepper_busy = False     # previous status of _stepper_busy
        self.direction = 1 if self.reverse else 0 # motor direction (rotation) is set to 0 unless modified (reverse==True)
        self.position = 0                   # counter with the motor position (reference position is 0)
        self.estimated_pos = 0              # counter with the expected motor steps (reference position is 0)
        
        # settings for holding current
        self.current_irun = 31              # Run current (100%)
        self.current_ihold = 15             # Holding current (50%)
        
        # Status bitmask tracking (initialize as disabled)
        self.status_flags = 0               # current status as bitmask
        self._last_status = -1              # track last status for debug printing (ADD THIS LINE)
        self._homing_phase = None           # tracks homing phase: 'retract' or 'homing'
        self._homing_failed = False         # tracks if homing failed
        self.stallguarded = False           # flag tracking StallGuard detection
        
        # microstepping map --> setting: (descriptor, steps_divider, ms2, ms1, SG_adjustment, serialport_nodeaddress)
        self.microstep_map = {0: ("1/8",  0.125,    0, 0, 1, 0),
                              1: ("1/32", 0.03125,  0, 1, 4, 1),
                              2: ("1/64", 0.015625, 1, 0, 8, 2),
                              3: ("1/16", 0.0625,   1, 1, 2, 3)
                              }
        
        ms_txt, steps_divider, ms2, ms1, self.SG_adj, serial_port_node_addr = self.microstep_map.get(ms)
        
        # Microstep resolution configuration (internal pull-down resistors)
        # TMC2209 MS2,MS1: 00:1/8, 11:1/16, 01:1/32, 10:1/64
        ret = self._set_micro_step_pins(ms2, ms1, ms_txt)
        
        # stepper steps for a full revolution, based on stepper steps and microstepping
        self.full_rev = int(self.stepper_steps/steps_divider)    # steps for a stepper full revolution
        print("[core 0] self.full_rev:", self.full_rev)
        
        # instantiate the module that communicates with the UART of TMC 2209 stepper driver
        # args: pin_step, pin_dir, pin_en, rx_pin, tx_pin, mtr_id=0, baudrate=115200, serialport=0
        self.tmc = TMC_2209( rx_pin=Pin(13), tx_pin=Pin(12),
                             mtr_id=serial_port_node_addr,
                             serialport=0, baudrate=115200) #230400)
        
        # check if the TMC 2209 stepper driver reacts to the UART (suggesting if it's powered on)
        if self._is_tmc_powered():
            self.set_stallguard(threshold=0)                     # set the TMC 2209 StallGuard off (get max torque)
            self.tmc.setCoolStep_Threshold(threshold=3200)       # set the TMC 2209 CoolStep to 1600 (experimental)
        
            # CoolStep and StallGuards are active when threshold > TSTEP (timeStep)
            # TSTEP is the step period in TMC clocks units (f=2MHz) calculated for 1/256 microstep
            # threshold = 1600 ensures STALLGUARD operation for steps frequency
            #   --> f > 400Hz  at 1/8  microstep
            #   --> f > 800Hz  at 1/16 microstep
            #   --> f > 1600Hz at 1/32 microstep
            #   --> f > 3200Hz at 1/34 microstep
            
            # after 2 seconds idle, current drops to 50% of run current
            # when movement starts again, current automatically returns to 100%
            self.tmc.setStepperCurrent(irun=self.current_irun, ihold=self.current_ihold)
        
        
        # set the default steppers speed for the sensorless hominng
        homing_freq = 800                                        # default homing steps frequency in Hz
        self.homing_freq = homing_freq * self.SG_adj             # default homing speed adjusted for the microstepping
        print("[core 0] self.homing_freq:", self.homing_freq)    # feedback is printed to the terminal
        
        self.home = False                                        # home flag is initially set as False
        self.motor_enabled = False                               # stepper is disabled  (it can be enabled via I2C)
        
        # Initialize status (disabled state)
        self._update_status()                                    # update the bitmask status
        
        # state machine SM0 for motor steps generation, sync and completion IRQ
        self.sm0 = StateMachine(0,
                                self.steps_mot_sync_with_irq, 
                                freq=shared_variables.FREQUENCY, 
                                set_base=shared_variables.MOTOR_STEPS_PIN)
        self.sm0.irq(self._stepper_complete_handler)  # IRQ for completion
        self.sm0.active(0)              # state machine is kept deactivated
        
        
        # state machine SM1 for motor steps tracking (for verification only)
        self.sm1 = StateMachine(1,
                                self.steps_counter_pio,
                                freq=shared_variables.MAX_FREQUENCY,
                                in_base=shared_variables.MOTOR_STEPS_PIN)
        self.set_pls_counter(0)         # sets the initial value for motor steps counting
        self.sm1.active(1)              # starts the state machine for motor steps counting
        
        
    def _update_status(self):
        """
        Update the status bitmask based on current state.
        This method is called whenever any state-changing variable is modified.
        The status is written to shared_variables for I2C reading.
        """
        status = 0
        
        # Set bits based on current state
        if self.motor_enabled:
            status |= (1 << self.BIT_ENABLED)
        
        if self._stepper_busy:
            status |= (1 << self.BIT_BUSY)
            status |= (1 << self.BIT_MOVING)  # Busy implies moving (or waiting to move)
        
        if self.home:
            status |= (1 << self.BIT_HOMED)
        
        # Homing phase tracking
        if self._homing_phase is not None:
            status |= (1 << self.BIT_HOMING)
            if self._homing_phase == 'retract':
                status |= (1 << self.BIT_RETRACT)
        
        # Error and stall conditions
        if self.stallguarded:
            status |= (1 << self.BIT_STALL)
        
        # Note: BIT_ERROR can be set in future for other error conditions
        # if self._error_condition:
        #     status |= (1 << self.BIT_ERROR)
        
        # Store the status
        self.status_flags = status
        shared_variables.motor_status.write(status)
        
        if self.printout and self._last_status != status:
            print(f"[core 0] Status updated: 0x{status:02X} ({status:3d}) - {self._decode_status(status)}")
            self._last_status = status
        
        return status
    
    
    def _decode_status(self, status):
        """Helper method to decode status bitmask for debug printing"""
        if status == 0:
            return "DISABLED"
        
        states = []
        if status & (1 << self.BIT_ENABLED): states.append("ENABLED")
        if status & (1 << self.BIT_MOVING):  states.append("MOVING")
        if status & (1 << self.BIT_HOMED):   states.append("HOMED")
        if status & (1 << self.BIT_BUSY):    states.append("BUSY")
        if status & (1 << self.BIT_STALL):   states.append("STALL")
        if status & (1 << self.BIT_ERROR):   states.append("ERROR")
        if status & (1 << self.BIT_HOMING):  states.append("HOMING")
        if status & (1 << self.BIT_RETRACT): states.append("RETRACT")
        
        return " | ".join(states) if states else "IDLE"
    
    
    def get_status(self):
        """Return current status bitmask (for external queries)"""
        return self.status_flags
    
    def is_state(self, bit):
        """Check if a specific bit is set in the current status"""
        return bool(self.status_flags & (1 << bit))
    
    def is_ready_for_command(self):
        """Check if motor is ready to accept new commands"""
        return (self.motor_enabled and 
                not self._stepper_busy and 
                self._homing_phase is None and
                not self.stallguarded)
        
    
    @asm_pio(set_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT)
    def steps_mot_sync_with_irq():
        """
        PIO function for the steps generation.
         - Steps frequency based on delay, passed within the 32 bit packed variable.
         - Stops after predefined steps, passed within the 32 bit packed variable.
        
        Expects: 32-bit word with:
         - Bits 31:14 = 18-bit delay value, to determine the speed (steps frequency)
         - Bits 13:0  = 14-bit steps countdown (steps-1)
        
        out_shiftdir=PIO.SHIFT_RIGHT it shifts right, so:
         - First out(x, 14) gets lower 14 bits (steps)
         - Second out(y, 18) gets upper 18 bits (delay)
         
        The function has 8 fix intructions + 4 * delay instrustions
        """
        
        wait(1, gpio, 14)        # waits for synchronization GPIO pin
        
        label("new_block")
        pull(block)              # get 32-bit word
        
        out(x, 14)               # first 14 bits to X (steps)
        out(y, 18)               # next 18 bits to Y (delay)

        # High level pin
        label("step_loop")
        set(pins, 1)             # step HIGH
        mov(isr, y)              # save Y
        label("high_delay")
        nop()                    # 1 cycle
        jmp(y_dec, "high_delay") # decrement Y
        
        # Low level pin
        set(pins, 0)             # step LOW
        mov(y, isr)              # restore Y
        label("low_delay")
        nop()                    # 1 cycle
        jmp(y_dec, "low_delay")
        
        # Restore Y for next step
        mov(y, isr)
        
        jmp(x_dec, "step_loop")  # next stepper step
        
        irq(block, rel(0))       # IRQ is called after each Pull
        jmp("new_block")         # pull in next Word
    
    

    
    
    # sm1, the PIO program for counting steps (for verification only)
    @asm_pio()
    def steps_counter_pio():      # no 'self' in PIO callback function !
        """
        This PIO function counts steps for verification purposes.
        - Reads the rising edges from the output GPIO that generates the steps.
        - De-counts the 32bit OSR.
        - The starting value, to decount from, is sent by inline command.
        - The reading is pushed to the rx_fifo on request, via an inline command.
        """
        label("loop")
        wait(0, pin, 0)     # wait for step signal to go LOW
        wait(1, pin, 0)     # wait for step signal to go HIGH (rising edge)
        jmp(x_dec, "loop")  # decrement x and continue looping
    
    
    
    def reset_sm0_state(self):
        """Manually reset SM0's internal state."""
        self.sm0.active(0)
        self._clear_sm0_fifo()
        
        # Clear X and Y registers
        self.sm0.exec("mov(x, null)")  # set X to 0
        self.sm0.exec("mov(y, null)")  # set Y to 0
        
        # Clear any pending IRQ
        self.sm0.exec("irq(clear, 0)")  # clear IRQ flag

    
    def set_pls_counter(self, val):
        """Sets the initial value for the steps counter."""
        self.sm1.put(val)
        self.sm1.exec(self.PULL_ENCODED)       # execute pre-encoded 'pull()' instruction
        self.sm1.exec(self.MOV_X_OSR_ENCODED)  # execute pre-encoded 'mov(x, osr)' instruction
         

    
    def get_pls_count(self):
        """Gets the steps counter value from sm1 (for verification)."""
        self.sm1.exec(self.MOV_ISR_X_ENCODED)  # execute pre-encoded 'mov(isr, x)' instruction
        self.sm1.exec(self.PUSH_ENCODED)       # execute pre-encoded 'push()' instruction
        if self.sm1.rx_fifo():
            return -self.sm1.get() & 0xffffffff
        else:
            return -1

    
    
    def set_steps_value(self, val):
        """Set the ref position (after homing)."""
        self.position = val
    
    
    
    def reset_variables(self):
        """Reset all tracking variables to initial state."""
        self.sm0.active(0)              # stop SM0
        self._clear_sm0_fifo()          # clear any pending parameters
        
        self.sm0.active(1)              # reactivate SM0 (waits for sync)
        self.set_pls_counter(0)         # reset SM1 counter
        
        self.position = 0
        self.estimated_pos = 0
        self.is_stepper_busy(False)
        self.stallguarded = False
        self._homing_phase = None
        self._homing_failed = False
        self._update_status()           # Update status bitmask

        
        if self.printout:
            print("[core 0] Variables reset to zero")
        
        
    
    def _is_tmc_powered(self):
        """Test if TMC2209 driver responds to the UART (i.e. if powered)."""
        if self.tmc.test():
            return 1
        else:
            if self.printout:
                print("[core 0] TMC_2209 driver is not powered (or UART not connected)")
            return 0
        
        
    
    def read_stallguard(self):
        """Read StallGuard value from TMC2209."""
        return self.tmc.getStallguard_Result()
    
    
    
    def print_stallguard(self):
        """Print StallGuard value for debugging."""
        if self.printout:
            print("[core 0]", self.tmc.getStallguard_Result())
        
    
    
    def set_stallguard(self, threshold):
        """Configure TMC2209 StallGuard with given threshold."""
        # clamp the SG threshold between 0 and 255
        threshold = max(0, min(threshold, 255))
        
        # set the StallGuard threshold
        self.tmc.setStallguard_Threshold(threshold = threshold)
        
        if self.printout:
            if threshold != 0:
                print("\n[core 0] Setting StallGuard to {}".format(threshold))
            else:
                print("\n[core 0] Setting StallGuard to 0 (max possible torque)")
    
    
    def _stallguard_callback(self, pin):  
        """Callback for TMC2209 StallGuard detection."""
        self.sm0.active(0)               # state machine for motor-steps generation is deactivated
        self._clear_sm0_fifo()           # clear any old parameters
        self.stallguarded = True         # flag tracking the StallGuard at GPIO is set True
        self.is_stepper_busy(False)      # updated the stepper moving related variables
        self._update_status()            # Update status bitmask (STALL bit will be set)
        if self.printout:
            print("\n[core 0] StallGuard detections\n")
            print("[core 0] Motor StallGuard detection via the DIAG pin")
    
    
    def check_stallguard_pin(self):
        """Check if StallGuard triggered via DIAG pin (polling version)"""
        if self.diag_pin.value() == 1 and not self.stallguarded:
            self.stallguarded = True         # flag tracking the StallGuard at GPIO is set True
            self.stepper_busy = False        # flag monitoring/setting the motor state, is set False
            self.sm0.active(0)               # state machine for motor-steps generation is deactivated
            self._clear_sm0_fifo()           # clear any old parameters
            self._update_status()            # Update status bitmask
            if self.printout:
                print("[core 0] Motor StallGuard detection via DIAG pin (polled)")
            return True
        return False
    
    
    def get_stepper_frequency(self, pio_val):
        """Convert the PIO value (delay) to stepper frequency."""
        if pio_val > 0:
            return int(self.frequency / (pio_val * self.pio_var + self.pio_fix))
        else:
            return None


    def get_stepper_value(self, stepper_freq):
        """Convert the stepper frequency to PIO value (delay)."""
        if stepper_freq > 0:
            return int((self.frequency - stepper_freq * self.pio_fix) / (stepper_freq * self.pio_var))
        else:
            return None
    
    
    
    def _set_micro_step_pins(self, ms2, ms1, ms_txt):
        """Set microstep resolution on TMC2209 driver."""
        # TMC2209 MS2,MS1: 00:1/8, 11:1/16, 01:1/32, 10:1/64
        # ms ==> 0=1/8, 1=1/32, 2=1/64 and 3=1/16
        try:
            self.motor_ms2.value(ms2)      # motor_ms2 output set to ms2 value
            self.motor_ms1.value(ms1)      # motor_ms1 output set to ms1 value
            if self.printout:
                print(f"[core 0] Microstep to {ms_txt}")  # feedback is printed to the terminal
            return True                    # return True
        except:                            # case the ms setting does not exist
            if self.printout:
                print("[core 0] Wrong parameter for micro_step") # feedback is printed to the terminal
            return False

    
    
    def _update_ms_settings(self, ms):
        """
        Helper function to update some motor parameters based on the microstepping.
        This is useful when the microstepping setting is received via the i2C.
        Set microstep resolution on TMC2209 driver, and the steppers steps for a full revolutuon.
        """
        # TMC2209 MS2,MS1: 00:1/8, 11:1/16, 01:1/32, 10:1/64
        # ms ==> 0=1/8, 1=1/32, 2=1/64 and 3=1/16
        if int(ms) != self.ms:
            self.ms = int(ms)
            ms_txt, steps_divider, ms2, ms1, self.SG_adj, serial_port_node_addr = self.microstep_map.get(int(ms))
            ret = self._set_micro_step_pins( ms2, ms1, ms_txt)
            
            self.tmc = TMC_2209( rx_pin=Pin(13), tx_pin=Pin(12),
                                 mtr_id=int(serial_port_node_addr),
                                 serialport=0, baudrate=115200)
            
            # stepper steps for a full revolution, based on stepper steps and microstepping
            self.full_rev = int(self.stepper_steps/steps_divider)    # steps for a stepper full revolution
            
            if self.printout:
                print("[core 0] self.full_rev:", self.full_rev)
        
        self._update_status()  # Update status after microstep change
        
    
    def stop_stepper(self):
        """Stop the motor immediately."""
        self.sm0.active(0)               # state machine for motor-steps generation is deactivated
        self.is_stepper_busy(False)      # updated the stepper moving related variables
        self._update_status()            # Update status bitmask
    
    
    
    def start_stepper(self):
        """LEGACY - Start motor (activates SM0 which waits for sync)."""
        self.sm0.active(1)               # state machine for motor-steps generation is activated
        self._update_status()            # Update status bitmask (BUSY bit will be set when moving)
    
    
    
    def disable_stepper(self):
        """Disable motor (prevent new commands from being processed)."""
        self.motor_enabled = False       # variable used to process new motor data or not
        self.is_stepper_busy(False)      # updated the stepper moving related variables
        self._update_status()            # Update status bitmask (ENABLED bit will be cleared)


    
    def enable_stepper(self):
        """Enable motor (allow new commands to be processed)."""
        self.motor_enabled = True        # variable used to process new motor data or not
        self._update_status()            # Update status bitmask (ENABLED bit will be set)

    
    
    def _some_prints(self):
        """Debug helper to print position tracking variables."""
        if self.printout:
            print(f"\n[core 0] self.estimated_pos: {self.estimated_pos}")
            print(f"[core 0] self.position: {self.position}\n")
    
    
            
    def _stepper_complete_handler(self, sm):
        """
        Called by PIO IRQ when motor completes all steps.
        This happens after the LAST pulse is generated.
        """
        # Update actual position based on what was actually requested
        pulses = self.get_pls_count()
        
        if self.direction != self.reverse:
            self.position += pulses
        else:
            self.position -= pulses
                
        # reset the _stepper_busy state
        self.is_stepper_busy(False)           # updated the stepper moving related variables
        self._update_status()                 # Update status bitmask (BUSY/MOVING bits will be cleared)
        
        if self.printout:
            print(f"[core 0] Current position (steps): {pulses}")
    
    
    
    def pack_parameters(self, delay, steps):
        """
        Pack delay and steps into 32-bit word: 18-bit delay, 14-bit steps.
        delay: 0-262,143 (18 bits) - larger speed range (for low speeds)
        steps: 1-16,384 (14 bits) - enough for 1 rev at microstepping 16
        """
        if steps < 1:
            steps = 1
        if steps > 16384:
            steps = 16384
        
        steps_countdown = steps - 1                 # 0-16383  (14 bits)
        delay_clamped = min(max(delay, 0), 262143)  # 0-262143 (18 bits)
        
        # Pack: delay in upper 18 bits, steps in lower 14 bits
        packed = (delay_clamped << 14) | (steps_countdown & 0x3FFF)
        return packed
    
    
    
    def unpack_parameters(self, packed):
        """Unpack 32-bit word for debugging/verification."""
        steps_countdown = packed & 0x3FFF    # Lower 14 bits
        steps = steps_countdown + 1
        delay = (packed >> 14) & 0x3FFFF     # Upper 18 bits (mask 18 bits)
        return delay, steps
    
    
    
    def _clear_sm0_fifo(self):
        """Clear SM0's FIFO to ensure clean state."""
        while self.sm0.rx_fifo():
            _ = self.sm0.get()
    
    
    
    def _set_stepper_current(self, irun=None, ihold=None, iholddelay=5):
        """
        Set holding current parameters.
        Only parameters that are not None will be updated.
        
        irun: 0-31 (0=3%, 31=100%) - Current when motor is moving.
        ihold: 0-31 (0=3%, 15=50%, 31=100%) - Current when motor is stopped.
        iholddelay: 0-15 - Additional delay before reduction, in units of ~0.022 seconds.
        """
        # update only provided parameters
        if irun is not None:
            self.current_irun = max(0, min(irun, 31))
        if ihold is not None:
            self.current_ihold = max(0, min(ihold, 31))
        
        # Calculate percentage for display
        irun_percent = int(100 * self.current_irun / 31)
        ihold_percent = int(100 * self.current_ihold / 31)
        
        if self.printout:
            print(f"[core 0] Current settings: IRUN={self.current_irun} ({irun_percent}%), IHOLD={self.current_ihold} ({ihold_percent}%)")
        
        self.tmc.setStepperCurrent(irun=self.current_irun, 
                                   ihold=self.current_ihold, 
                                   iholddelay=iholddelay)
    
    
    
    def _set_power_down_delay(self, delay):
        """
        Set auto current reduction delay (TPOWERDOWN register).
        
        delay: 2-255 (minimum 2 required for StealthChop auto tuning)
        Time range: about 0 to 5.6 seconds
        Actual time = delay * 2^18 / 12MHz ≈ delay * 0.022 seconds
        
        Typical values:
        delay=2   → about 0.044s (minimum for StealthChop)
        delay=20  → about 0.44s (default)
        delay=45  → about 1.0s
        delay=90  → about 2.0s
        delay=136 → about 3.0s
        delay=255 → about 5.6s (maximum)
        """
        # clamp to valid range: 2-255 (minimum 2 for StealthChop)
        delay = max(2, min(delay, 255))
        
        if self.printout:
            delay_seconds = delay * 0.022  # approximate
            print(f"[core 0] TPOWERDOWN = {delay} (current reduction after ~{delay_seconds:.2f}s)")
        
        self.tmc.setPowerDownDelay(delay)
    
    
    
    def is_stepper_busy(self, status=None):
        """
        Check or set the motor 'busy' status:
        When checking, it returns
        - True: Either waiting for sync OR actively moving
        - False: Idle (not programmed with steps)
        """
        if status is None:
            return self._stepper_busy
        
        else:
            self._stepper_busy = status
            self._update_status()  # Update status bitmask whenever busy state changes
            
            # Also update shared_variables.motor_status for backward compatibility
            # but now it contains the bitmask instead of simple codes
            if self._stepper_busy:
                pass  # Status already updated by _update_status
            elif not self._stepper_busy:
                if self.home:
                    pass  # Status already updated by _update_status
                elif not self.motor_enabled:
                    pass  # Status already updated by _update_status
    
    
    
    def _drive_stepper(self, direction, delay, steps):
        """
        Sends parameters to the steps generators (PIO SMO).
        Returns immediately (about 230 us).
        (the long if/elif/else method looks weird, yet faster than other methods tested)
        """
        
        if steps <= 0:
            if self.printout:
                print("[core 0] No steps requested")
            return False
        
        if self._stepper_busy:
            if self.printout:
                print("[core 0] Motor currently moving, cannot prepare new move")
            return False
        
        # update motor state
        self.is_stepper_busy(True)           # updated the stepper moving related variables
        
        # zeroing the pulse counter
        self.set_pls_counter(0)
        
        # set direction
        if direction != self.reverse:
            self.motor_dir.value(1)
            self.direction = 1
        else:
            self.motor_dir.value(0)
            self.direction = 0
            
        
        # clear any old parameters
        self.sm0.active(0)
        self._clear_sm0_fifo()
        
        # determine ramp-up (and ramp-down) steps
        steps_ramp = 0
        
        if steps >= 32:
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
        
        if steps_ramp > 0:
            # ramp-up (and ramp-down) speed is 66% of the target speed
            reduction = 1.5
            delay_ramp = min(int(reduction * delay), 262143)
            
            # pack the stepper parameters for the ramp-up and ramp-down
            packed_ramp = self.pack_parameters(delay_ramp, steps_ramp)
            
            # pack the stepper parameters for stable speed
            packed_target = self.pack_parameters(delay, steps - 2 * steps_ramp)
            
            # send data to the PIO steps generator
            self.sm0.put(packed_ramp)
            self.sm0.put(packed_target)
            self.sm0.put(packed_ramp)
        
        else:
            # pack the stepper parameters for the 32bit PIO variable
            packed = self.pack_parameters(delay, steps)
        
            # send data to the PIO steps generator
            self.sm0.put(packed)
        
        # activate the SM0 (stepper will turn once the sync pimp goes high)
        self.sm0.active(1)

        # update position estimate (for tracking)
        if self.direction != self.reverse:
            self.estimated_pos += steps
        else:
            self.estimated_pos -= steps

        if self.printout:
            if self.reverse:
                print(f"[core 0] Motor prepared: dir={'CCW' if direction else 'CW'}, "
                      f"         delay={delay}, steps={steps}")
                print(f"[core 0] Estimated new position: {self.estimated_pos}")
            else:
                print(f"[core 0] Motor prepared: dir={'CW' if direction else 'CCW'}, "
                      f"         delay={delay}, steps={steps}")
                print(f"[core 0] Estimated new position: {self.estimated_pos}")
        
        return True
    
    
    
    def run_stepper(self, speed, steps):
        """
        Main method called from I2C handler.
        speed: 16-bit value where <32768 = CCW, >32768 = CW, 32768 = stop
        steps: number of steps to move
        """
        if shared_variables.halt.read():  # case the shared_variables.halt variable is set True
            self.stop_stepper()
            return
        
        if speed == 32768:
            self.stop_stepper()
            if self.printout:
                print("[core 0] speed == 32768 at run_stepper - stopping motor")
                print("[core 0] Previous position", self.position)
            return
        
        elif speed <= 20:
            self._special_functions(field1=speed, field2=steps)
            return
        
        if not self.motor_enabled:
            if self.printout:
                print("[core 0] Motor disabled, ignoring command")
            return
        
        # determine direction and delay
        speed_k = 8
        if speed < 32768:
            direction = 1 if self.reverse else 0        # CW if self.reverse otherwise CCW
            # delay uses a 18bit range (0 to 262144)
            delay = speed_k * (32768 - (32768 - speed))
        elif speed > 32768:
            direction = 0 if self.reverse else 1        # CCW if self.reverse otherwise CW
            # delay uses a 18bit range (0 to 262144)
            delay = speed_k * (32768 - (speed - 32768))
        
        # prepare motor (will wait for sync, if no ramps...)
        success = self._drive_stepper(direction, delay, steps) #compensated_steps)
    

    
    def _special_functions(self, field1, field2):
        """
        Handle special function commands (speed < 20).
        
        field1: command code (0-19)
        field2: command parameter
        
        Command codes (all are write-only except status):
        0:  Set microstepping (field2: 0=1/8, 1=1/32, 2=1/64, 3=1/16)
        1:  Reset variables and zero position counter
        2:  Disable motor
        3:  Enable motor
        4:  Set StallGuard threshold (field2: 0-255)
        5:  Set homing frequency (field2: 400-1800 Hz)
        6:  Start sensorless homing (field2: k_factor 1-100)
        7:  Get motor status (returns bitmask - this is the ONLY read command)
        8:  Reverse motor direction
        9:  Hard reset of RP2040, via machine.reset()
        10: Set stepper holding current IHOLD, after TPOWERDOWN time since last step
        11: Set TPOWERDOWN time, to lower the stepper current to IHOLD
        
        """
        
        # command 0: set microstepping
        if field1 == 0:
            if field2 >= 0 and field2 <= 3:
                self._update_ms_settings(field2)
                if self.printout:
                    print(f"[core 0] Microstepping changed to {field2}")

        # command 1: reset variables
        elif field1 == 1:
            if self.printout:
                print("\n[core 0] Reset_variables and zeroing position counter\n")
            self.reset_variables()
            
        # command 2: disable motor
        elif field1 == 2:
            if self.printout:
                print("\n[core 0] Received I2C request to disable the motor\n")
            self.disable_stepper()
            
        # command 3: enable motor
        elif field1 == 3:
            if self.printout:
                print("\n[core 0] Received I2C request to enable the motor\n")
            self.enable_stepper()
        
        # command 4: set StallGuard threshold
        elif field1 == 4:
            if self.printout:
                print(f"\n[core 0] Setting TMC StallGuard threshold = {field2}\n")
            self.set_stallguard(threshold=field2)
        
        # command 5: set homing frequency
        elif field1 == 5:
            if self.printout:
                print(f"\n[core 0] Setting homing speed frequency = {field2}Hz \n")
            self._set_homing_freq(homing_freq=field2)
        
        # command 6: start sensorless homing
        elif field1 == 6:
            if self.printout:
                print(f"\n[core 0] Sensorless homing with k_factor = {round(field2/100,2)}\n")
            self.reset_sm0_state()
            sleep_ms(50)
            
            # clear any previous stall/homing flags
            self.stallguarded = False
            self._homing_phase = None
            self._update_status()
            
            # start homing (non-blocking on the I2C side)
            # note: This will run in background while I2C continues
            self._homing(k_factor=field2)
        
        # command 7: get motor status - THIS IS THE ONLY READ COMMAND
        elif field1 == 7:
            """
            Return current motor status via I2C read.
            The I2C handler will read shared_variables.motor_status
            """
            self._update_status()  # Ensure status is current
            if self.printout:
                status = self.get_status()
                if status != self._last_status:
                    print(f"[core 0] Status: 0x{status:02X} - {self._decode_status(status)}")
                    self._last_status = status
                    
            # status is automatically returned via I2C read
        
        # command 8: maintain or reverse motor direction
        elif field1 == 8:
            "Maintain or reverse the motor direction"
            if field2 == 0:
                self.reverse = False
                if self.printout:
                    print(f"\n[core 0] Received I2C request to maintain the motors direction\n")
            elif field2 == 1:
                self.reverse = True
                if self.printout:
                    print(f"\n[core 0] Received I2C request to reverse the motors direction\n")
            
            
        # command 9: reset RP2040
        elif field1 == 9:
            "Hard reset the RP2040, via machine.reset() command"
            reset()
            
        # command 10: set IRUN and IHOLD (packed into field2)
        # field2 bits: high byte (bits 8-12) = irun (0-31), low byte (bits 0-4) = ihold (0-31)
        elif field1 == 10:
            irun = (field2 >> 8) & 0x1F      # extract upper 5 bits (0-31)
            ihold = field2 & 0x1F            # extract lower 5 bits (0-31)
            self._set_stepper_current(irun=irun, ihold=ihold)
            if self.printout:
                print(f"[core 0] Set IRUN={irun}, IHOLD={ihold}")

        # command 11: set TPOWERDOWN (auto current reduction delay)
        # field2: 20-255, delay ≈ 2~5.6 seconds
        elif field1 == 11:
            self._set_power_down_delay(delay=field2)
        
        else:
            print(f"\n[core 0] Command {field1} not recognized\n")
    

    
    
    def _set_homing_freq(self, homing_freq = 1000):
        """
        Set the stepper frequency when sensorless homing.
        This parameter can be modified via I2C.
        """
        min_freq = 400          # min stepper frequency for sensorless homing
        max_freq = 1800         # max stepper frequency for sensorless homing
        
        # homing speed (frequency) clamped in range min ~ max
        self.homing_freq = self.SG_adj * max(min_freq, min(homing_freq, max_freq))
        
    
    def _clear_homing_flags(self):
        """Clear all homing-related flags and update status"""
        self._homing_phase = None
        self._homing_failed = False
        self.stallguarded = False
        self._update_status()
    
    
    def _homing(self, k_factor=45):
        """
        Spins the stepper at stepper_freq until StallGuard value fall below threshold.
        The motor moves shortly away from home prior the homing attempt.
        Argument h_speed is the PIO value to get stepper_freq at the GPIO pin.
        k_factor reduction coefficient (<= 0.5 according to TMC datasheet, to be tuned in final application).
        
        Returns: True if homing successful, False otherwise
        """
        
        if self.printout:
            print("\n[core 0] Homing started...")
        
        self.home = False
        self._homing_failed = False
        
        # some constants
        startup_loops = 15                           # number of code loops for the stepper startup
        retracting_steps = int(self.full_rev / 16)   # number of retrieve steps (move awai from home in case it's already home)
        max_homing_steps = int(self.full_rev)        # max number of steps to reach home
        
        max_retracting_ms = int(200 + 1000 * retracting_steps / self.homing_freq) # timeout in ms when retracting from home
        max_homing_ms = int(1000 * max_homing_steps / self.homing_freq)           # timeout in ms when homing
        
        # passed k_factor clamped between 0.01 and 1
        k_factor = max(1, min(k_factor, 100)) / 100  # k_factor clamped between 0 and 1
        
        # calculate the SG threshold reference to which SG_value readings from UART are compared
        min_sg_expected = int(0.15 * self.homing_freq / self.SG_adj)  # expected minimum StallGuard value on free spinning
        
        # calculate the SG threshold, to be set on the stepper driver and acting to the DIAG pin
        sg_threshold_diag = int(k_factor * min_sg_expected) # StallGuard threshold acting on the DIAG pin

        # calculate the value to pass to the sm0 for the intended homing frequency (speed)
        stepper_val = self.get_stepper_value(self.homing_freq) # stepper pio value is calculated
        
        self.stallguarded = False                    # flag tracking the StallGuard at GPIO is set initially False
        self.home = False                            # home flag is set as False
        
        # ===== PHASE 1: RETRACT FROM HOME =====
        self._homing_phase = 'retract'
        self._update_status()  # Status will show HOMING | RETRACT | BUSY | MOVING
        
        if self.printout:
            print("[core 0] Phase 1: Retracting from home position")
        
        # retract the motor away from home, in case it is already at home
        self.stop_stepper()                          # stepper is stopped (in the case it wasn't)
        direction = 0 if self.reverse else 1         # set stepper direction to 0 (CW) if self.reverse otherwise 1 (CCW)
        self.motor_dir.value(direction)              # set stepper direction to 1 (CW) if self.reverse otherwise 1 (CCW)
        self.set_pls_counter(0)                      # sets the initial value for stepper steps counting
        self.set_stallguard(threshold = 0)           # set SG threshold acting on the DIAG pin, to max torque
        packed_target = self.pack_parameters(stepper_val, retracting_steps) # speed and number of steps to retract from home
        self.sm0.put(packed_target)                  # stepper speed and number of steps
        self.is_stepper_busy(True)                   # flag monitoring/setting the motor state, is set True
        self.start_stepper()                         # stepper is started
        
        t_ref = ticks_ms()                           # time reference for the retracting movement time
        while ticks_ms() - t_ref < max_retracting_ms: # wait time while the motor spins
            if self._stepper_busy:                   # case the motor is still busy
                sleep_ms(20)                         # waits some little time
            else:                                    # case the moto is not anymore busy
                self.is_stepper_busy(False)          # flag monitoring/setting the motor state, is set True
                break                                # break the while loop
        
        
        if self.printout:                            # case self.printout is set True
            retract_time = ticks_ms() - t_ref        # time for the stepper retracting
            steps_done = self.get_pls_count()        # number of steps made by the stepper during retract
            print(f"[core 0] Retracted the stepper by {steps_done} steps, in {retract_time} ms ")
        

        # ===== PHASE 2: ACTUAL HOMING =====
        self._homing_phase = 'homing'
        self._update_status()                        # status will show HOMING | BUSY | MOVING (RETRACT cleared)
        
        if self.printout:
            print("[core 0] Phase 2: Searching for home position")

        # home the motor
        direction = 1 if self.reverse else 0         # set stepper direction to 1 (CW) if self.reverse otherwise 0 (CCW)
        self.motor_dir.value(direction)              # set stepper direction to 1 (CW) if self.reverse otherwise 0 (CCW)
        self.set_stallguard(threshold = 0)           # set SG threshold acting on the DIAG pin, to max torque (startup the motor)
        self.set_pls_counter(0)                      # sets the initial value for stepper steps counting
        packed_target = self.pack_parameters(stepper_val, max_homing_steps) # speed and number of steps to find home
        self.sm0.put(packed_target)                  # stepper speed
        self.is_stepper_busy(True)                   # flag monitoring/setting the motor state, is set True
        self.start_stepper()                         # stepper is started
        
        t_ref = ticks_ms()                           # time reference
        i = 0                                        # iterator index
        while ticks_ms() - t_ref < max_homing_ms:    # while loop until timeout (unless Stalling detection)
            i+=1                                     # iterator index is increased
            if self.home:
                break
            if i > startup_loops:                    # case of at least startup_loops SG_readings (avoids startup effect to SG)
                self.set_stallguard(threshold = sg_threshold_diag)  # set SG threshold acting on the DIAG pin
                while ticks_ms() - t_ref < max_homing_ms: # while loop until timeout (unless Stalling detection)

                    if self.check_stallguard_pin():
                        self.stop_stepper()          # stepper is stopped
                        self.reset_sm0_state()       # sm0 buffer is cleared
                        self.home = True             # home flag is set True
                        self.is_stepper_busy(False)  # flag monitoring/setting the motor state, is set False
                        break

        self.set_stallguard(threshold = 0)           # set SG threshold acting on the DIAG pin, to max torque
        
        # ===== CLEANUP =====
        self._clear_homing_flags()                   # clear homing phase flags and update status
        
        if self.home:
            if self.printout:
                print("[core 0] Homing completed successfully!")
            self._update_status()                    # status will show HOMED (if enabled)
            return True                              # return True (successfull homing)
        
        else:
            self.stop_stepper()                      # stepper is stopped (if not done by the _homing func)
            self._homing_failed = True
            self._update_status()                    # status may show ERROR if we add that bit
            if self.printout:
                print("[core 0] Failed homing")               # feedback is printed to the Terminal
            return False                             # return False (homing failure)




stepper_controller = StepperController()
