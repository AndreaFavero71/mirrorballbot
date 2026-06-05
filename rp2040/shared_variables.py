"""
Andrea Favero 20260605

MirrorBallBot (MBB), a robot seeing through a mirror

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/


# Bit position constants for motor status bitmask
# These allow multiple states to be represented simultaneously in a single byte
BIT_ENABLED   = 0  # 1 << 0 = 1   - Motor driver powered on
BIT_MOVING    = 1  # 1 << 1 = 2   - Rotor currently turning
BIT_HOMED     = 2  # 1 << 2 = 4   - At reference position
BIT_BUSY      = 3  # 1 << 3 = 8   - Executing command (moving OR waiting for sync)
BIT_STALL     = 4  # 1 << 4 = 16  - Stall/overcurrent detected
BIT_ERROR     = 5  # 1 << 5 = 32  - Error state (thermal, overcurrent, etc)
BIT_HOMING    = 6  # 1 << 6 = 64  - Homing procedure active
BIT_RETRACT   = 7  # 1 << 7 = 128 - Retracting from home (sub-phase of homing)



MIT License
Copyright (c) 2026 Andrea Favero
"""

from machine import mem32
from shared_memory import SharedMemory

# The CPUID register for RP2040 is located at 0xd0000000
# Reading it returns 0 for Core 0 and 1 for Core 1
# core_id = mem32[0xd0000000]



class SharedVariables:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init()
        return cls._instance
    
    
    def _init(self):
        
#         print("\nUploading shared_variables ...")
        core_id = mem32[0xd0000000]
        print(f"[core {core_id}] Uploading shared_variables ...")
        
        # pins used at RP2040-ZERO (or RP2350-Zero)
        self.I2C0_SDA_PIN =     0           # i2c0 sda pin
        self.I2C0_SCL_PIN =     1           # i2c0 scl pin
        self.MOTOR_MS1_PIN =    3           # motor microstep MS1 pin
        self.MOTOR_MS2_PIN =    4           # motor microstep MS2 pin
        self.MOTOR_STEPS_PIN =  5           # motor steps pin
        self.MOTOR_DIR_PIN =    6           # motor direction pin
        self.I2C_ADR_ID0_PIN =  7           # input pin id0 definiting the i2c address in combo with id1
        self.I2C_ADR_ID1_PIN =  8           # input pin id1 definiting the i2c address in combo with id0
        self.SYNC_PIN =        14           # synchronization pin
        self.DIAG_PIN =        11           # input pin connected to the DIAG pin of TMC driver
        self.LED_ONBOARD_PIN = 16           # onboard led pin (neopixel led type)
        
        # PIO frequency
        self.FREQUENCY =       5_000_000    # PIO frequency for steps generator
        self.MAX_FREQUENCY = 100_000_000    # PIO frequency for steps counter (RP2040 max 125MHz, RP2350 max 150MHz)
        
        self.PIO_FIX = 8                    # number of fix PIO instrustions when generating steps
        self.PIO_VAR = 4                    # number of variable PIO instrustions when generating steps
        
        self.MOTOR_STEPS = 200              # steps/rev for the motor (can be overuled via I2C)
        

        # define fix memory locations
        HALT_FLAG_ADR   = 0x20041FE0        # memory address with flag halting core1
        I2C_COUNTER_ADR = 0x20041FE2        # memory address with counter for every new I2C data
        MOTOR_STATUS    = 0x20041FEC        # memory address with stepper status
        
        FIELD1_ADR      = 0x20041FE4        # memory address intercores data sharing field 1
        FIELD2_ADR      = 0x20041FE6        # memory address intercores data sharing field 2
        FIELD3_ADR      = 0x20041FE8        # memory address intercores data sharing field 3
        FIELD4_ADR      = 0x20041FEA        # memory address intercores data sharing field 4
        # these address were intentionally chosen toward the upper range limit
        # last available address in RP2040 SRAM 0x20041FFF    (on RP2350 the range is larger)
        # for a 16bit the last usable address is 0x20041FFC
        
        self.halt = SharedMemory(HALT_FLAG_ADR)
        self.i2c_counter = SharedMemory(I2C_COUNTER_ADR)
        self.motor_status = SharedMemory(MOTOR_STATUS)
        
        self.halt.write(0)                  # 0 = run, 1 = halt
        self.i2c_counter.write(0)           # assign 0 to the i2c counter
        self.motor_status.write(4)          # 1 = home, 2 = busy, 3 = out of home, 4 = disabled

        self.field1 = SharedMemory(FIELD1_ADR)
        self.field2 = SharedMemory(FIELD2_ADR)
        self.field3 = SharedMemory(FIELD3_ADR)
        self.field4 = SharedMemory(FIELD4_ADR)        
        self.fields = (self.field1, self.field2, self.field3, self.field4) # tuple with the fields objects
        
        # define initial values for the used fix memory locations
        self.field1.write(32768)                # set initial field 1 (motor speed) value to 32768 (= stop motor)
        for field in self.fields[1:]:           # iterate over the other fields
            field.write(0)                      # set initial value to 0
        
        # field1 is used for the stepper speed. If value < 20 it is used for special purposes
        # field2 is used for the stepper steps. If field1 < 20, field2 is used to pass the parameter for the special purpose
        # field3 is not used (set at the project beginning, when working on the communication protocol, yet not needed)
        # field4 is not used (set at the project beginning, when working on the communication protocol, yet not needed)
        
        




shared_variables = SharedVariables()