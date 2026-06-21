"""
This code is a small extract of a much larger driver for TMC 2209 stepper driver.
In this implementation are only kept the StallGuard related functions.

Acknowledgements:
Many thanks to Chr157i4n for making that exstensive TMC_2209 library.
Many thanks to anonymousaga for his adaptation for Raspberry Pi Pico.


Original file: TMC_2209_StepperDriver.py

Source: https://github.com/troxel/TMC_UART
Source: https://github.com/Chr157i4n/TMC2209_Raspberry_Pi
Source: https://github.com/kjk25/TMC2209_ESP32
Source: https://github.com/anonymousaga/TMC2209_RPI_PICO

Copyright (c) 2020 troxel
Copyright (c) 2014 The Python Packaging Authority (PyPA)
Copyright (c) 2014 Mapbox

Modified by: Andrea Favero (07/06/2025)

Licensed under the GNU General Public License v3.0

"""



from TMC_2209_uart import TMC_UART
from machine import Pin


#-----------------------------------------------------------------------
# TMC_2209
#
# this class has two different functions:
# 1. change setting in the TMC-driver via UART
# 2. move the motor via STEP/DIR pins
#-----------------------------------------------------------------------
class TMC_2209:
    
#-----------------------------------------------------------------------
# constructor
#-----------------------------------------------------------------------
    def __init__(self, rx_pin, tx_pin, mtr_id=0, baudrate=115200, serialport=0):
        self.tmc_uart = TMC_UART(serialport, baudrate, rx_pin, tx_pin, mtr_id)

        self.GCONF      =   0x00
        self.IHOLD_IRUN =   0x10
        self.TPOWERDOWN =   0x11
        self.TCOOLTHRS  =   0x14
        self.SGTHRS     =   0x40
        self.SG_RESULT  =   0x41
        
        # bit for setting the PDN (Power Down Pin) via GCONF register
        self.GCONF_PDN_DISABLE = 6

        # disable the PDN (Power Down Pin), allowing for UART control on this response
        self.disablePdnPin()
        
        # set the delay time start reducing power after the 'last' step
        self.setPowerDownDelay()

#-----------------------------------------------------------------------
# destructor
#-----------------------------------------------------------------------
    def __del__(self):
        print("[TMC 2209] Closing TMC driver serial comm")
        self.tmc.close()



#-----------------------------------------------------------------------
# deactivate PDN pin (PowerDown) to enable power down via UART
#-----------------------------------------------------------------------
    def disablePdnPin(self):
        print("[TMC 2209] Disable PDN pin")
        try:
            val = self.tmc_uart.read_int(self.GCONF)
            val |= (1 << self.GCONF_PDN_DISABLE)       # set pdn_disable
            self.tmc_uart.write_reg_check(self.GCONF, val)
        
        except Exception as e:
            print(f"[TMC 2209] Failed to disable PDN pin: {e}")



#-----------------------------------------------------------------------
# return the current stallguard result
# its will be calculated with every fullstep
# higher values means a lower motor load
#-----------------------------------------------------------------------
    def getStallguard_Result(self):
        sg_result = self.tmc_uart.read_int(self.SG_RESULT)
        return sg_result



#-----------------------------------------------------------------------
# sets the register bit "SGTHRS" to a given value
# this is needed for the stallguard interrupt callback
# SG_RESULT becomes compared to the double of this threshold.
# SG_RESULT ≤ SGTHRS*2
#-----------------------------------------------------------------------
    def setStallguard_Threshold(self, threshold):
        self.tmc_uart.write_reg_check(self.SGTHRS, threshold)



#-----------------------------------------------------------------------
# This  is  the  lower  threshold  velocity  for  switching  
# on  smart energy CoolStep and StallGuard to DIAG output. (unsigned)
#-----------------------------------------------------------------------
    def setCoolStep_Threshold(self, threshold=1600):
        # CoolStep and StallGuards are active when threshold > TSTEP (timeStep)
        # TSTEP is the step period in TMC clocks units (f=2MHz) calculated for 1/256 microstep
        # threshold = 3200 ensures STALLGUARD operation for
        #   --> f > 400Hz  at 1/8  microstep
        #   --> f > 800Hz  at 1/16 microstep
        #   --> f > 1600Hz at 1/32 microstep
        #   --> f > 3200Hz at 1/64 microstep
        self.tmc_uart.write_reg_check(self.TCOOLTHRS, threshold)



#-----------------------------------------------------------------------
# set a function to call back, when the driver detects a stall 
# via stallguard
# high value on the diag pin can also mean a driver error
#-----------------------------------------------------------------------
#     def setStallguard_Callback(self, threshold = 50): #, handler=None):
#         self.setStallguard_Threshold(threshold)
#         self.setCoolStep_Threshold()
#         gp11 = Pin(11, Pin.IN, Pin.PULL_DOWN)
#         gp11.irq(trigger=Pin.IRQ_RISING, handler=handler)



#-----------------------------------------------------------------------
# test the UART functionality
#-----------------------------------------------------------------------
    def test(self):
        return self.tmc_uart.test()



#-----------------------------------------------------------------------
# set holding current (motor stopped, with default holding current)
#-----------------------------------------------------------------------
    def setStepperCurrent(self, irun=31, ihold=15, iholddelay=5):
        """
        Set motor run current and holding current.
        
        irun: 0-31 (0=3%, 31=100%) - Current when motor is moving
        ihold: 0-31 (0=3%, 15=50%, 31=100%) - Current when motor is stopped
        iholddelay: 0-15 - Additional delay before reduction, units of ~0.022 seconds
        """
        irun = max(0, min(irun, 31))
        ihold = max(0, min(ihold, 31))
        iholddelay = max(0, min(iholddelay, 15))
        
        irun_percent = int(100 * irun / 31)
        ihold_percent = int(100 * ihold / 31)
        delay_seconds = iholddelay * 0.022
        
        print(f"[TMC 2209] Current: IRUN={irun} ({irun_percent}%), IHOLD={ihold} ({ihold_percent}%))")
        
        val = (irun << 8) | (iholddelay << 16) | ihold
        self.tmc_uart.write_reg_check(self.IHOLD_IRUN, val)


#-----------------------------------------------------------------------
# defines the delay from the last step to start reducing ihold
#-----------------------------------------------------------------------
    def setPowerDownDelay(self, delay=20):
        """
        Set delay from standstill to current power down.
        
        delay: 2-255 (minimum 2 required for StealthChop auto tuning)
        Actual time = delay * 2^18 / 12MHz ≈ delay * 0.022 seconds
        Range: about 0.04s to 5.6s
        """
        actual_seconds = delay * 0.022
        print(f"[TMC 2209] TPOWERDOWN={delay} (current reduction after ~{actual_seconds:.2f}s of standstill)")
        self.tmc_uart.write_reg_check(self.TPOWERDOWN, delay)
        
