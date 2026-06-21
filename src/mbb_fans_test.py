"""
Andrea Favero 20260605

MirrorBallBot (MBB), an alternative ball balance robot

More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code handling the test for the cooling fans




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

__version__ = "0.0.1"


from gpiozero import OutputDevice, PWMOutputDevice
from time import time, sleep
import sys

# MirrorBallBot custom modules
from mbb_settings_mgr import SettingsManager


# ============================================================================
# COOLING FANS CONTROLLER
# ============================================================================

class CoolingFans:
    """Control for the fans according to the RPI temperature"""

    def __init__(self, settings_mgr=None):
        
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

        
        self.fan = PWMOutputDevice(self.FANS_GPIO_PIN, frequency=self.FANS_PWM_FREQ)
        
        self.fan.value = 0


    def get_cpu_temp(self):
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            # the file hold the CPU temp in millidegrees (ie 45000 = 45.0 degrees)
            return float(f.read()) / 1000.0
    
    def fans_off(self):
        self.fan.value = 0
        print(f"\nSwithed the fans OFF")


# ============================================================================
# MAIN EXECUTION to only test the fans 
# ============================================================================
    
def main():
    """Main program entry point for fans test."""
    
    # create a setting manager instance
    settings_mgr = SettingsManager()
    
    # initialize the system
    print("\n" + "="*60)
    print("FANS TEST")
    print("\nNote: PWM range is from 0.0 to 1.0")
    print("Note: At low PWM the fans might not start-up")
    print("="*60)
    
    # auto-calibration runs at startup by default
    fans = CoolingFans(settings_mgr=settings_mgr)
    
    cpu_temp = fans.get_cpu_temp()
    if cpu_temp < fans.CPU_TEMP_FANS_ON:
        print(f"\nCPU temperature is {cpu_temp}°C, so belove the threshold of {fans.CPU_TEMP_FANS_ON}°C\n")
    else:
        print(f"\nCPU temperature is {cpu_temp}°C, so above the threshold of {fans.CPU_TEMP_FANS_ON}°C\n")
    
    
    try:
        test_runs = 5
        for test_run in range(test_runs):
            
            # default PMM
            period = 10
            print(f"\n================ Test run number {test_run + 1} out of {test_runs} ================")
            print(f"Swith-on the fans at default PWM ({fans.FANS_PWM_DUTY_CYCLE}), for {period} seconds")
            fans.fan.value = fans.FANS_PWM_DUTY_CYCLE
            sleep(period)
            
            # fans off
            fans.fans_off()
            sleep(5)
            
            # PMW sweep
            period = 4
            print(f"\nPWM sweep, every pwm level is maintained for {period} seconds:")

            for pwm in range(100, 45, -5):
                pwm = max(0, min(100, pwm))/100
                if pwm < 1:
                    print(f"Fans at PWM {pwm:.2f}")
                    fans.fan.value = pwm
                    sleep(period)
                elif pwm == 1:
                    print(f"Fans at PWM {pwm:.2f} (max speed)")
                    fans.fan.value = pwm
                    sleep(period * 1.5)
                
            
            # fans off
            fans.fans_off()
            if test_run < test_runs - 1:
                sleep(5)
            print()
        
        print("Test conluded\n\n")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")

    

if __name__ == "__main__":
    main()