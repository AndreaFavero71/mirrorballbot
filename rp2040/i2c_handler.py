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
from i2c_responder import I2CResponder
from rgb_led import rgb_led
from machine import Pin
import time

class I2CHandler:
    
    def __init__(self, fields=1):
        print("\n[core 1] Uploading i2c_handler ...")
        
        
        i2c_address = self.get_i2c_address()        # gets the RP2040 I2C address, based on the hardware location (GPIO pins value)
        print("[core 1] Detected i2c_address:", i2c_address) # feedback is printed to the terminal
        
        # sda and scl pins
        sda_pin = shared_variables.I2C0_SDA_PIN
        scl_pin = shared_variables.I2C0_SCL_PIN
        
        # instantiate the I2C responder
        self.s_i2c = I2CResponder(i2c_device_id=0, sda_gpio=sda_pin, scl_gpio=scl_pin, responder_address=i2c_address)
        
        self.df_fields = fields                      # (max 4) number of (16bits) fields per dataframe via I2C
        print("[core 1] Number of fields:", self.df_fields)
        
        self.max_buffer = 2 + 4 * self.df_fields     # max buffer size
        self.raw_data = bytearray()                  # bytearray storing bytes arriving at the i2C
        
        self.i2c_counter = shared_variables.i2c_counter.read()
        self.prev_i2c_counter = -1                   # stores the last i2c counter (16 bit integer overflowing to 0)
        print()
        
    
    
    
    def get_i2c_address(self):
        id0 = Pin(shared_variables.I2C_ADR_ID0_PIN, Pin.IN, Pin.PULL_UP)   # Input pin for ID0
        id1 = Pin(shared_variables.I2C_ADR_ID1_PIN, Pin.IN, Pin.PULL_UP)   # Input pin for ID1
        i2c_address_map = {(1, 1): 0x41, (0, 1): 0x42, (1, 0): 0x43, (0, 0): 0x44} # map pin states (id0, id1) to I2C addresses
        pin_values = (id0.value(), id1.value())          # get pin values as a tuple
        i2c_address = i2c_address_map.get(pin_values)    # retrieve the I2C address from the map
        
        if i2c_address is None:                          # case i2c_address is None (should never happen)
            print("[core 1] Error: Invalid pin state for I2C address")  # feedback is opprinted to the terminal
        
        return i2c_address
    
    
    
    def calculate_checksum(self, data):
        """ Returns the checksum of the received data, as module of 256."""
        return sum(data) & 0xFF
    
    
    
    def _process_received_data(self, df_fields, max_buffer, red_led=False, blue_led=False):
        """
        Analyze the raw_data and return the clean_data.
        This means finding the STX header character (0x02) and the 
        the ETX terminator character (0x03).
        The \ escape character (0x5c) is used in front of the stx, etx (also the escape
        character) when these are part of the data. This requires the removal of the escape
        character in these cases.
        """
        
        raw_data = self.raw_data
#         df_fields = self.df_fields
        
        if len(raw_data) < 2 + 2 * df_fields:           # case to little data
            return [], False, False, False              # returns empty data and False checksum
        
        data = []
        checksum_result = False                         # checksum_result is initially set False
        start = None                                    # start is the STX index in raw_data, initially set on None
        i = 0
        while i < len(raw_data):                        # iterating though the raw_data
            byte = raw_data[i]                          # byte at the i index position
            prev_byte = raw_data[i - 1] if i > 0 else None  # byte in previous index position
            if byte == 0x02 and prev_byte != 0x5C:      # case the byte equals to stx and not escape in front
                start = i                               # dataframe-start index (STX index location in raw_data)
            elif byte == 0x03 and start is not None:    # case the byte equals to ETX and STX not None
                is_escaped = prev_byte == 0x5C          # bool of previous byte being an escape character
                double_escape = is_escaped and i > 1 and raw_data[i - 2] == 0x5C # bool of prev two bytes == escape character
                
                if not is_escaped or double_escape:     # case index i is not following 1 or 2 escapes
                    stop = i                            # dataframe-end index (ETX index location in raw_data)
                    if i == len(raw_data) - 2 and raw_data[i + 1] == 0x03: # case EXT is followed by a second ETX
                        stop += 1                       # dataframe-end index takes the second ETX
                    if stop > start + 2 * df_fields:    # case dataframe has enough data
                        clean_data = self._escapes_removal(raw_data, start, stop) # removing escape characters
                        data, checksum_result, red_led, blue_led = self._validate_data(clean_data, df_fields)   # validating data
                        break                           # end of the while loop
            i += 1                                      # move to next byte index
        return data, checksum_result, red_led, blue_led   # interpreted data and checksum result are returned
    
    
    
    
    def _escapes_removal(self, raw_data, start, stop):
        """
        Process data between STX and ETX, handling escape sequences properly.
        """ 

        clean_data = bytearray()                   # bytearray for dataframe purged from STX, ETX and escapes characters
        escape_next = False                        # checksum_result is set False
        
        idx = start                                # iteration start at the raw_data index where STX was found 
        while idx <= stop:                         # iteration until raw_data index where ETX was found
            byte = raw_data[idx]                   # byte at idx index location
            if escape_next:                        # case escape_next is True
                clean_data.append(byte)            # byte is appended to clean_data
                escape_next = False                # case escape_next is False
            
            elif byte == 0x5C:                     # case of escape character at idx raw_data index location
                if idx + 1 <= stop and raw_data[idx + 1] == 0x5C: # case escape is followed by another escape
                    clean_data.append(0x5C)        # one escape is appended
                    idx += 1                       # index is increased (to slip the second escape)
                else:                              # case escape is not followed by another escape
                    escape_next = True             # escape_next is set True (skip this escape)
            else:                                  # case byte is not an escape
                clean_data.append(byte)            # byte is appended to clean_data
            idx += 1                               # move to the next byte
        return clean_data                          # return the clean_data
        
    
    
    def _validate_data(self, clean_data, df_fields):
        """
        The clean_data is a dataframe of bytes: STX + n * fields (2 bytes each) + checksum + ETX
        The n fields are calculated from the relative bytes.
        The checksum is retrieved from the clean_data.
        The checksum is calculated (it refers to the STX + the n * fields, ETX is excluded).
        The calculated checksum is confronted with the one received, and a boolean returned.
        """

        data = []                        # empty list storing the interpreted data from datafram
        checksum_result = False                                     # checksum_result is set False
        red_led =False
        blue_led=False
        
        if len(clean_data) < 3 + 2 * df_fields:                     # case clean_data has too little data
            red_led = True                                          # set the flag for red led flashing
            print("[core 1] Incomplete message:", list(self.raw_data), list(clean_data)) # feedback is printed
            return data, checksum_result, red_led, blue_led         # return empty data and False checksum
        
        for i in range(0, 2 * df_fields, 2):                        # iteration over the even byte 
            value = (clean_data[i+1] << 8) | clean_data[i+2]        # generate a 16bit value out of 2 bytes
            data.append(value)                                      # interpreted value is appended
        checksum = clean_data[-2]        # received chacksum (8 bits) i retrieved from the clean_data
        
        # case the checksum calculated on received (and interpreted) data equals the one received
        if self.calculate_checksum(clean_data[:-2]) == checksum:
            checksum_result = True                                  # local flag is set True
            blue_led = True                                         # set the flag for blue led flashing
        else:                        # case the calculated checksum differs from the one received                                
            red_led = True                                          # set the flag for red led flashing
            print("[core 1] Checksum error")                        # feedback is printed to the terminal
        
        return data, checksum_result, red_led, blue_led
    
    
    
    def _read_i2c_data(self, byte, df_fields, max_buffer):
        """
        Receives one byte at the time, and feeds the raw_data list.
        For every byte received, it calls the function that checks if a complete dataframe is formed.
        In case a correct dataframe is found, the led is shortly turned on/off.
        """
        raw_data = self.raw_data              # local variable from instace variable
        data = []                             # list to store the interpreted data
        
        raw_data.append(byte)                 # the recived byte is appended to the raw_data list
        
        # checks if a complete dataframe is retrieved
        data, checksum_result, red_led, blue_led = self._process_received_data(df_fields, max_buffer)
        
        if checksum_result:                   # case of correct checksum
            self.raw_data = bytearray()       # self.raw_data is 'cleaned'
        else:                                 # case of incorrect checksum
            if len(raw_data) > max_buffer:    # case of local raw_data is longher than max_buffer
                self.raw_data = raw_data[1:]  # older byte is sliced out and the all assigned to instance variable
        
        return data, checksum_result, red_led, blue_led  # data and checksum fals are returned
    
    
    
    def run(self):
        """
        This is essentially the main function of this Class.
        It keep checking whether there is data arrival or request at i2c.
        If there is data arrival, it checks it: If it's the completion of a dataframe global variables are updated
        If there is data request, it reply with 3 possible bytes:
            0 if the last received data completed a dataframe with not correct checksum
            1 if the last received data completed a dataframe with correct checksum
            9 if there is no data received yet
        """
        s_i2c = self.s_i2c             # local object of the i2c instance  
        df_fields = self.df_fields     # local variable from instace variable of number of fields in dataframe
        max_buffer = self.max_buffer   # local variable from instace variable of max bytes in buffer
    
        data = []                      # empty list to be populated with data received at i2c
        check_ok = False               # flag for coherent i2c data receival is initially set False
        printout = True                # flag to print some data to the terminal (only for debug purpose)
        
        last_received_data = ''
        
        while True:                    # infinite loop
            
            if shared_variables.halt.read():  # case the shared_variables.halt variable is set True
                break                         # infinite loop is interrupted
            
            if s_i2c.write_data_is_available():             # case there is data at the i2c arrival buffer
                data, check_ok, red_led, blue_led = self._read_i2c_data(s_i2c.get_write_data()[0],
                                                                        df_fields, max_buffer)  # byte is read and analyzed
                
                if check_ok and len(data) > 0:              # case the received data reppresents a complete dataframe
                    i2c_counter = shared_variables.i2c_counter.read()  # i2c data counter counter is assigned
                    
                    for i, field in enumerate(shared_variables.fields[:df_fields]): # iteration over the number of fields
                        field.write(data[i])                # updated value at the specific data sharing memory location
                    shared_variables.i2c_counter.write(i2c_counter + 1)  # increment i2c counter at specific memory location
                    
                    if blue_led:                            # case the blue led flag is set True
                        rgb_led.fast_flash_blue(ticks=10)   # very short flashing of blue led
                        
                    if printout:                            # case printout is set True
                        if data != last_received_data:
                            print("\n[core 1] Received data:", data) # feedbaclk is printed to the terminal
                            last_received_data = data

                if red_led:                                 # case the red led flag is set True
                    rgb_led.fast_flash_red(ticks=20)        # short flashing of red led
            
            elif s_i2c.read_is_pending():                   # case there is i2c data request    
                if len(data) > 0:                           # case there is data from previous i2c data arrival
                    if check_ok:                            # checksum ok
                        status = shared_variables.motor_status.read() & 0xff # get the status bitmask
                        self.s_i2c.put_read_data(status)    # return the status bitmask
                    else:                                   # checksum error
                        self.s_i2c.put_read_data(255)       # 255 = checksum error
                else:                                       # no data received yet
                    self.s_i2c.put_read_data(254)           # 254 = no command yet
    
    
    
    
    def dummy_i2c(self, delay_us=10000, target=10):
        """ Function to mimick i2c data receival.
            I2c receives data at 8bits at the time, with the following dataframe format:
            [etx, field1_HSB, field1_LSB, field2_HSB, field2_LSB, field3...., checksum, etx]
            wherein the checsum sums up etx and field(s) not the etx.
        """

        from urandom import randint

        print("\n[core 1] Running the dummy function mimicking I2C data arrival\n")
        print("[core 1] The test stops after 20 seconds\n")
        
        df_fields = 1                # local variable from instace variable of shared_variables.fields
        max_buffer =  2 + 4 * df_fields   # max buffer size
        t_start = time.ticks_ms()    # time reference for timing check
        
        dataframe = []               # dataframe is initialized as emptu list
        stx = 0x02                   # stx character used to start a dataframe
        etx = 0x03                   # etx character used to end a dataframe
        escape = 0x05c               # escape character used in front of fields when field value equals to stx, etx or escape
        exclude = (stx, etx, escape) # these values are send to i2c with an escape character in front, here excluded to keep it simple
        checksum = 0                 # checksum is initialized as zero
        runs = 0                     # runs is initialized as zero, and used to count the while True loops
        test_time = 20               # timeout for the test
        start_time = time.time()
        
        while time.time()-start_time < test_time:     # looping for test_time
            
            if shared_variables.halt.read():          # case the shared_variables.halt variable is set True
                print("[core 1] Core 1 stopping safely") # feedback is printed to the terminal
                break                                 # exit the infinite while True loop
            
            # generating a dataframe as per those received at i2c
            while len(dataframe) == 0:                # while loop until dataframe is empty
                if shared_variables.halt.read():      # case the shared_variables.halt variable is set True
                    break                             # exit the while loop
                
                dataframe = []                        # dataframe is set back empty                    
                dataframe.append(stx)                 # stx is appended to dataframe
                
                for i in range(2 * df_fields):        # iteration for each data field (2 bytes per each field, as a field is 16bits) 
                    rand = randint(0, 255)            # one byte random integer is generated
                    if rand in exclude:               # case rand is one of the excluded values from the fummy_i2c function
                        while rand in exclude:        # looping until rand equals an excluded value
                            rand = randint(0, 255)    # one byte random integer is generated
                    dataframe.append(rand)            # the random generated byte is appended to the dataframe
                   
                checksum = sum(dataframe) & 0xff      # checksum value is calculated
                if checksum not in exclude:           # case checsum value is not one of the excluded values from the fummy_i2c function
                    dataframe.append(checksum)        # checksum is appended to the dataframe
                    dataframe.append(etx)             # etx is appended to the dataframe
                    break                             # the while empty dataframe loop is interrupted

            # sending the dataframe values to the read_i2c_data reading function
            for val in dataframe:                     # iteration over the elements of dataframe
                data, check_ok, red_led, blue_led = self._read_i2c_data(val & 0xff,
                                                                        df_fields,
                                                                        max_buffer) # the 8bit data is sent to the read_i2c_data function
                time.sleep_us(delay_us)               # delay in betweeen the read_i2c_data function call

            # using the return from the read_i2c_data reading function
            if check_ok and len(data)>0:              # case the read_i2c_data funtion returns coherent data
                for i, field in enumerate(shared_variables.fields[:df_fields]): # iteration over the number of fields
                    field.write(data[i])              # updated value at the specific data sharing memory location
                if blue_led:                          # case the blue led flag is set True
                    rgb_led.fast_flash_blue(ticks=10) # very short flashing of blue led
            
            if red_led:                               # case the red led flag is set True
                rgb_led.fast_flash_red(ticks=20)      # short flashing of red led
            
            runs += 1                                 # counter is incremented (also in case of incoherend data)
            
            if runs >= target -1:                     # case the iterations reach the target
                printout = "Mimicking the i2c data receival: {runs:.d} runs took {time:d} ms"  # printout definition
                print(printout.format(runs = target, time=time.ticks_ms()-t_start))            # feedback is printed to the terminal
                t_start = time.ticks_ms()             # time reference is set again
                runs = 0                              # iteration counter is set back to zero



if __name__ == "__main__":
    """Test function for this Class, mimicking part of the i2C data receival and analysis."""
    i2c_instance = I2CHandler()
    i2c_instance.dummy_i2c(delay_us=20000, target=10)
