from collections import deque
import numpy as np
from math import floor
import RPi.GPIO as _GPIO 


#################################################################################
# This file contains useful functions used by main.                             #
#################################################################################

class GPIO:
    
    #################################################################################
    # GPIO abstracts the setup and switching of the GPIO pins away from main to     #
    # keep it tidier. Note that _GPIO refers to the normal raspberry pi class.      #
    #################################################################################
    def __init__(self, main):
        self.main = main
        _GPIO.setwarnings(False)
        _GPIO.setmode(_GPIO.BCM)

        self.angle_zero = 17
        self.led_good = 22
        self.led_violation = 23

        _GPIO.setup(self.angle_zero, _GPIO.IN, pull_up_down=_GPIO.PUD_DOWN)
        _GPIO.add_event_detect(self.angle_zero, _GPIO.RISING)
        _GPIO.add_event_callback(self.angle_zero, self.gpio_button_callback)

        _GPIO.setup(self.led_good, _GPIO.OUT, initial=_GPIO.LOW)
        _GPIO.setup(self.led_violation, _GPIO.OUT, initial=_GPIO.LOW)
        
    def gpio_reset(self):
        #Method used to 'zero' the gpio pins to a known state
        _GPIO.output(self.led_good, _GPIO.HIGH)
        _GPIO.output(self.led_violation, _GPIO.LOW)
                
    def gpio_violation(self):
        #Sets the red external LED as on, and the green pin off
        _GPIO.output(self.led_good, _GPIO.LOW)
        _GPIO.output(self.led_violation, _GPIO.HIGH)

    def gpio_button_callback(self, *args):
        #Called when the button is pressed, used to switch from idle to the main loop
        self.main.initial_idle = False
    
    def gpio_flip(self):
        #Switches the green LED's state, used in the idle mode 
        if _GPIO.input(self.led_good):
            _GPIO.output(self.led_good, _GPIO.LOW)
        else:
            _GPIO.output(self.led_good, _GPIO.HIGH)


class Sensor:
    
    #################################################################################
    # Barebones class that forces new sensors to implement certain functionality.   #
    # Also offers some basic helper functions to keep sensor implementations        #
    # cleaner.                                                                      #
    #################################################################################
    def __init__(self, name, i2c_addr, hist = 100, **kwargs):

        self.i2c_addr = i2c_addr
        #Sensor keeps a note of the last 100 samples (can be overriden), this could be used later to resent missed data
        #Currently used only to provide an average reading
        self.history = deque(maxlen = hist)
        self.name = name

    def read(self):
        #Every sensor will have a different way of reading data (ie, different registers, data processing etc.). 
        #Therefore this function must always be implemented by inheriting classes.
        raise NotImplementedError
    
    def get_average_reading(self, samples, val=None):
        #Gives an average reading of a value over a specific number of samples 
        #val: the reading within history that the average should be taken over 
        
        sample_num = min(samples, len(self.history))#Ensures that there will be no indexing errors
        queue_slice = list(self.history)[-sample_num:]#if design requires self.history to be much larger then this is not an efficient solution, for up to a few thousand entries this is fine
        if val != None:
            queue_slice = [entry[val] for entry in queue_slice]#Required for more complex data returns 
        #Using np.average as queue can be a list of lists, or just a normal list
        return np.average(queue_slice, axis = 0)

    def setup(self, bus):
        #Some sensors don't require any additional setup (e.g. TMP-007) but some initialisation may need to be done, in which case function can be overriden.
        self.bus = bus

    def get_last_reading(self):
        return self.history[-1]

    def form_results(self, reading):
        #form_results is passed the current formatted list of results, and must add its own inline. All sensors are different, hence requiring implimentation
        raise NotImplementedError 

    def twos(self, val, bytes = 2, byteorder = 'big'):
        #Helper function
        #Performs conversion from unsigned int to signed int (using two's complement)
        b = val.to_bytes(bytes, byteorder, signed=False)
        return int.from_bytes(b, byteorder, signed=True)
