import smbus
from collections import deque
import time
from datetime import datetime
import threading
import json
import os
import numpy as np
from util import Sensor, GPIO
from mqtt import Client
import RPi.GPIO as GPIO_module 

class Motion(Sensor):
    #################################################################################
    # Class adds specific behaviour relevant to the GY-521 sensor to the generic    #
    # Sensor class defined in utils.py. Due to having several measurements it       #
    # performs a larger amount of data handleing than the standalone temperature    #
    # sensor.                                                                       #
    #################################################################################
    def __init__(self, hist = 100):
        super().__init__("Motion", 0x68)
        #All registers are 8 bits, 2 registers per value -> 16 bit 2s complement
        #Note that recorded address is the register of the most significant byte of the value, also need to access the following register for the full 16 bit
        #pg 29-31 of the MPU-6000 register map document for sensitivities/range selection, lowest range is selected in all cases as experiments showed higher ranges weren't needed
        #---------------------------------------------------
        #Accelerometer
        self.reg_accel_addrs = [
            0x3B, #most significant X
            0x3D, #most significant Y
            0x3F, #most significant Z
        ]
        #---------------------------------------------------
        #Temperature 
        self.reg_temp_addr = 0x41
        #---------------------------------------------------
        #Gyroscope
        self.reg_gyro_addrs = [
            0x43, #most significant X
            0x45, #most significant Y
            0x47, #most significant Z
        ]
        #---------------------------------------------------
        #The zeroed angle vector, defaults to sensor being horizontal (ie, 1g of acceleration in the z plane)
        self.angle_initial = [0,0,1]

    def setup(self, bus):
        self.bus = bus
        #Wake from sleep mode
        self.bus.write_i2c_block_data(self.i2c_addr, 0x6B, [0])
        #Set gyroscope resolution/ensure that self test is off
        self.bus.write_i2c_block_data(self.i2c_addr, 0x1B, [0])
        #Set accelerometer resolution/ensure that self test is off
        self.bus.write_i2c_block_data(self.i2c_addr, 0x1C, [0])
        #Set the initial angle needed
        self.set_angle_initial()

    def set_angle_initial(self):
        #Gets average zero vector over a period of a second when activated 
        for i in range(10):
            self.read()
            time.sleep(0.1)
        self.angle_initial = self.get_average_reading(10, 'accelerometer')

    def read(self):
        #Accelerometer
        new_reading = {
            'accelerometer' : [None, None, None],
            'temperature' : None,
            'gyroscope' : [None, None, None],
            'angle_difference' : None
        }

        #temp_read values indexed as read_i2c_block_data returns a list of byte values
        #value reading split from processing/saving due to readability

        temp_read1 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_accel_addrs[0], 2)
        temp_read2 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_accel_addrs[1], 2)
        temp_read3 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_accel_addrs[2], 2)

        #16384 is the scale factor required for the selected sensitivity
        new_reading['accelerometer'][0] = self.twos(256*temp_read1[0] + temp_read1[1]) / 16384 
        new_reading['accelerometer'][1] = self.twos(256*temp_read2[0] + temp_read2[1]) / 16384 
        new_reading['accelerometer'][2] = self.twos(256*temp_read3[0] + temp_read3[1]) / 16384 
        #Scaling & offset are defined in the datasheet
        temp_read1 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_temp_addr, 2)
        new_reading['temperature'] = ((self.twos(256*temp_read1[0] + temp_read1[1]))/340) + 36.53 

        temp_read1 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_gyro_addrs[0], 2)
        temp_read2 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_gyro_addrs[1], 2)
        temp_read3 = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_gyro_addrs[2], 2)

        #131 is the scale factor required for the selected sensitivity
        new_reading['gyroscope'][0] = self.twos(256*temp_read1[0] + temp_read1[1]) / 131
        new_reading['gyroscope'][1] = self.twos(256*temp_read2[0] + temp_read2[1]) / 131
        new_reading['gyroscope'][2] = self.twos(256*temp_read3[0] + temp_read3[1]) / 131

        #Angle difference is the difference between the current angle and that calculated upon initial setup with the set_angle_initial() method.
        #The angle is calculated as the current vector given by the accelerometer, as it is assumed that it will be stationary when zeroed, meaning that
        #gravity will supply an accurate zero vector
        new_reading['angle_difference'] = self.angle_between_vectors(new_reading['accelerometer'], self.angle_initial, degree = True)

        self.history.append(new_reading)

    def form_results(self, results):
        latest = self.get_last_reading()
        results['accel'] = latest['accelerometer']
        results['gyro'] = latest['gyroscope']
        results['temp_integrated'] = latest['temperature']
        results['angle'] = latest['angle_difference']
        return results

    
    def angle_between_vectors(self, vector_1, vector_2, degree = False):
        #Credit to https://stackoverflow.com/a/13849249 for safe implementation of dot product
        vector_1_unit = self.get_unit_vector(vector_1)
        vector_2_unit = self.get_unit_vector(vector_2)
        angle = np.arccos(np.clip(np.dot(vector_1_unit, vector_2_unit), -1.0, 1.0))
        if degree:
            return np.rad2deg(angle)
        else:
            return angle

    def get_unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

class Temperature(Sensor):
    #################################################################################
    # Class adds specific behaviour relevant to the TMP-007 temperature sensor.     #
    # Sensor is relatively simple, therefore only the basic behaviours need to be   #
    # extended from Sensor.                                                         #
    #################################################################################
    def __init__(self, hist = 100):
        super().__init__("Temperature", 0x40)
        self.reg_temp_addr = 0x01 #16 bit register, temperature information in bits 2 to 15, lsb = 0.03125C

    def read(self):
        reading = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_temp_addr, 2)
        processed_reading = ((reading[0]*64) + (reading[1]/4))*0.03125 #Combine readings of each of the registers with appropriate scaling and multiply by the LSB scale
        self.history.append(processed_reading)
    
    def form_results(self, results):
        results['temp_discrete'] = self.get_last_reading()
        return results


class Main:
    #################################################################################
    # Main operational class that is running. When measurements are be taking the   #
    # process goes around the main 'loop' method. Upon first initialisation/        #
    # recieving of a reset command, it goes the 'idle' method until commanded to    #
    # begin normal operations (done, in the demo, with a button connected to a GPIO #
    # interrupt, a commercial product would likely use an RFID signal from a phone, #
    # or similar).                                                                  #
    #################################################################################

    def __init__(self, bus_num = 1):
        #sensors tracks each of the initialized sensor classes
        self.sensors = [] 

        self.bus = smbus.SMBus(bus_num)
        self.initial_idle = True

        #Values for the sensor limits are stored in a text file to allow for concurrency between sessions.
        limit_path = os.getcwd() + '/limits.txt'
        limits = open(limit_path, 'r')
        limit_vals = json.loads(limits.readline())
        limits.close()

        #Violation limits (if values go outside these limits a violation is recorded)
        try:
            #A try/catch is used in the case that there was an error with values being written to the limits.txt file,
            #in which case values default to those in except.
            self.max_angle = limit_vals['Max_Angle'] #degrees
            self.max_discrete_temperature = limit_vals['Max_Discrete_Temperature'] #degrees celcius
            self.min_discrete_temperature = limit_vals['Min_Discrete_Temperature'] #degrees celcius
            self.max_integrated_temperature = limit_vals['Max_Discrete_Temperature'] #degrees celcius
            self.min_integrated_temperature = limit_vals['Min_Discrete_Temperature'] #degrees celcius
            self.max_acceleration = limit_vals['Max_Acceleration'] #g
        except:
            self.max_angle = 30 #degrees
            self.max_discrete_temperature = 30 #degrees celcius
            self.min_discrete_temperature = 0 #degrees celcius
            self.max_integrated_temperature = 30 #degrees celcius
            self.min_integrated_temperature = 0 #degrees celcius
            self.max_acceleration = 1 #g

        #sensor_results tracks the last set of values returned by the sensors
        self.sensor_results = []

        #Any single violation on a sensor is appended to this list and transmitted
        #Each entry is a dictionary with structure: {'timestamp'  : val, 'temp' : val, 'impact' : val, 'angle' : val}
        self.violations = []

        #Try to initialise MQTT connection. If this fails then it will be attempted again in the main loop. 
        try:
            self.mqtt = Client("pi", self, ip = mqtt_IP, port = mqtt_port, encrypt = mqtt_encrypt)
        except Exception as e:
            print(e)
            self.mqtt = None

        #Setup GPIO
        self.gpio = GPIO(self)

    def add_sensor(self, sensor):
        #Adds a sensor to be used, allows for easy addition of extra functionality.
        self.sensors.append(sensor)
        
    def remove_sensor(self, sensor):
        #Removes a sensor from use, would allow for dynamic operation of sensors. This is not used in current implementation.
        if sensor in self.sensors:
            self.sensors.remove(sensor)
        else:
            print("Sensor not currently active")

    def test_violation(self):
        #If any values in the most recent reading exceeds the allowed values then it is recorded here.
        #This is limited as it has the same sample rate as the normal processing, therefore would miss any short events (e.g. the acceleration from being dropped on the floor).
        #The GY-521 sensor includes a register that operates as an interrupt signal for sudden acceleration events, therefore further improvements would make use of this register
        #with a thread running a higher frequency polling signal to detect shorter acceleration events.
        reading = self.sensor_results[-1]
        is_violation = False

        temp_violation = {
            'timestamp' : reading['timestamp'],
            'temp' : reading['temp_discrete'],
            'angle' : reading['angle'],
            'impact' : np.linalg.norm(reading['accel']) - 1 #one is removed from the magnitude as gravity is a constant acceleration being felt.
        }
        
        if temp_violation['temp'] < self.min_discrete_temperature or reading['temp_discrete'] > self.max_discrete_temperature:
            is_violation = True
        if temp_violation['angle']  > self.max_angle:
            is_violation = True
        if temp_violation['impact'] > self.max_acceleration:
            is_violation = True
        if is_violation:
            #Set LEDs to show violation, and append value to the violations list to be included in the next transmission
            self.gpio.gpio_violation()
            self.violations.append(temp_violation)


    def run_readings(self, period):
        #Function runs in a thread to poll the sensors at the set period to get their values.
        #Values are saved to the sensor_results list, and the values are checked to see if they exceed any limits by calling test_violation
        while self.initial_idle == False:
            temp_results = {'timestamp' : str(datetime.utcnow().replace(microsecond=0))}
            for sensor in self.sensors:
                sensor.read()
                temp_results = sensor.form_results(temp_results)

            self.sensor_results.append(temp_results)
            self.test_violation()
            #Time requirement of reading each sensor is minimal, therefore timing drift is not critical
            time.sleep(period)

    def idle(self):
        #Idle loop called upon first initialisation, or upon 'reset' being called from the website. 
        while self.initial_idle:
            self.gpio.gpio_flip()
            time.sleep(0.4)
        self.loop()
            

    def get_json(self):
        #Method ensures that the readings and the violations are combined in the form that the server is expecting.

        json_return = {
            'readings' : self.sensor_results,
            'violations' : self.violations
        }
        #sensor_results and violations must be reset upon the results being formed to avoid data being repeated twice. 
        #A more robust implementation could keep these values stored to allow them to be resent if the connection fails for a period of time 
        self.sensor_results = [] 
        self.violations = []
        return str(json.dumps(json_return))

    def loop(self, period_transmit = 10, period_poll = 1):
        #Main loop of the program, every time it is called it goes through setup of threads and gpio to ensure that the state of the program is zeroed.

        #Ensure that the LED state matches the program state
        self.gpio.gpio_reset() 
        
        #Perform setup of sensors.
        for sensor in self.sensors:
            sensor.setup(self.bus)

        #Starts thread to collect sensor readings. 
        background_readings = threading.Thread(target = self.run_readings, args = (period_poll,))
        background_readings.daemon = True
        background_readings.start()

        #Assuming that the mqtt connection was correctly setup, it is subscribed to the topic that messages from the webserver will be sent from.
        if self.mqtt != None:
            self.mqtt.subscribe(server_pub_topic)

        #Main loop is wrapped in a try/catch to ensure that exit of the program correctly releases control of the GPIO pins.
        try:
            #Actual program loop
            while self.initial_idle == False:
                #Publish the last sensor readings/violations
                if self.mqtt != None:
                    self.mqtt.publish(client_pub_topic,  self.get_json())
                else:
                    try:
                        #If setup failed in the first place, attempt to set it up again.
                        #Note that this isn't automatic reconnect if it disconnects as that is handled within the library
                        self.mqtt = Client("pi", self, ip = mqtt_IP)
                        self.mqtt.connect()
                    except Exception as e:
                        print(e)

                #The sleep function isn't accurate for timekeeping, but the reporting frequency is not of high importance
                time.sleep(period_transmit)
            
            #Ensures that readings stop when a reset command is recieved, otherwise setup of the loop would cause another thread to be started
            background_readings.join()
            
            #Ensures the gpio signals are in a known state and moves to the idle loop
            self.gpio.gpio_reset()
            self.idle()
        except KeyboardInterrupt:
            print("Exiting on keyboard interrupt.")
        finally:
            #Ensures a safe exit. 
            GPIO_module.cleanup()

            
mqtt_IP = 'test.mosquitto.org' #Public server IP
mqtt_port = 8884 #Encrypted server port
mqtt_encrypt = True 

client_pub_topic = 'IC.embedded/jaffa_cakes/comms/pi'
server_pub_topic = 'IC.embedded/jaffa_cakes/comms/server'

if __name__ == "__main__":

    main = Main()

    #Setup both sensors, extra sensors could easily be added at this stage. 
    #Extra sensors must inherit from the Sensor class, but nothing otherwise is required on the client side for basic functionality.
    #More work must be added on the server side to make use of the extra information, but it will not cause an error.
    try:
        temp = Temperature()
        main.add_sensor(temp)
    except Exception as e:
        print("Temperature setup failed:")
        print(e)
        
    try:
        motion = Motion()
        main.add_sensor(motion)
    except Exception as e:
        print("Motion setup failed")
        print(e)
    
    #Move to the idle loop after setup is performed
    main.idle()


