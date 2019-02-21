import paho.mqtt.client as mqtt
from threading import Thread
import json
import os

    
class Client():
    #################################################################################
    # Abstracts mqtt functionality away from the main loop, and handles recieving   #
    # and transmitting all data.                                                    #
    #################################################################################
    def __init__(self, name, main, port = 1883, ip = '127.0.0.1', encrypt = False):
        #Broker address defaults to localhost on port 1883
        self.client = mqtt.Client(name)
        self.connected = False

        self.ip = ip
        self.port = port

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
    
        self.main = main

        if encrypt:
            #The keys/certificates must be in the same directory as this file.
            BROKER_CERT = 'mosquitto.crt'
            CLIENT_CERT = 'client.crt'
            PRIVATE_KEY = 'client.key'
            self.client.tls_set(ca_certs = BROKER_CERT, certfile = CLIENT_CERT, keyfile = PRIVATE_KEY)

        self.connect()
    
    def connect(self):
        #Threading in this function is used to apply a timeout functionality when trying to connect. 
        #This allows for the rest of the program to operate as normal and not crash if there is no 
        #connection. Connection will be tried again later. 
        t = Thread(target = self.client.connect, args = (self.ip, self.port,))
        t.daemon = True
        t.start()
        t.join(5)
        if t.is_alive():
            print("Connection timeout")
            raise Exception("Timeout when trying to connect to {0} on port {1}".format(self.ip, self.port))
        else:
            self.connected = True

        self.client.loop_start()

    def subscribe(self, topic):
        self.client.subscribe(topic)
    
    def publish(self, topic, message):
        self.client.publish(topic, message)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

    def on_message(self, client, userdata, message):
        #Recieved message is parsed in the try/catch to ensure that it meets the required formatting. 
        try:
            data = str(message.payload.decode("utf-8"))
            data = json.loads(data)
            print(data)
            self.parse_message(data)

        except Exception as e:
            print("Couldn't decode message:")
            print(e)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection")
        self.connected = False

    def parse_message(self, data):
        #Received data is processed to update data in the main thread. 
        if 'Reset' in data:
            self.main.initial_idle = data['Reset']
        if 'Max_Angle' in data:
            self.main.max_angle = data['Max_Angle']
        if 'Max_Discrete_Temperature' in data:
            self.main.max_discrete_temperature = data['Max_Discrete_Temperature']
        if 'Min_Discrete_Temperature' in data:
            self.main.min_discrete_temperature = data['Min_Discrete_Temperature']
        if 'Max_Integrated_Temperature' in data:
            self.main.max_integrated_temperature = data['Max_Integrated_Temperature']
        if 'Min_Integrated_Temperature' in data:
            self.main.min_integrated_temperature = data['Min_Integrated_Temperature']
        if 'Max_Acceleration' in data:
            self.main.max_acceleration = data['Max_Acceleration']
        
        data.pop('Reset')
        #new violation limits are saved in a text file so that they can be read and used on next boot
        limit_path = os.getcwd() + '/limits.txt'
        limit_file = open(limit_path, 'w')
        limit_file.write(json.dumps(data))
        limit_file.close()
