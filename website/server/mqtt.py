from django.apps import apps
import paho.mqtt.client as mqtt
from threading import Thread
import json, sys, os
class listener_client():

    #################################################################################
    # listener_client is only able to subscribe to messages from the designated     #
    # topics. It is designed to run in a thread in the server.                      #
    #################################################################################

    def __init__(self, name, main = None, port = 1883, ip = '127.0.0.1', encrypt = False):
        #Defaults to localserver connection
        self.client = mqtt.Client(name)
        self.name = name
        self.ip = ip
        self.port = port

        if main != None:
            self.main = main

        if encrypt:
            #The keys/certificates must be in the same directory as this file.
            cwd = os.getcwd()
            BROKER_CERT = cwd + r'\server\mosquitto.crt'
            CLIENT_CERT = cwd + r'\server\client.crt'
            PRIVATE_KEY = cwd + r'\server\client.key'
            self.client.tls_set(ca_certs = BROKER_CERT, certfile = CLIENT_CERT, keyfile = PRIVATE_KEY)

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
    
        self.connect()

    def connect(self):
        print("mqtt connect")
        t = Thread(target = self.client.connect, args = (self.ip, self.port,))
        t.daemon = True
        t.start()
        t.join(5)
        if t.is_alive():
            raise Exception("Timeout when trying to connect to {0} on port {1}".format(self.ip, self.port))
        else:
            self.connected = True
        print(self.connected)
        self.client.loop_start()
    
    def subscribe(self, topic):
        self.client.subscribe(topic)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

    def on_message(self, client, userdata, message):
        try:
            data = str(message.payload.decode("utf-8"))
            data = json.loads(data)
            self.parse_message(data)
        except Exception as e:
            print(self.name + ": " + e)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection")
        self.connected = False

    def on_server_stop(self, *args):
        self.client.loop_stop()
        sys.exit(0)

    def parse_message(self, message):
        sensor_values = apps.get_model(app_label = 'server', model_name='sensor_values')
        violation_values = apps.get_model(app_label = 'server', model_name='violation_values')
        """
        Expected message format:
        message = {
            'readings' : [
                {
                    'timestamp' : val,
                    'temp_discrete' : val,
                    'accel' : [x,y,z],
                    'gyro' : [x,y,z],
                    'temp_integrated' : val,
                    'angle' : val
                    
                },
                ...
                #any number of messages (10 maybe?)
            ]
            'violations' : [
                {
                    'timestamp'  : val,
                    'temp' : val,
                    'impact' : val,
                    'angle' : val
                },
                ...
                #any number of violations
            ]
            #violations are separate to the message as they can be asyncronous to the sampled values
            #e.g. a polling thread can be left to detect sudden changes in acceleration, can't reliably be detected by normal sampling as the period is too long
        }
        """
        
        readings = message['readings']
        violations = message['violations']

        for reading in readings:
            new_sensor_values = sensor_values(
                time = reading['timestamp'],
                temp_discrete = reading['temp_discrete'],
                temp_integrated = reading['temp_integrated'],
                gyro_x = reading['gyro'][0],
                gyro_y = reading['gyro'][1],
                gyro_z = reading['gyro'][2],
                accel_x = reading['accel'][0],
                accel_y = reading['accel'][1],
                accel_z = reading['accel'][2],
                angle = reading['angle']
            )
            new_sensor_values.save()

        for violation in violations:
            new_violation_values = violation_values(
                time = violation['timestamp'],
                angle = violation['angle'],
                accel = violation['impact'],
                temp = violation['temp']
            )
            new_violation_values.save()
        

class publish_client():
    #################################################################################
    # publish_client is only able to publish messages to the designated broker,     #
    # otherwise it is similar to all other MQTT client classes                      #
    #################################################################################
    def __init__(self, name, port = 1883, ip = '127.0.0.1', encrypt = False):
        #Defaults to localserver connection
        self.client = mqtt.Client(name)
        self.name = name
        self.ip = ip
        self.port = port

        if encrypt:
            cwd = os.getcwd()
            BROKER_CERT = cwd + r'\server\mosquitto.crt'
            CLIENT_CERT = cwd + r'\server\client.crt'
            PRIVATE_KEY = cwd + r'\server\client.key'
            self.client.tls_set(ca_certs = BROKER_CERT, certfile = CLIENT_CERT, keyfile = PRIVATE_KEY)

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
    
        self.connect()

    def connect(self):
        print("mqtt connect")
        t = Thread(target = self.client.connect, args = (self.ip, self.port,))
        t.daemon = True
        t.start()
        t.join(5)
        if t.is_alive():
            raise Exception("Timeout when trying to connect to {0} on port {1}".format(self.ip, self.port))
        else:
            self.connected = True
        print(self.connected)
        self.client.loop_start()

    def publish(self, topic, message):
        print('publishing')
        self.client.publish(topic, message)
    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection")
        self.connected = False

    def close_connection(self, *args):
        self.client.loop_stop()
