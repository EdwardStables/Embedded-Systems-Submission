from django.apps import AppConfig
from .mqtt import listener_client
import signal 

class ServerConfig(AppConfig):
    name = 'server'

    def ready(self):

        #########################################################################################################################
        # When using the development server the --noreload option must be selected for the MQTT client to be launched correctly #
        #########################################################################################################################
        print("ready")
        client_IP = 'test.mosquitto.org'
        client_Port = 8884
        name = 'Update_Listener'
        client = listener_client(name = name,  port = client_Port, ip = client_IP, encrypt=True)

        signal.signal(signal.SIGINT, client.on_server_stop)
        
        client.subscribe("IC.embedded/jaffa_cakes/comms/pi")

