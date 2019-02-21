Embedded Systems Final Submission
=================================
## Notes
* The presentation website has a demonstration of the look of the data presentation page. The data is hardcoded as a demonstration.
* The server and pi code contained in the pi and website subdirectories require the SSL key/certificate as well as the broker certificate in the same directory as the `mqtt.py` file (for both). This has been removed for security reasons. 
* The code for the presentation website can be found in the `marketingwebsite` directory, the live site can also be found at embedded-site.edward-stables.com.

## Theory Of Operation
The main operation for the Pi is the `Main` class in `main.py`. In the same file are the classes for each sensor. Each sensor inherits from the `Sensor` class contained in the `util.py` file. The `Sensor` class enforces certain behaviours on the sensor and allows a new sensor to be easily added without altering large parts of the code. Also contained in the `util.py` file is the `GPIO` class which abstracts the exact GPIO controls away from the Main class, both for protection and code neatness. The final file is `mqtt.py` which contains all the required mqtt behaviour, including parsing new information. This is abstracted out of the `Main` class for the same reason as the `GPIO` class. 

Upon launch the program intialises all sensors, attempts to connect to the broker, and then stays in an idle loop until given an external signal (currently a GPIO interrupt from a button, further development would change this to an RFID message from a phone). Upon exiting the idle loop, the pi takes a zero reading of the angle (an average of the current acceleration due to gravity being felt) and enters the main loop. Before the main loop starts, a thread is started to make sensor readings at a defined polling rate. Finally the actual main loop starts, which reports the last set of readings at a preset frequency. The default periods for sampling and reporting are 1s and 10s respectively, giving 10 samples per report.

After each sample reading the program checks to see if the values have exceeded any set limits,and if so, saving the values of the sensors and reporting them separately. If this occurs, the external red LED will turn on to indicate the violation (but functionality continues as normal).

Signals can be recieved from the web app as well as sent. All signals limits can be set, and the program can be reset (turning the red LED off and putting it back in the idle loop, allowing the angle to be re-zeroed). All limits are saved in a text file upon being recieved, and read into the program upon first launch. This allows for persistence in the limits between uses. 

All mqtt communications are encrypted, and the library has been set to attempt to reconnect to the broker if connection is lost. If connection couldn't be made upon first launch, it is retried on every repition of the main loop. Further development would include saving data while there is no connection and sending a larger burst after connection is reestablished. 
 

