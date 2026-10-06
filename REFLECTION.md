# Reflection : SIS401 Programming Assignment 1

**Student:** GROUP 1

In general, the entire system worked properly as it was supposed to do. LDR was in working condition right from the very beginning and after correcting all wired connections, DHT22 ADC and OLED successfully produced readings while sending data to Mosquitto broker every 5 seconds in a consistent fashion. The fact that the same readings were obtained on OLED, Serial Monitor and in mosquitto_sub indicated that all components worked very well together.

The first problem we faced was connected with Mosquitto broker. My computer (Mac) reported that There is a problem with Mosquitto, which claimed that it is unable to open configuration file. The reason for this was that mosquitto.conf file was missing from the folder that Mosquitto was running in. Therefore, we created a configuration file in the home directory with listener 1883 and allow-anonymous true configurations.

Secondly, ESP32 module failed to stay connected to the Wi-Fi and was sending an error message "sta is connecting, return error". We implemented Wi-Fi scan function at startup and the connection status messages to inform users about the network issues and reduced the frequency of reconnection attempts so that they would not interfere with the ongoing connection process.

Every read from the DHT22 resulted in a `nan` error code. The issue turned out to be a wiring issue, leading to the decision of adding a failure counter and hints for troubleshooting wiring problems into the firmware.

The most difficult issue was experienced with the MPU6050. Though the power LED was on, the OLED continued to display "IMU: not found". We found out that the GY-521 board has its pins mapped with SCL appearing before the SDA pin, as well as that many modules are built using clone chips that are not accepted by the Adafruit library. Instead of using the library, we added a layer for direct register access and incorporated an automatic retry mechanism so that the IMU will connect automatically once the wiring is done correctly.

The main lesson was that diagnostics save time. Building the firmware to report its own problems made each fault much faster to find than guessing.
