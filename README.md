# solar-panel-aquisition-card-demo
This project is a demo of an aquisition card which will collect the information about the current, voltage, humidity and temperature of a solar panel then, the data will be organized into packets and sent to a central server via Wi-Fi to store it in the database to use it later.

The current data will be collected using ACS712 sensor.
The voltage data will be collected using Arduino Sensor DC Voltage.
The temperature and humidity data will be collected using DHT11 sensor.

The data will be sent via Wi-Fi through ESP8266 module.
The main processing, data collection and interacting with ESP8266 will be done using Atmega32 micro controller.

Next we show the schematic of the circuit:
![Solar Pannel Card](https://user-images.githubusercontent.com/110384824/207528741-4acc0cbd-9e07-4483-ac8e-0ce912aa0f03.jpg)

![Solar Pannel Card2](https://user-images.githubusercontent.com/110384824/207528781-70f15611-213e-473a-aca8-f04e4ea0db5b.jpg)
