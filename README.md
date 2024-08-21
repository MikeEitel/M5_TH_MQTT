# M5_TH_MQTT
M5stack based device to measure temperature and humidity with DHT22, DS18B20, MAX6675.

This version is based on the M5stack-3CU device.
It allows to measure DHT22, DS18B20 and thermocoppler via MAX6675 chip.
The results are transfered via mqtt.

In the credentials.h  you define what sensoric to use. They can read all in the same device at the same time. The number of DS18B20 is restricted to 3.
As this is used in some different networks in the .ino it is defined whitch credential file to use.
I dislike DHCP in static IOT networks. So the last IP digit is also used to define the mqtt name.
So in a correct credential file only the one number in   #define myIP "100"   is enough to compile another device.
