# M5_TH_MQTT
M5stack based device to measure temperature and humidity with DHT22, DS18B20, MAX6675.

This version is based on the M5stack-3CU device.
It allows to measure DHT22, DS18B20 and thermocoppler via MAX6675 chip.
The results are transfered via mqtt.

In the credentials.h  you define what sensoric to use plus address and network / mqtt parameters.. 
This device can read all sensors in the same device at the same time. The number of DS18B20 is restricted to 3.

As these devices are used also in mulziple networks in the .ino it is defined whitch credential file for a network is to use.

I dislike DHCP in static IOT networks and the last IP digit is also used to define the mqtt name. Easier for debugging. 
So in a correct credential file only the one number in   #define myIP "100"   is enough to compile another device.
