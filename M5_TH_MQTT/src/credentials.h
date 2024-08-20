/*Made by Mike Eitel to store device credenitals on one place
  No rights reserved when private use. Otherwise contact author.

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
  and associated documentation files. The above copyright notice and this permission notice shall 
  be included in all copies or substantial portions of the software.
*/

// Wifi acess definitions of network to connect to  
#define wifi_ssid "xxx"
#define wifi_password "yyy"

#define myIP "100"                                    // Device ID

// Wifi definitions of this device  
//IPAddress staticIP(192,168,x,atoi(myIP));           // IOT device IP
IPAddress staticIP(1,1,1,atoi(myIP));      // REPLACE !!!!
IPAddress subnet(255,255,255,0);                      // Network subnet size
//IPAddress gateway(192,168,x,y);                     // Network router IP
IPAddress gateway(1,1,1,1);                // REPLACE !!!!

// Mosquitto MQTT Broker definitions
#define mqtt_server    "192.168.x.z"                  // IOT MQTT server IP
#define mqtt_user      "admin"
#define mqtt_password  "admin"
#define mqtt_port      1883
#define WiFi_timeout    101                           // How many times to try before give up
#define mqtt_timeout     11                           // How many times to try before try Wifi reconnect


// MQTT Topics
#define mytype          "esp/M5-TH-"                    // Client Typ
#define iamclient       mytype myIP                     // Client name 
#define in_topic        iamclient "/command"            // This common input is received from MQTT
#define out_param       iamclient "/signal"             // Wifi signal strength is send to MQTT
#define out_status      iamclient "/status"             // This is a general message send to MQTT
#define out_watchdog    iamclient "/watchdog"           // A watchdog bit send to MQTT
#define mqtt_out_hum    iamclient "/ext_humidity/0"     // This is send to MQTT
#define mqtt_out_temp   iamclient "/ext_temperature/0"  // This is send to MQTT
#define mqtt_out_tempm  iamclient "/ext_temperature/1"  // This is send to MQTT
#define mqtt_out_temp1  iamclient "/ext_temperature/2"  // This is send to MQTT
#define mqtt_out_temp2  iamclient "/ext_temperature/3"  // This is send to MQTT
#define mqtt_out_temp3  iamclient "/ext_temperature/4"  // This is send to MQTT
#define out_topic       iamclient "/loop"               // This is send to MQTT
#define out_sensors     iamclient "/sensors"            // This is a list of usable external sensors send to MQTT
#define out_topic       iamclient "/loop"               // This helper debug variable can be send to MQTT

float humcorr =             0.0;    // Humidity sensor error correction surplus
float tempcorr =            0.0;    // Temperatur 0 DHT sensor error correction surplus
float tempcorrm =           0.0;    // Temperatur 1 Max6675 sensor error correction
float tempcorr1 =           0.0;    // Temperatur 2 DS sensor 1 error correction surplus
float tempcorr2 =           0.0;    // Temperatur 3 DS sensor 2 error correction surplus
float tempcorr3 =           0.0;    // Temperatur 4 DS sensor 3 error correction surplus
// Errors send as values in test mode

// error =  -7    Max 6675 not found
// error =  -6    Switched DHT to DS18B20
// error =  -5    Switched from DS18B20 to DHT
// error =  -4    DS18B20 Humidity sensor not read
// error =  -3    DHT Humidity sensor not read
// error =  -2    DHT Temperatur sensor not read
// error =  -1    Wrong command received
// error =   0    Normal status
// error =   1    First time connected
// error =   2    Reconnect succesfull
// error =   3    Not implemented

int LEDbrightness =          96;          // Brightness of the modules buildin led

// Allowed are:  DHT 11  or   DHT 12  or  DHT 22 (AM2302), AM2321   or   DHT 21 (AM2301)
#define DHTtyp   DHT22                    // See above

#define enableDHT                         // Enable DHT sensor      
#define enableMAX                         // Enable Max6675 thermocouple sensor  
#define enableDS                          // Enable DS18B20x sensors  

// Constant how often the mqtt message is send
#if defined(TEST)
  const long readinterval =  1000;        // Interval at which to publish sensor readings
  const long sendinterval =  3000;        // Interval at which sensor data is send via mqtt
#else
  const long readinterval =   5000;       // Interval at which to publish sensor readings
  const long sendinterval =  60000;       // Interval at which sensor data is send via mqtt
#endif
