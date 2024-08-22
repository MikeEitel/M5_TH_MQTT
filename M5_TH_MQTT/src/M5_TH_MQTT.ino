// Modified by Mike Eitel to read temperatur and humidity by M5-C3U based board
// All special modified libraries are stored in the via platformio.ini defined libraries directory !!!
// Took some time to get the right versions etc. working flawless, so this unusual approach is choosen deliberately.
// No rights reserved when used non commercial, otherwise contact author.

// M5 pinout
// static const uint8_t G0 /  =  0;    LED 1 und Boot
// static const uint8_t G1 /  =  1;    Switch
// static const uint8_t G2 /  =  2;    Internal WS2812 typ LED = SK6812
// static const uint8_t G3 /  =  3;    Data for MAX6675
// static const uint8_t G4 /  =  4;    Chip select for MX6675
// static const uint8_t G5 /  =  5;    Used for DHT data
// static const uint8_t G6 /  =  6;    Used for powering DHT to have it resetable
// static const uint8_t G7 /  =  7;    Used for DS18B20 x
// static const uint8_t G8 /  =  8;    Has interal -> pullup high = Flash mode
// static const uint8_t G9 /  =  9;    Internal button
// static const uint8_t G10 / = 10;    Clock for MAX6675
// static const uint8_t G18 / = 18;    USB D-
// static const uint8_t G19 / = 19;    USB D+
// static const uint8_t G20 / = 20;    Rx
// static const uint8_t G21 / = 21;    Tx

//#define Rhy      // If defined ( Me .. MeIOT .. LU ..  Rhy ) use private network for testing, otherwise use IOT standard
//#define TEST     // Testmodus

#include <WiFi.h>

#include <PubSubClient.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include <Adafruit_Sensor.h>
#include <DS18B20.h>                // Standard library for DS18B20 x
#include <DHT.h>                    // Standard library for DHT
#include <max6675.h>                // Standart libary for high temp sensor typ K

//define network to connect to  
//#define wifi_ssid "xxx"
//#define wifi_password "xxx"

// This files contain the device definitions for different networks
// In the credential files the fixed IP has to be defined
#if defined(Me)                       
    #include <Me_credentials.h>      
#elif defined(MeIOT)
    #include <MeIOT_credentials.h>     
#elif defined(LU)
    #include <LU_credentials.h>     
#elif defined(Rhy)
    #include <Rhy_credentials.h> 
#else
    #include <credentials.h>         
#endif

// Define used pins
#define RESTART_PIN    9            // Press of on top switch to restart
#define LED_PIN        2            // Use ON BOARD LED
#define DHT_POWER      6            // Digital pin P6 on M5-C3U to supply power to the DHT sensor
#define DHT_PIN        5            // Digital pin P5 on M5-C3U connect data to the DHT sensor
#define DS_PIN         7            // Digital pin P7 on M5-C3U connect data to the DS18B20 sensors

#define SDO            3            // Digital pin P3  on M5-C3U connect data   to the Max6625 chip
#define SCK           10            // Digital pin P4  on M5-C3U connect clock  to the Max6625 chip
#define CS             4            // Digital pin P10 on M5-C3U connect chipselect of the Max6625 chip
                                    // No MISO pin needed

// One wire based temperatur sensors
DHT dht(DHT_PIN,DHTtyp);            // Allows to have different types of DHT's on "first channel"
DS18B20 ds(DS_PIN);                 // Set the "second channel" for pure DS18 devices

// SPI based temperatur sensor on "third channel"; no MISO needed
MAX6675 thermocouple(CS, SDO, SCK); // Software bitbang SPI variable pins when hw pins not changable
//MAX6675 thermocouple(CS, &SPI);                         // HW SPI with above defined pins
#define mySPI_DELAY    6                // Ca. in microseconds 

// Define onboard led
#define LEDS_COUNT     1
#define CHANNEL		     0

Freenove_ESP32_WS2812 led = Freenove_ESP32_WS2812(LEDS_COUNT, LED_PIN, CHANNEL, TYPE_GRB);

int LEDhigh =       255;
int LEDmid  =       128;
int LEDlow  =         0;

// Declare global variables and constants
unsigned long currentMillis;                  // Actual timer 
unsigned long prevRMQTTMillis = 2764472319;   // Stores last MQTT time value was published 2764472319->FASTER START
unsigned long prevSMQTTMillis = 2764472319;   // Stores last MQTT time value was published 2764472319->FASTER START
unsigned long prevMinMillis = 2764472319;     // Stores last minutes time value ->  2764472319->FASTER START
unsigned long prevTickerMillis = 2764472319;  // Stores last ticker time value ->  2764472319->FASTER START
String Sendme;                                // Used for clear text messages in MQTT
String MySensors;                             // A list of usable sensors
int receivedlenght;                           // How long is mqtt message
char lastreceived;                            // Stores the last received status
char receivedChar[10];                        //  = "";
bool received;                                // Actual received status
int watchdogW = 1;                            // Counter if there is no wifi connection
int watchdogM = 1;                            // Counter if there is no MQTT connection
int watchdogS = 1;                            // Counter if there is no DHT sensor readout
int mqttstatus;                               // Helper to see whats going on
bool watchdog = true;                         // Signal via mqtt that device is still ok
bool statusreset = false;                     // Used to minimize error 0 sendouts
int looped = 1;                               // Loop counter as debug helper
bool dhtpresent = true;                       // Checked if DHT is connected
bool toomany = false;                         // Too many DS18 sensors connected
int tstatus = 0;                              // Status of Max6675 readout
int dscounter = 1;                            // Helper to seperate DS18 sensors
int dscounted = 0;                            // Readable value
      int sensors = 0;                              // variable that shows to mqtt the amount of detected sensors
float temp;                                   // Measured temperature DHT 
float mtemp;                                  // Measured temperature Max6625
float dtemp;                                  // Measured temperature DS18
float dtemp1 = -99;                           // Measured temperature 1. DS18
float dtemp2 = -99;                           // Measured temperature 2.
float dtemp3 = -99;                           // Measured temperature 3.
float dtemp4 = -99;                           // Measured temperature 4.
float dtemp5 = -99;                           // Measured temperature 5.
float hum;                                    // Measured humidity

// Setup the background classes  
WiFiClient   espClient;     // Get Wifi access
PubSubClient mqttclient;    // MQTT protokol handler


void setup_wifi() {
  delay(10);

  WiFi.config(staticIP, gateway, subnet);

  Serial.println("");
  Serial.print("Try connect to: ");
  Serial.println(wifi_ssid);
  Serial.print("With IP address: ");
  Serial.println(WiFi.localIP());

  WiFi.begin(wifi_ssid, wifi_password);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  while ((WiFi.status()!= WL_CONNECTED) && (watchdogW <= WiFi_timeout)) {
    led.setLedColorData(0,LEDmid,LEDlow,LEDlow);
    led.show();    
    delay(250);
    led.setLedColorData(0,LEDhigh,LEDlow,LEDlow);
    led.show();
    Serial.print(".");
    Serial.print(watchdogW);
    watchdogW++;
    }
    if (WiFi.status()!= WL_CONNECTED){
      Serial.println ("No connection ");
      Serial.println ("Restart");
      led.setLedColorData(0,LEDhigh,LEDlow,LEDlow);
      led.show();
      delay(2000);
      ESP.restart();
    }
  else {
    Serial.println(" Successfull connected Wifi");
    Serial.print("RSSI: ");   Serial.println(WiFi.RSSI());
    led.setLedColorData(0,LEDmid,LEDmid,LEDlow);
    led.show();
  }
}

void reconnect() {
  delay(100);
  // Loop until we're reconnected to MQTT server
  while (!mqttclient.connected() && (watchdogM <= mqtt_timeout)) {
    mqttclient.clearWriteError();       // Cleaning MQTT write buffer
    mqttclient.flush();                 // Cleaning MQTT data buffer
    mqttstatus = mqttclient.state();    // Decoding of MQTT status 
    Serial.println("");
    Serial.print("Attempting MQTT connection try Nr. ");
    led.setLedColorData(0,LEDlow,LEDlow,LEDhigh);
    led.show();
    Serial.println(watchdogM);
  
    const char *reason;
    switch (mqttstatus) {
      case -4 : {reason = "The server didn't respond within the keepalive time"; break;}
      case -3 : {reason = "The network connection was broken"; break;}
      case -2 : {reason = "The network connection failed"; break;}
      case -1 : {reason = "The client is disconnected cleanly"; break;}
      case  0 : {reason = "The client is connected"; break;}
      case  1 : {reason = "The server doesn't support the requested version of MQTT"; break;}
      case  2 : {reason = "The server rejected the client identifier"; break;}
      case  3 : {reason = "The server was unable to accept the connection"; break;}
      case  4 : {reason = "The username/password were rejected"; break;}
      case  5 : {reason = "The client was not authorized to connect"; break;}
      default: {   }    // Wrong               
    }
    Serial.println(reason);

    if (mqttclient.connect(iamclient, mqtt_user, mqtt_password)) {
      Serial.print("Connected as: ");
      Serial.println(iamclient);
      Serial.println("");
      watchdogM = 1;
      led.setLedColorData(0,LEDlow,LEDhigh,LEDlow);
      led.show();
      // Send status after start
      #if defined(TEST)                               // Send status after start
        mqttclient.publish(out_status, "1" ,false);
      #else
        mqttclient.publish(out_status, "MQTT Connected" ,false);
        delay(500);
      #endif
    }
      Serial.print("RSSI: ");   Serial.println(WiFi.RSSI());
      Serial.println("Retry MQTT in 4 seconds ... ");
      // Wait 4 seconds before retrying
      led.setLedColorData(0,LEDlow,LEDlow,LEDhigh);
      led.show();
      delay(1000);
      led.setLedColorData(0,LEDmid,LEDlow,LEDhigh);
      led.show();
      delay(1000);
      led.setLedColorData(0,LEDlow,LEDlow,LEDhigh);
      led.show();
      delay(1000);
      led.setLedColorData(0,LEDmid,LEDlow,LEDhigh);
      led.show();
      delay(1000);
      watchdogM++;
      Serial.println("_");
    }
   

  if (watchdogM >= mqtt_timeout) {
    Serial.println("");
    Serial.println("NO MQTT available");
    Serial.println("RESTART");
    led.setLedColorData(0,LEDhigh,LEDhigh,LEDhigh);
    led.show();
    delay(5000);
    ESP.restart();                               // REBOOT of the system !!!!!
  }
}

void callback(char* topic, uint8_t* payload, unsigned int length) {
  receivedlenght = length;
  Serial.println("");
  Serial.print("Message for [");
  Serial.print(topic);
  Serial.print("] arrived = L:(");
  Serial.print(length);
  Serial.print(")->");
  for (unsigned int i = 0; i < length; i++) {
    receivedChar[i] = payload[i];    
    Serial.print(receivedChar[i]);                          //     Received from mqtt text
  }
  Serial.println();
  #if defined(TEST)
      mqttclient.publish(out_status, "3" ,false);
    #else
      Sendme = "Command  ";
      Sendme = Sendme + (receivedChar[0]);
      Sendme = Sendme + " received";
      mqttclient.publish(out_status, (String(Sendme).c_str()) ,false);
      statusreset = true;
      delay(1000);
  #endif
 
  switch (receivedChar[0]) {          // Detecition what command is detected 
   case '?' : {                       // request parameters
      Sendme = "RSSI: ";
      Sendme = Sendme + (WiFi.RSSI());
      mqttclient.publish(out_param,   (String(Sendme).c_str()), false);       // Send the wifi signal strenght
      mqttclient.publish(out_sensors, (String(MySensors).c_str()), false);  // Send list of the usable sensors
      readvalues();
      break;
    }
    case 'X' : {                       // restart device
      mqttclient.publish(out_status, "Restart" ,false);
      delay(1000);
      ESP.restart();                   // REMOTE RESTART
      break;
    }
    default: {                        // Wrong command
      #if defined(TEST)
        mqttclient.publish(out_status, "-1" ,false);
      #else
        mqttclient.publish(out_status, "No valid command" ,false);
        statusreset = true;
        delay(1000);
      #endif
      break;
    }         
  }
}

void readvalues() {   
  MySensors = "-";
  #if defined(enableDHT)
    readDHT();                 // Sensor reading DHT
  #endif

  #if defined(enableMAX)
    readMAX();                 // High temp sensor reading
  #endif

  #if defined(enableDS)
    readDS();                  // Sensor reading DS18B20x
  #endif
}

void readDHT(){              // Will read all but only up to 3 DS18B20x sensors are supported 
  dht.begin();
  delay(10);
  temp = dht.readTemperature();

  if (isnan(temp)){
    #if defined(TEST)
      mqttclient.publish(out_status, "-2" ,false);
    #else
      mqttclient.publish(out_status, "DHT Temperatur unavailable" ,false);
    #endif
    statusreset = true;

    digitalWrite(DHT_POWER, LOW);  // Reset DHT via power
    led.setLedColorData(0,LEDlow,LEDmid,LEDlow);
    led.show();
    delay(1000);        // Give sensor time
    led.setLedColorData(0,LEDlow,LEDhigh,LEDlow);
    led.show();
    digitalWrite(DHT_POWER, HIGH);  // Initialize DHT via power
    }
  else{ 
    temp = temp - tempcorr;
    mqttclient.publish(mqtt_out_temp, String(temp).c_str(), false);
    MySensors = MySensors + "DHT_T";

  }

  hum = dht.readHumidity();
  if (isnan(hum)) {
    #if defined(TEST)
      mqttclient.publish(out_status, "-3" ,false);
    #else
      mqttclient.publish(out_status, "DHT Humidity unavailable" ,false);
    #endif
    statusreset = true;
    sensors = 0;
    digitalWrite(DHT_POWER, LOW);  // Reset DHT via power
    led.setLedColorData(0,LEDlow,LEDmid,LEDlow);
    led.show();
    delay(1000);        // Give sensor time
    led.setLedColorData(0,LEDlow,LEDhigh,LEDlow);
    led.show();
    delay(1000);
    digitalWrite(DHT_POWER, HIGH);  // Initialize DHT
    }
  else
    {
    // Correct against sensor selfwarming and impossible humidity > 100%
    hum = hum + humcorr;
    if (hum >= 100){
      hum = 100;
    }
    mqttclient.publish(mqtt_out_hum, String(hum).c_str(), true);
    watchdogS = 1;         // Reset the sensor switching decision
    MySensors = MySensors + "H ";
;
  }

  #if defined(TEST)
    Serial.print(" Readout DHT = ");
    Serial.print(temp);
    Serial.print("C - ");
    Serial.print(hum);
    Serial.println("%");
  #endif
}

void readMAX(){                           // Will read the high temp sensor
  tstatus = thermocouple.read();       // To get a detailed error analysis
  mtemp = thermocouple.getTemperature();  //  readCelsius();

  if ((tstatus != 0)) {
    #if defined(TEST)
      mqttclient.publish(out_status, "-7" ,false);
    #else 
      switch (tstatus) {
        case   4: { 
          mqttclient.publish(out_status, "Thermocouple wiring error", false);
          break;  }
        case 129: { 
          mqttclient.publish(out_status, "Thermocouple no communication", false);
         break;   }
       default : {
          mqttclient.publish(out_status, "K Temp unknown error", false);  //
          break;  } 
      }
    #endif
    statusreset = true;
  }
  else {
    mqttclient.publish(mqtt_out_tempm, String(mtemp).c_str(), false);
    MySensors = MySensors + "& Max6675";
;
    }
    #if defined(TEST)                                   
      Serial.print("  Readout Maxstatus = ");
      Serial.print(thermocouple.getStatus());
      switch (tstatus) {
      case   0: { 
        Serial.print(" -> Value OK");
        Serial.print("  Temp = ");
        Serial.print(mtemp);
        Serial.println("C");
        sensors = sensors + 100;
        break; }
      case   4: { 
        Serial.println(" -> Thermocouple wiring error");
        break; }
      case 128: { 
        Serial.println(" -> No read done yet");
        break; }
      case 129: { 
        Serial.println(" -> No communication");
        break; }
      default : {
        Serial.println(" -> undefined error"); 
        } 
      }
  #endif
}

void readDS() {
  dscounter = ds.getNumberOfDevices();
  dscounted = dscounter;
  #if defined(TEST)
    Serial.print("  Readout ");
    Serial.print(dscounter);
    Serial.println(" DS18 Senors:");
  #endif

  if (dscounter == 0){
    #if defined(TEST)
      mqttclient.publish(out_status, "-4" ,false);
    #else
      mqttclient.publish(out_status, "DS18B20 Temperatur unavailable" ,false);
    #endif
    statusreset = true; }
  else {
    MySensors = MySensors +" & " +dscounted + " DS18B20";
  }

  while (ds.selectNext()){
    dtemp = ds.getTempC();

    #if defined(TEST)
      Serial.print("(");
      Serial.print(dscounter);
      Serial.print(")= ");
      Serial.print(dtemp);
      Serial.print("C - ");
    #else
      Serial.print(".");
    #endif

    if (isnan(dtemp)){
      #if defined(TEST)
        mqttclient.publish(out_status, "-4" ,false);
      #else
        mqttclient.publish(out_status, "DS18B20 Temperatur unavailable" ,false);
      #endif
      statusreset = true;
    }

    // Correct against sensor selfwarming of ca. 2 degrees C and send
    switch (dscounter){
      case 1:{
        dtemp1 = dtemp - tempcorr1;
        mqttclient.publish(mqtt_out_temp1, String(dtemp1).c_str(), false);}
      break;
      case 2:{
        dtemp2 = dtemp - tempcorr2;
        mqttclient.publish(mqtt_out_temp2, String(dtemp2).c_str(), false);}
      break;
      case 3:{
        dtemp3 = dtemp - tempcorr3;
        mqttclient.publish(mqtt_out_temp3, String(dtemp3).c_str(), true);}
      break;
      default:{
        Serial.println("DS18 adressing error. Too many sensors");
        delay(300);
        #if defined(TEST)                           // In this version Not implemented
          mqttclient.publish(out_status, "-5" ,false);
        #else
          mqttclient.publish(out_status, "Switch back to DHT sensor" ,false); 
        #endif
        toomany = true;   
        statusreset = true;
      }
    }
    dscounter--; 

   #if defined(TEST)
      Serial.print("[");
      Serial.print(dscounter);
      Serial.print("]");
    #endif
  }

  #if defined(TEST)
    Serial.println();
  #endif
  watchdogS = 1;
 }


// XXXXXXXXXXXXXXXXXXXXXXXX PROGRAM  START XXXXXXXXXXXXXXXXXXXXXXX

void setup() {
  // Initialize onboard LED and restart button
  led.begin();
  led.setBrightness(LEDbrightness);
  led.setLedColorData(0,LEDmid,LEDmid,LEDlow);
  led.show();
  pinMode(RESTART_PIN, INPUT);

  // Power reset the DHT to conteract startup bug of some devices
  pinMode(DHT_POWER, OUTPUT);
  digitalWrite(DHT_POWER, LOW);

  // Initialize debug output and wifi and preset mqtt
  Serial.begin(115200);
  Serial.println("");
  Serial.println("");
  Serial.print("I am: ");
  Serial.print(iamclient);
  Serial.println("");

  setup_wifi();                     // Start the wifi connection
  delay(100);
  mqttclient.setClient(espClient);
  mqttclient.setServer(mqtt_server, mqtt_port);
  mqttclient.setCallback(callback);
  mqttclient.setKeepAlive(61);      // MQTT_KEEPALIVE : keepAlive interval in seconds. Override setKeepAlive()
  mqttclient.setSocketTimeout(63);  // MQTT_SOCKET_TIMEOUT: socket timeout interval in Seconds. Override setSocketTimeout()
  void reconnect();                 // Start the mqtt connection
  mqttclient.subscribe(in_topic);   // Listen to the mqtt inputs
 
  digitalWrite(DHT_POWER, HIGH);    // Initialize DHT power on
  
  //Start SPI and thermocoupler communication
  //SPI.begin(sck,miso,mosi,ss)
  //SPI.begin(SCK, SDO, -1, CS);
  //SPI.setFrequency(SPI_SPEED);
    void setOffset(float tempcorrm);
  //SPI.setClockDivider(SPI_CLOCK_DIV64);

  thermocouple.begin();
  thermocouple.setSWSPIdelay(mySPI_DELAY);
  thermocouple.setOffset(tempcorrm);
}


//  This is the Main loop
void loop() {                             
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = x milliseconds) it reads a new MQTT message
  if (currentMillis - prevRMQTTMillis >= readinterval) {
    prevRMQTTMillis = currentMillis;

    if (WiFi.status() == WL_CONNECTED) { Serial.print("+");}
    else {
      WiFi.begin(wifi_ssid, wifi_password);
      Serial.println("Try Wifi reconnect "); 
    }
    
    if (!mqttclient.connected()) {
      reconnect();                                // In case no mqtt it will reconnect
      mqttclient.subscribe(in_topic);
      Serial.print("Go in loop with topic: " );
      Serial.println(in_topic);
      #if defined(TEST)
        mqttclient.publish(out_status, "2" ,false);
      #else
        mqttclient.publish(out_status, "Reconnected" ,false);
      #endif
      statusreset = true;
    }
    mqttclient.loop();                            // Request if there is a message
    
    watchdog = !watchdog;                         // Create toggeling watchdog signal
    mqttclient.publish(out_watchdog, String(watchdog).c_str() ,false); 

    if (statusreset){
      statusreset = false;
      delay(1000);
      #if defined(TEST)
        mqttclient.publish(out_status, "0" ,false);
      #else
        mqttclient.publish(out_status, "Normal" ,false);
      #endif
    }
    // Here are the sensordata send via mqtt when exist
    // It is intentional that a send is only done within receive interval. (To avoid unchecked connection abd with buffer problems)
    if (currentMillis - prevSMQTTMillis >= sendinterval) {
      prevSMQTTMillis = currentMillis;
      readvalues();
      Serial.print("--> ");
      Serial.print("T:");  
      Serial.print(temp,1);
      Serial.print("C _ H:");  
      Serial.print(hum,1);
      Serial.print("% __ ");
      Serial.print(mtemp,0);
      Serial.print("C __ ");
      Serial.print(dtemp1,1);
      Serial.print("C _ ");
      Serial.print(dtemp2,1);
      Serial.print("C _ ");
      Serial.print(dtemp3,1);
      Serial.println("C");
      Serial.println(""); 
    }
  }

  #if defined(TEST)
  // mqttclient.publish(out_topic, String(looped).c_str() ,false); // Only use that when in doubts of loop
    looped++;
  #endif

  if (!digitalRead(RESTART_PIN)){ESP.restart();}
  delay(50);
}