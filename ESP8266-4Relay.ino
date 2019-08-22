
// This code has used code from www.github/computourist as a basis.
// 
//  added OTA, adapted it for use on a Wemos mini and later cut it down to use on an ESP8266-01
//
//  The MQTT topic is home/direction/nodeid/devid
//  where direction is "sb" towards the node and "nb" towards the MQTT broker.
//  So for example: home/sb/node01/dev10

//  Reserved ranges for node devices, as implemented in the gateway are:
//  0  - 15       Node system device
//  16 - 32       Binary output (LED, relay)
//  32 - 40       Integer output (pwm, dimmer)
//  40 - 48       Binary input (button, switch, PIR-sensor)
//  48 - 64       Real input (temperature, humidity)
//  64 - 72       Integer input (light intensity, slider setting)
//
//  72  string:     transparant string transport
//  73 - 90   Special devices not implemented yet)
// **********************************************************
//  Defined devices are:
//  dev00 uptime:   read uptime in minutes
//  dev01 interval: read/set transmission interval for push messages
//  dev02 RSSI:   read radio signal strength
//  dev03 version:  read software version
//  dev05 ACK:    read/set acknowledge message after a 'set' request
//  dev06 toggle:   read/set select toggle / timer function
//  dev07 timer:    read/set timer interval in seconds
//  dev08
//  dev09
//  dev10  IP:     Read IP address
//  dev11  SSID: Read SSID
//  dev12  PW: Read Password
//  dev13 
//  dev16  read/set relay1 output
//  dev17  read set relay2 output
//  dev18  read set relay3 output
//  dev19  read set relay4 output
//  dev40  button    tx only: button pressed
//  dev92  error:    tx only: device not supported
//  dev91  error:    tx only: syntax error
//  dev99  wakeup:   tx only: first message sent on node startup
//
//  Hardware connections as in Sonoff device:
//  - pin 0 is connected to a button that switches to GND, pullup to VCC, also used to enter memory flash mode.
//  - pin 13 is connected to LED and current limiting resistor to VCC, used to indicate MQTT link status
//  - pin 12 is connected to the relais output
//
//  version 1.0 by computourist@gmail.com June 2016
//  version 1.1 by computourist@gmail.com July 2016
//    specific client name to avoid conflicts when using multiple devices.
//    added mqtt will to indicate disconnection of link
//  version 1.2 by computourist@gmail.com July 2016
//    fallback SSI and autorecovery for wifi connection.
//    a connection attempt to each SSI is made during 20 seconds .

//
// https://arduinodiy.wordpress.com/2018/06/14/simple-wifi-relay-board-or-diy-sonoff-sv-sonoff-4-channel-2/

/*
 * Listen : 
  mosquitto_sub -h mqtt.zalin.home -t home/#
 * Publish :  
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev16' -m ON -u MqttUser -P <password>
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev17' -m ON -u MqttUser -P <password>
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev18' -m ON -u MqttUser -P <password>
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev19' -m ON -u MqttUser -P <password>

 */
/*
 * Par defaut le port son tous ouverts.
 * 
 */

#include "userdata.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>        // Include the mDNS library
#include "PubSubClient.h"
#define DEBUG                 // uncomment for debugging  
#define ACT1 12                 // Actuator pin (LED or relay to ground)  //D6 on Wemos unnecessary
#define MQCON 13                // MQTT link indicator D7 op Wemos unnecessary
//#define BTN 15                 // Button pin (also used for Flash setting)  D3 op Wemos  unnecessary
#define SERIAL_BAUD 115200
#define HOLDOFF 1000  // blocking period between button messages
#define RELTemporisation  10000 // 1 minutes = 60000ms entre 2 action sur une valve

/*
   GPIo05  D1-->IN1  DID16  ok
   GPIo04  D2-->IN2  DID17  ok
   GPIo14  D5-->IN3  DID18  
   GPIo12  D6-->IN4  DID19  ok
   GPIo13  D7-->BTN  DID40  
*/

#define BTN 13
#define REL1 5
#define REL2 4
#define REL3 14
#define REL4 12

//  STARTUP DEFAULTS

long  TXinterval = 60;            // periodic transmission interval in seconds
long  TIMinterval = 120;           // timer interval in seconds
int  ackButton ;            // flag for message on button press
bool  toggleOnButton = true;          // toggle output on button press
bool  fallBackSsi = false;          // toggle access point

//  VARIABLES

int   DID;                  // Device ID
int   error;                  // Syntax error code
long  lastPeriod = -1;            // timestamp last transmission
long  lastBtnPress = -1;            // timestamp last buttonpress
long  lastMinute = -1;            // timestamp last minute
long  upTime = 0;               // uptime in minutes
int   ACT1State;                // status ACT1 output
bool  RELStateArray[5];
bool  mqttNotCon = true;            // MQTT broker not connected flag
int   signalStrength;             // radio signal strength
bool  wakeUp = true;              // wakeup indicator
bool  setAck = true;             // acknowledge receipt of actions
bool  curState = true;            // current button state
bool  lastState = curState;           // last button state
bool  timerOnButton = false;          // timer output on button press
bool  msgBlock = false;           // flag to hold button message
bool  readAction;               // indicates read / set a value
bool sendArray[100]; // message triggers
String  IP;                   // IPaddress of ESP
char  buff_topic[30];             // mqtt topic
char  buff_msg[32];             // mqtt message
char  clientName[10];             // Mqtt client name
char  wifi_ssid[20];                // Wifi SSID name
char  wifi_password[20];              // Wifi password

//===============================================================================================

// Relay Configuration
#include "Relay.h"

Relay relayStruct[5] = {
  Relay(0,RELTemporisation),
  Relay(REL1,RELTemporisation),
  Relay(REL2,RELTemporisation),
  Relay(REL3,RELTemporisation),
  Relay(REL4,RELTemporisation)
};

// wifi configuration
//
WiFiClient espClient;

// OTA configuration
//
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* host = HOST;
// function prototypes required by Arduino IDE 1.6.7
void setupOTA(void);

// MQTT defintion
//
void mqttSubs(char* topic, byte* payload, unsigned int length);
PubSubClient client(mqtt_server, mqttPort, mqttSubs, espClient); // instantiate MQTT client

// Button Management
//
#include <Bounce2.h>
Bounce debouncer = Bounce(); // Instantiate a Bounce object

//===============================================================================================

void pubMQTT(String topic, String topic_val) { // publish MQTT message to broker
#ifdef DEBUG
  Serial.print("topic " + topic + " value:");
  Serial.println(String(topic_val).c_str());
#endif
  client.publish(topic.c_str(), String(topic_val).c_str(), true);
}

void mqttSubs(char* topic, byte* payload, unsigned int length) {  // receive and handle MQTT messages
  int i;
  error = 4;                    // assume invalid device until proven otherwise
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif
  if (strlen(topic) == 20) {            // correct topic length ?  was 27
    DID = (topic[18] - '0') * 10 + topic[19] - '0'; // extract device ID from MQTT topic  was 25 en 26
    payload[length] = '\0';           // terminate string with '0'
    String strPayload = String((char*)payload); // convert to string
    readAction = (strPayload == "READ");      // 'READ' or 'SET' value
    if (length == 0) {
      error = 2; // no payload sent
    }
    else {
      if (DID == 0) {               // uptime
        if (readAction) {
          sendArray[DID] = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 1) {             // transmission interval
        error = 0;
        if (readAction) {
          sendArray[DID] = true;
        } else {                // set transmission interval
          TXinterval = strPayload.toInt();
          if (TXinterval < 10 && TXinterval != 0) TXinterval = 10; // minimum interval is 10 seconds
        }
      }
      if (DID == 2 || DID == 3){               // RSSI // version
        if (readAction) {
          sendArray[DID] = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 5 || DID == 6) {             // ACK // toggle / timer mode selection
        if (readAction) {
          sendArray[DID] = true;
          error = 0;
        } else if (strPayload == "ON") {      // select toggle mode
          setAck = true;
          if (setAck) sendArray[DID] = true;
          error = 0;
        } else if (strPayload == "OFF") {   // select timer mode
          setAck = false;
          if (setAck) sendArray[DID] = true;
          error = 0;
        } else error = 3;
      }

      if (DID == 7) {             // Timer interval
        error = 0;
        if (readAction) {
          sendArray[DID] = true;
        } else {                // set timer interval
          TIMinterval = strPayload.toInt();
          if (TIMinterval < 60 && TIMinterval != 0) TIMinterval = 60; // minimum interval is 60 seconds
        }
      }
      //------------------------------------------
      if (DID == 8) {             // Timer interval
        error = 0;
        if (readAction) {
          sendArray[DID] = true;
        } else {                // set timer interval
          //TIMShower = strPayload.toInt();
          //if (TIMShower <3&& TIMShower !=0) TIMShower = 3; // minimum interval is 3 min
          //Serial.print(TIMShower);
        }
      }
      //--------------------------------------------------
      if (DID == 9) {             // Timer interval
        error = 0;
        if (readAction) {
          sendArray[DID] = true;
        } else {                // set timer interval
          // TIMShower2 = strPayload.toInt();
          // Serial.print(TIMShower2);
        }
      }
      //---------------------------------------
      //ip
      if (DID >= 10 && DID <=12) {               // IP //SSID        //PW
        if (readAction) {
          sendArray[DID] = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }

      //--------------------
       if (DID == 13) {             // Timer interval
        error = 0;
        if (readAction) {
          sendArray[13] = true;
        } else {                // set timer interval
          // moistTimer = strPayload.toInt();
          // Serial.print(moistTimer);
        }
      }
      //-------------------
      // Relay 1 to 4
      if (DID >= 16 && DID <= 19) {              // state of Relay1 GPIO1 TX
        if (readAction) {
          //Serial.end();
          sendArray[DID] = true;
          error = 0;
        } else if (strPayload == "ON") {
          relayStruct[DID-15].close();
          if (setAck) sendArray[DID] = true;
          error = 0;
        } else if (strPayload == "OFF") {
          relayStruct[DID-15].open();
          if (setAck) sendArray[DID] = true;
          error = 0;
        } else error = 3;
      }
      //-----------------------------

      if (DID == 46) { //coop door
        if (readAction) {
          sendArray[46] = true;
          error = 0;
        } else error = 3;
      }

      //-----------------------------------------
      if (DID == 49 || DID == 51 || DID == 52 || DID == 53 || DID == 55 || DID == 59 || DID == 60) {   //49 batterij //51 DHT11 //52 BH1750 //53 pcf8591 // 55 ds18b20
        if (readAction) {
          sendArray[DID] = true;
          error = 0;
        } else error = 3;
      }

    }
  } else error = 1;
  if (error != 0) {             // send error message
    sprintf(buff_topic, "home/nb/node%02d/dev91", nodeId);
    sprintf(buff_msg, "syntax error %d", error);
    pubMQTT(buff_topic, buff_msg);
  }
}

void connectMqtt() {                // reconnect to mqtt broker
  sprintf(buff_topic, "home/sb/node%02d/#", nodeId);
  sprintf(clientName, "ESP_%02d", nodeId);
  if (!client.loop()) {
    mqttNotCon = true;              // LED off means high voltage
#ifdef DEBUG
    Serial.print("Connect to MQTT broker...");
#endif
    if (client.connect(clientName, mqtt_user, mqtt_password, "home/disconnected", 0, true, clientName)) {
      mqttNotCon = false;           // LED on means low voltage
      client.subscribe(buff_topic);

#ifdef DEBUG
      Serial.println("connected");
#endif
    } else {
#ifdef DEBUG
      Serial.println("Failed, try again in 5 seconds");
#endif
      delay(5000);
    }
    digitalWrite(MQCON, mqttNotCon);      // adjust MQTT link indicator
  }
}

void connectWifi() {                // reconnect to Wifi
  while (WiFi.status() != WL_CONNECTED ) {
    int i = 0;
    digitalWrite(MQCON, HIGH);            // turn off MQTT link indicator
    if (fallBackSsi)                  // select main or fallback access point
    { strcpy(wifi_ssid, wifi_ssid_B);
      strcpy(wifi_password, wifi_password_B);
    }
    else
    { strcpy(wifi_ssid, wifi_ssid_A);
      strcpy(wifi_password, wifi_password_A);
    }
#ifdef DEBUG
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);
#endif
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    while ((WiFi.status() != WL_CONNECTED) && (i < 20)) {
      delay(1000);
      i++;
#ifdef DEBUG
      //Serial.print(".");
#endif
    }
    fallBackSsi = !fallBackSsi;             // toggle access point
  }
  digitalWrite(MQCON, LOW);             // turn on MQTT link indicator
  delay(1000);                    // for 1 second to indicate WIFI connection
  digitalWrite(MQCON, HIGH);            // turn off MQTT link indicator
  delay(1000);

    if (!MDNS.begin(host)) {             // Start the mDNS responder for esp8266.local
#ifdef DEBUG
    Serial.println("Error setting up MDNS responder!");
#endif

  }
  IP = WiFi.localIP().toString();
  
#ifdef DEBUG
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(IP);
  Serial.println("mDNS responder started");
#endif
}

void sendMsg() {                // send any outstanding messages
  int i;
  if (wakeUp) {                 // send wakeup message
    wakeUp = false;
    sprintf(buff_topic, "home/nb/node%02d/dev99", nodeId);
    sprintf(buff_msg, "NODE %d WAKEUP: %s",  nodeId, clientName);
    sendArray[99] = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[0]) {                  // send uptime
    sprintf(buff_topic, "home/nb/node%02d/dev00", nodeId);
    sprintf(buff_msg, "%d", upTime);
    sendArray[0] = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[1]) {                  // send transmission interval
    sprintf(buff_topic, "home/nb/node%02d/dev01", nodeId);
    sprintf(buff_msg, "%d", TXinterval);
    sendArray[1] = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[2]) {                  // send transmission interval
    sprintf(buff_topic, "home/nb/node%02d/dev02", nodeId);
    signalStrength = WiFi.RSSI();
    sprintf(buff_msg, "%d", signalStrength);
    sendArray[2] = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[3]) {                  // send software version
    sprintf(buff_topic, "home/nb/node%02d/dev03", nodeId);
    for (i = 0; i < sizeof(VERSION); i++) {
      buff_msg[i] = VERSION[i];
    }
    buff_msg[i] = '\0';
    sendArray[3] = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[5]) {                  // send ACK state
    sprintf(buff_topic, "home/nb/node%02d/dev05", nodeId);
    if (!setAck) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    sendArray[5] = false;
  }

  if (sendArray[6]) {                  // send toggleOnButton state
    sprintf(buff_topic, "home/nb/node%02d/dev06", nodeId);
    if (!toggleOnButton) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    sendArray[6] = false;
  }

  if (sendArray[7]) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev07", nodeId);
    // sprintf(buff_msg, "%d", TIMinterval);
    pubMQTT(buff_topic, buff_msg);
    sendArray[7] = false;
  }

  if (sendArray[8]) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev08", nodeId);
    //sprintf(buff_msg, "%d", TIMShower);
    pubMQTT(buff_topic, buff_msg);
    sendArray[8] = false;
    //pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[9]) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev09", nodeId);
    //sprintf(buff_msg, "%d", TIMShower2);
    pubMQTT(buff_topic, buff_msg);
    sendArray[9] = false;
    //pubMQTT(buff_topic, buff_msg);
  }

  if (sendArray[10]) {               // send IP address
    sprintf(buff_topic, "home/nb/node%02d/dev10", nodeId);
    for (i = 0; i < 16; i++) {
      buff_msg[i] = IP[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    sendArray[10] = false;
  }

  if (sendArray[11]) {   //send SSID
    sprintf(buff_topic, "home/nb/node%02d/dev11", nodeId);
    strcpy(buff_msg,wifi_ssid);
    pubMQTT(buff_topic, buff_msg);
    sendArray[11] = false;
  }

  if (sendArray[12]) {   //send PW
    sprintf(buff_topic, "home/nb/node%02d/dev12", nodeId);
    for (i = 0; i < 20; i++) {
      buff_msg[i] = wifi_password[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    sendArray[12] = false;
  }

  if (sendArray[13]) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev13", nodeId);
    // sprintf(buff_msg, "%d", moistTimer);
    pubMQTT(buff_topic, buff_msg);
    sendArray[13] = false;
    //pubMQTT(buff_topic, buff_msg);
  }
  
  for (byte i=1 ; i<5 ; i++) {
    if (sendArray[i+15]) {                 // send actuator state
      sprintf(buff_topic, "home/nb/node%02d/dev%02d", nodeId, i+15);
      if (relayStruct[i].get()){ 
        sprintf(buff_msg, "ON");
      }else{
        if (!relayStruct[i].get()){
          sprintf(buff_msg, "OFF");
        }
      }
      pubMQTT(buff_topic, buff_msg);
      sendArray[i+15] = false;
      
      }
  }

  if (sendArray[40]) {                 // send button pressed message
    sprintf(buff_topic, "home/nb/node%02d/dev40", nodeId);
    if (ACT1State == relayStruct[0].off()) sprintf(buff_msg, "OFF");
    if (ACT1State == relayStruct[0].on()) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    sendArray[40] = false;
  }
 
  //-----------------
  if (sendArray[48]) {
    sprintf(buff_topic, "home/nb/node%02d/dev48", nodeId);
    //dtostrf(raw, 10, 2, buff_msg);
    while (buff_msg[0] == 32) {        // remove any leading spaces
      for (int i = 0; i < strlen(buff_msg); i++) {
        buff_msg[i] = buff_msg[i + 1];
      }
    }
    pubMQTT(buff_topic, buff_msg);
    sendArray[48] = false;
  }
  //---------------
#ifdef USE_LIPO
  if (sendArray[49])  //batteryvoltage
  {
    sprintf(buff_topic, "home/nb/node%02d/dev49", nodeId);
    batterij();
    dtostrf(voltage, 10, 2, buff_msg);
    //despace();
    while (buff_msg[0] == 32) {        // remove any leading spaces
      for (int i = 0; i < strlen(buff_msg); i++) {
        buff_msg[i] = buff_msg[i + 1];
      }
    }

    // pubMQTT(buff_topic, buff_msg);
    pubMQTT(buff_topic, data);
    sendArray[49] = false;

  }
#endif

}

void checkChip() {
   Serial.println("Check ID in:");
   Serial.println("https://www.wemos.cc/verify_products");
   Serial.printf("Chip ID = %08Xn", ESP.getChipId());
}

void openCloseAllRelay(int newState){
    for (byte i=1 ; i<5 ; i++) {
      relayStruct[i].set(newState);
      sendArray[i+15] = true;
  }
}

//===============================================================================================
//  SETUP
void setup(void) {

               
  //pinMode(1,FUNCTION_3);
  //pinMode(3,FUNCTION_3);  

  // Init Serial
#ifdef DEBUG
    Serial.begin(SERIAL_BAUD);
    //checkChip();
#endif

  // Push Buton
  debouncer.attach(BTN,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode 
  debouncer.interval(50); // Use a debounce interval of 50 milliseconds
  //---------
  ACT1State=relayStruct[1].get();

  setAck=true;

  sendArray[0] = false;//uptime
  sendArray[1] = true; // interval for push messages
  sendArray[2] = true; // RSSI
  sendArray[3] = true;// Version
  sendArray[5] = true;// ACK
  sendArray[7] = false;//timerinterval (sec)
  sendArray[8] = false;//timer1 (min)
  sendArray[9] = false; //timer2 (minm)
  sendArray[10] = true;// send IP on startup
  sendArray[11] = false; //SSID
  sendArray[13] = false; // timer for moisture
  sendArray[16] = true; //read/set actuator output
  sendArray[17] = true; //Relay
  sendArray[18] = true;
  sendArray[19] = true;
  sendArray[40] = true; //button
  sendArray[46] = false; 
  sendArray[48] - false; 
  sendArray[49] = false; 
  sendArray[50] = false; 
  sendArray[51] = true; 
  sendArray[52] = false; 
  sendArray[53] = true; 
  sendArray[54] = false; 
  sendArray[59] = true;
  sendArray[60] = true;
  
  setupOTA(); //  OTA Configuration
  
  IP = WiFi.localIP().toString();
  sendMsg();                          // send any mqtt messages
}

//  LOOP

//===============================================================================================
void loop() {                       // Main program loop
  
  ArduinoOTA.handle();
  
  if (WiFi.status() != WL_CONNECTED) {            // Wifi connected ?
    connectWifi();
  }

  if (!client.connected()) {                  // MQTT connected ?
    connectMqtt();
  }
  client.loop();

  debouncer.update(); // Update the Bounce instance
  if ( debouncer.fell() ) {  // Call code if button transitions from HIGH to LOW
    if (setAck) sendArray[40] = true;                // send button message
    ACT1State = !ACT1State;
    openCloseAllRelay(ACT1State);    
  }
  //-------------------------------------------------------------

  // INCREASE UPTIME

  if (lastMinute != (millis() / 60000)) {         // another minute passed ?
    lastMinute = millis() / 60000;
    upTime++;
  }

  // PERIODIC TRANSMISSION


  if (TXinterval > 0) {
    int currPeriod = millis() / (TXinterval * 1000);
    if (currPeriod != lastPeriod) {             // interval elapsed ?
      lastPeriod = currPeriod;

      // list of sensordata to be sent periodically..
      // remove comment to include parameter in transmission


      sendArray[0] = true;                     // uptime
      sendArray[1] = true; //  interval
      sendArray[2] = false;                   // send RSSI
      sendArray[3] = false;                   // send version
      sendArray[8] = false;                  
      sendArray[9] = false;                 
      sendArray[10] = true;                // send IP address
      sendArray[11] = true;
      sendArray[13] = false; // 
      sendArray[16] = true;                    // output state relay
      sendArray[17] = true; //Relay2
      sendArray[18] = true; // Relay3
      sendArray[19] = true; // Relay4
      sendArray[40] = true; //button
      sendArray[41] = false; //
      sendArray[42] = false; //
      sendArray[46] = false; //
      sendArray[48] = false; //
      sendArray[49] = false; //
      sendArray[51] = false; //
      sendArray[53] = false; //
      sendArray[55] = false; //
      sendArray[59] = false; //
      sendArray[60] = false; // 
      sendArray[61] = false; //
    }
  }

  //--------------

  sendMsg();                          // send any mqtt messages

}   // end loop
