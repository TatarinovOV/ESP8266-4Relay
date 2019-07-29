
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
//  00 uptime:   read uptime in minutes
//  01 interval: read/set transmission interval for push messages
//  02 RSSI:   read radio signal strength
//  03 version:  read software version
//  05 ACK:    read/set acknowledge message after a 'set' request
//  06 toggle:   read/set select toggle / timer function
//  07 timer:    read/set timer interval in seconds
//  08
//  09
//  10  IP:     Read IP address
//  11  SSID: Read SSID
//  12  PW: Read Password
//  13 
//  16  read/set relay1 output
//  17  read set relay2 output
//  18  read set relay3 output
//  19  read set relay4 output
//  40  button    tx only: button pressed
//  92  error:    tx only: device not supported
//  91  error:    tx only: syntax error
//  99  wakeup:   tx only: first message sent on node startup
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
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev16' -m OFF -u MqttUser -P WgyK1LKm
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev17' -m OFF -u MqttUser -P WgyK1LKm
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev18' -m OFF -u MqttUser -P WgyK1LKm
  mosquitto_pub -h mqtt.zalin.home -t 'home/sb/node01/dev19' -m OFF -u MqttUser -P WgyK1LKm

 */

/*
   GPIO0  -->IN3  DID18
   GPIO1  (Tx)  -->IN1  DID16
   GPIO2  --> IN2  DID17
   GPIO3  (Rx) -->IN4  DID 19
*/

#include "userdata.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>        // Include the mDNS library //LJU
#include "PubSubClient.h"
#define DEBUG                 // uncomment for debugging  
#define ACT1 12                 // Actuator pin (LED or relay to ground)  //D6 on Wemos unnecessary
#define MQCON 13                // MQTT link indicator D7 op Wemos unnecessary
#define BTN 0                 // Button pin (also used for Flash setting)  D3 op Wemos  unnecessary
#define SERIAL_BAUD 115200
#define HOLDOFF 1000  // blocking period between button messages

#define REL1 2 //GPIO02 D4=>in1 //1   //DID16
#define REL2 14 //gpio14(D5) =>in2  //1 //DID17
#define REL3 12 //gpio12(D6) =>in3 //0   //DID18
#define REL4 13 //gpio13(D7) =>in4// 3   //DID19

/*
int const REL1 = 2; //gpio5(D3) =>in1 //1   //DID16
int const REL2 = 14; //gpio4(D4) =>in2  //1 //DID17
int const REL3 = 12; //gpio12(D6) =>in3 //0   //DID18
int const REL4 = 13; //gpio13(D7) =>in4// 3   //DID19
*/


//  STARTUP DEFAULTS

long  TXinterval = 60;            // periodic transmission interval in seconds
long  TIMinterval = 60;           // timer interval in seconds
bool  ackButton = false;            // flag for message on button press
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
bool  REL1State;
bool  REL2State;
bool  REL3State;
bool  REL4State;
bool  RELStateArray[4];
bool  mqttNotCon = true;            // MQTT broker not connected flag
int   signalStrength;             // radio signal strength
bool  wakeUp = true;              // wakeup indicator
bool  setAck = true;             // acknowledge receipt of actions
bool  curState = true;            // current button state
bool  lastState = true;           // last button state
bool  timerOnButton = false;          // timer output on button press
bool  msgBlock = false;           // flag to hold button message
bool  readAction;               // indicates read / set a value
bool  send0, send1, send2, send3, send5, send6, send7, send8, send9;
bool  send10, send11, send12, send13, send16, send17, send18, send19, send40, send41, send42, send46;
bool  send48, send49, send50, send51, send52, send53, send54, send55, send59, send60, send61;
bool  send99;  // message triggers
String  IP;                   // IPaddress of ESP
char  buff_topic[30];             // mqtt topic
char  buff_msg[32];             // mqtt message
char  clientName[10];             // Mqtt client name
char  wifi_ssid[20];                // Wifi SSID name
char  wifi_password[20];              // Wifi password

void mqttSubs(char* topic, byte* payload, unsigned int length);

WiFiClient espClient;
PubSubClient client(mqtt_server, mqttPort, mqttSubs, espClient); // instantiate MQTT client


// OTA configuration
//
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* host = HOST;
// function prototypes required by Arduino IDE 1.6.7
void setupOTA(void);



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
          send0 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 1) {             // transmission interval
        error = 0;
        if (readAction) {
          send1 = true;
        } else {                // set transmission interval
          TXinterval = strPayload.toInt();
          if (TXinterval < 10 && TXinterval != 0) TXinterval = 10; // minimum interval is 10 seconds
        }
      }
      if (DID == 2) {               // RSSI
        if (readAction) {
          send2 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 3) {             // version
        if (readAction) {
          send3 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 5) {             // ACK
        if (readAction) {
          send5 = true;
          error = 0;
        } else if (strPayload == "ON") {
          setAck = true;
          if (setAck) send5 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          setAck = false;
          if (setAck) send5 = true;
          error = 0;
        } else error = 3;
      }

      if (DID == 6) {             // toggle / timer mode selection
        if (readAction) {
          send6 = true;
          error = 0;
        } else if (strPayload == "ON") {    // select toggle mode
          toggleOnButton = true;
          if (setAck) send6 = true;
          error = 0;
        } else if (strPayload == "OFF") {   // select timer mode
          toggleOnButton = false;
          if (setAck) send6 = true;
          error = 0;
        } else error = 3;
      }

      if (DID == 7) {             // Timer interval
        error = 0;
        if (readAction) {
          send7 = true;
        } else {                // set timer interval
          TIMinterval = strPayload.toInt();
          if (TIMinterval < 5 && TIMinterval != 0) TIMinterval = 5; // minimum interval is 5 seconds
        }
      }
      //------------------------------------------
      if (DID == 8) {             // Timer interval
        error = 0;
        if (readAction) {
          send8 = true;
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
          send9 = true;
        } else {                // set timer interval
          // TIMShower2 = strPayload.toInt();
          // Serial.print(TIMShower2);
        }
      }
      //---------------------------------------
      //ip
      if (DID == 10) {               // IP
        if (readAction) {
          send10 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }

      //------------------------------------------

      if (DID == 11) {   //SSID
        if (readAction) {
          send11 = true;
          error = 0;
        } else error = 3;
      }
//--------------------------------------
      if (DID == 12) {   //PW
        if (readAction) {
          send12 = true;
          error = 0;
        } else error = 3;
      }
      //--------------------
       if (DID == 13) {             // Timer interval
        error = 0;
        if (readAction) {
          send13 = true;
        } else {                // set timer interval
          // moistTimer = strPayload.toInt();
          // Serial.print(moistTimer);
        }
      }
      //-------------------

      if (DID == 16) {              // state of Relay1 GPIO1 TX
        if (readAction) {
          //Serial.end();
          send16 = true;
          error = 0;
        } else if (strPayload == "ON") {
          REL1State = 0; //mind you,the relay used will switch on at LOW
          digitalWrite(REL1, REL1State);
          if (setAck) send16 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          REL1State = 1;
          digitalWrite(REL1, REL1State);
          if (setAck) send16 = true;
          error = 0;
        } else error = 3;
      }
      //-----------------------------

    if (DID == 17) {              // state of Relay2
        if (readAction) {
          send17 = true;
          error = 0;
        } else if (strPayload == "ON") {
          REL2State = 0;
          digitalWrite(REL2, REL2State);
          if (setAck) send17 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          REL2State = 1;
          digitalWrite(REL2, REL2State);
          if (setAck) send17 = true;
          error = 0;
        } else error = 3;
      }
      //-----------------------------
      if (DID == 18) {              // state of Relay3
        if (readAction) {
          send18 = true;
          error = 0;
        } else if (strPayload == "ON") {
          REL3State = 0;
          digitalWrite(REL3, REL3State);
          if (setAck) send18 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          REL3State = 1;
          digitalWrite(REL3, REL3State);
          if (setAck) send18 = true;
          error = 0;
        } else error = 3;
      }
    
      //---------------------------
      if (DID == 19) {              // state of Relay4  GPIO3 Rx
       // Serial.end();
        if (readAction) {
          send19 = true;
          error = 0;
        } else if (strPayload == "ON") {
          REL4State = 0;
          digitalWrite(REL4, REL4State);
          if (setAck) send19 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          REL4State = 1;
          digitalWrite(REL4, REL4State);
          if (setAck) send19 = true;
          error = 0;
        } else error = 3;
      }
      //-----------------------------
      //--------------------------
      if (DID == 46) { //coop door
        if (readAction) {
          send46 = true;
          error = 0;
        } else error = 3;
      }
      //----------------------

      //-----------------------------------------
      if (DID == 49) {   //batterij
        if (readAction) {
          send49 = true;
          error = 0;
        } else error = 3;
      }
      //-----------
      if (DID == 50) {   //DHT11
        if (readAction) {
          send50 = true;
          error = 0;
        } else error = 3;
      }
      //------------------
      if (DID == 51) {   //BMP
        if (readAction) {
          send51 = true;
          error = 0;
        } else error = 3;
      }
      //--------------
      if (DID == 52)  { //BH1750
        if (readAction) {
          send52 = true;
          error = 0;
        } else error = 3;
      }
      //----
      if (DID == 53) { ///pcf8591
        //DHT11
        if (readAction) {
          send53 = true;
          error = 0;
        } else error = 3;
      }
      //----------------------
      if (DID == 55) { //ds18b20
        //DHT11
        if (readAction) {
          send55 = true;
          error = 0;
        } else error = 3;
      }
      //---------------------------
      if (DID == 59) {
        if (readAction) {
          send59 = true;
          error = 0;
        } else error = 0;
      }
      //--------------
      if (DID == 60) {
        if (readAction) {
          send60 = true;
          error = 0;
        } else error = 0;
      }
      //---------
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
    Serial.println("Error setting up MDNS responder!");
  }

  
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
    send99 = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (send0) {                  // send uptime
    sprintf(buff_topic, "home/nb/node%02d/dev00", nodeId);
    sprintf(buff_msg, "%d", upTime);
    send0 = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (send1) {                  // send transmission interval
    sprintf(buff_topic, "home/nb/node%02d/dev01", nodeId);
    sprintf(buff_msg, "%d", TXinterval);
    send1 = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (send2) {                  // send transmission interval
    sprintf(buff_topic, "home/nb/node%02d/dev02", nodeId);
    signalStrength = WiFi.RSSI();
    sprintf(buff_msg, "%d", signalStrength);
    send2 = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (send3) {                  // send software version
    sprintf(buff_topic, "home/nb/node%02d/dev03", nodeId);
    for (i = 0; i < sizeof(VERSION); i++) {
      buff_msg[i] = VERSION[i];
    }
    buff_msg[i] = '\0';
    send3 = false;
    pubMQTT(buff_topic, buff_msg);
  }

  if (send5) {                  // send ACK state
    sprintf(buff_topic, "home/nb/node%02d/dev05", nodeId);
    if (!setAck) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send5 = false;
  }

  if (send6) {                  // send toggleOnButton state
    sprintf(buff_topic, "home/nb/node%02d/dev06", nodeId);
    if (!toggleOnButton) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send6 = false;
  }

  if (send7) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev07", nodeId);
    // sprintf(buff_msg, "%d", TIMinterval);
    pubMQTT(buff_topic, buff_msg);
    send7 = false;
  }


  if (send8) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev08", nodeId);
    //sprintf(buff_msg, "%d", TIMShower);
    pubMQTT(buff_topic, buff_msg);
    send8 = false;
    //pubMQTT(buff_topic, buff_msg);
  }


  if (send9) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev09", nodeId);
    //sprintf(buff_msg, "%d", TIMShower2);
    pubMQTT(buff_topic, buff_msg);
    send9 = false;
    //pubMQTT(buff_topic, buff_msg);
  }



  if (send10) {               // send IP address
    sprintf(buff_topic, "home/nb/node%02d/dev10", nodeId);
    for (i = 0; i < 16; i++) {
      buff_msg[i] = IP[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send10 = false;
  }


  if (send11) {   //send SSID
    sprintf(buff_topic, "home/nb/node%02d/dev11", nodeId);
    for (i = 0; i < 20; i++) {
      buff_msg[i] = wifi_ssid[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send11 = false;
  }


  if (send12) {   //send PW
    sprintf(buff_topic, "home/nb/node%02d/dev12", nodeId);
    for (i = 0; i < 20; i++) {
      buff_msg[i] = wifi_password[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send12 = false;
  }


  if (send13) {                  // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev13", nodeId);
    // sprintf(buff_msg, "%d", moistTimer);
    pubMQTT(buff_topic, buff_msg);
    send13 = false;
    //pubMQTT(buff_topic, buff_msg);
  }

 

  if (send16) {                 // send actuator state
    sprintf(buff_topic, "home/nb/node%02d/dev16", nodeId);
    if (REL1State == 0) sprintf(buff_msg, "ON");
    if (REL1State == 1) sprintf(buff_msg, "OFF");
    pubMQTT(buff_topic, buff_msg);
    send16 = false;
  }

    if (send17) {                 // send actuator state
    sprintf(buff_topic, "home/nb/node%02d/dev17", nodeId);
    if (REL2State == 0) sprintf(buff_msg, "ON");
    if (REL2State == 1) sprintf(buff_msg, "OFF");
    pubMQTT(buff_topic, buff_msg);
    send17 = false;
  }

    if (send18) {                 // send actuator state
    sprintf(buff_topic, "home/nb/node%02d/dev18", nodeId);
    if (REL3State == 0) sprintf(buff_msg, "ON");
    if (REL3State == 1) sprintf(buff_msg, "OFF");
    pubMQTT(buff_topic, buff_msg);
    send18 = false;
  }

    if (send19) {                 // send actuator state
    sprintf(buff_topic, "home/nb/node%02d/dev19", nodeId);
    if (REL4State == 0) sprintf(buff_msg, "ON");
    if (REL4State == 1) sprintf(buff_msg, "OFF");
    pubMQTT(buff_topic, buff_msg);
    send19 = false;
  }

  if (send40) {                 // send button pressed message
    sprintf(buff_topic, "home/nb/node%02d/dev40", nodeId);
    if (ACT1State == 0) sprintf(buff_msg, "OFF");
    if (ACT1State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send40 = false;
  }
 
  //-----------------
  if (send48) {
    sprintf(buff_topic, "home/nb/node%02d/dev48", nodeId);
    //dtostrf(raw, 10, 2, buff_msg);
    while (buff_msg[0] == 32) {        // remove any leading spaces
      for (int i = 0; i < strlen(buff_msg); i++) {
        buff_msg[i] = buff_msg[i + 1];
      }
    }
    pubMQTT(buff_topic, buff_msg);
    send48 = false;
  }
  //---------------
#ifdef USE_LIPO
  if (send49)  //batteryvoltage
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
    send49 = false;

  }
#endif

}

void checkChip() {
   Serial.println("Check ID in:");
   Serial.println("https://www.wemos.cc/verify_products");
   Serial.printf("Chip ID = %08Xn", ESP.getChipId());
}

//  SETUP

//===============================================================================================
void setup() {

               
  //pinMode(1,FUNCTION_3);
  //pinMode(3,FUNCTION_3);  
  pinMode(REL1,OUTPUT);
  pinMode(REL2,OUTPUT);
  pinMode(REL3,OUTPUT);
  pinMode(REL4,OUTPUT);

  delay(500);
  // Init Serial
#ifdef DEBUG
    Serial.begin(SERIAL_BAUD);
    //checkChip();
#endif
  //---------
  setAck=true;
  REL1State = REL2State = REL3State = REL4State = HIGH ; 
  digitalWrite(REL1,REL1State);
  digitalWrite(REL2,REL2State);
  digitalWrite(REL3,REL3State);
  digitalWrite(REL4,REL4State);


  send0 = false;//uptime
  send1 = true; // interval for push messages
  send3 = true;// Version
  send5 = true;// ACK
  send7 = false;//timerinterval (sec)
  send8 = false;//timer1 (min)
  send9 = false; //timer2 (minm)
  send10 = true;// send IP on startup
  send11 = true; //SSID
  send13 = true; // timer for moisture
  send16 = true; //read/set actuator output
  send17 = true; //Relay
  send18 = true;
  send19 = true;
  send40 = false;
  send46 = false; 
  send48 - false; 
  send49 = false; 
  send50 = false; 
  send51 = true; 
  send52 = false; 
  send53 = true; 
  send54 = false; 
  send59 = true;
  send60 = true;
  setupOTA();
  IP = WiFi.localIP().toString();
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


  // DETECT INPUT CHANGE
// not necessary for current application
  curState = digitalRead(BTN);                // Read button  
  msgBlock = ((millis() - lastBtnPress) < HOLDOFF);     // hold-off time for additional button messages
  if (!msgBlock &&  (curState != lastState)) {        // input changed ?
    delay(5);
    lastBtnPress = millis();                // take timestamp
    if (setAck) send40 = true;                // send button message
    if (curState == LOW) {
      if (toggleOnButton) {                 // button in toggle state ?
        ACT1State = !ACT1State;               // toggle output
        digitalWrite(ACT1, ACT1State);
        send16 = true;                    // send message on status change
      } else if (TIMinterval > 0 && !timerOnButton) {       // button in timer state ?
        timerOnButton = true;               // start timer interval
        ACT1State = HIGH;                 // switch on ACT1
        digitalWrite(ACT1, ACT1State);
        send16 = true;
      }
    }
    lastState = curState;
  }

  // TIMER CHECK

  if (TIMinterval > 0 && timerOnButton) {           // =0 means no timer
    if ( millis() - lastBtnPress > TIMinterval * 1000) {  // timer expired ?
      timerOnButton = false;                // then end timer interval
      ACT1State = LOW;                  // and switch off Actuator
      digitalWrite(ACT1, ACT1State);
      send16 = true;
    }
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


      send0 = true;                     // uptime
      send1 = true; //  interval
      send2 = false;                   // send RSSI
      send3 = false;                   // send version
      send8 = false;                  
      send9 = false;                 
      send10 = true;                // send IP address
      send11 = false;
      send13 = false; // 
      send16 = true;                    // output state relay
      send17 = true; //Relay2
      send18 = true; // Relay3
      send19 = true; // Relay4
      send41 = false; //
      send42 = false; //
      send46 = false; //
      send48 = false; //
      send49 = false; //
      send51 = false; //
      send53 = false; //
      send55 = false; //
      send59 = false; //
      send60 = false; // 
      send61 = false; //
    }
  }

  //--------------

  sendMsg();                          // send any mqtt messages

}   // end loop
