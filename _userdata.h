#define VERSION "ESP-01_relay.ino"
#define HOST  "WiFiRelay"
#define TRANSMITINTERVAL 30
#define wifi_ssid_A ""          // wifi station name
#define wifi_password_A ""      // wifi password
#define wifi_ssid_B  wifi_ssid_A          // fallback wifi station name; if not present use same as A
#define wifi_password_B wifi_password_A      // fallback wifi password; if not present use same as A
#define mqtt_server "mqtt.local"     // mqtt server IP  >make this yr MQTTserver IP
#define mqtt_user "MqttUser"
#define mqtt_password "mqttpassword"
#define nodeId 01               // node ID
#define mqttPort  1883
