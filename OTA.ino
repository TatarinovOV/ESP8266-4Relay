//
// OTA set up and events
//

//screen /dev/ttyUSB0 115200 ;
// CTRL+A, CTRL+D to exit

void setupOTA(void)
{
  ArduinoOTA.setHostname(host);
  ArduinoOTA.setPasswordHash((const char *)OTAPassMD5);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA upload start");
    // switch strip off in case OTA fails
  

  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA upload end");
    //Serial.println("Restarting...");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    openCloseAllRelay(HIGH);  // Open All relay on error
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); 
  });

  ArduinoOTA.begin();

  Serial.println("OTA initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP().toString());
}
