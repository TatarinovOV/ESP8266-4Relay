//
// OTA set up and events
//
void setupOTA(void)
{
  ArduinoOTA.setHostname(host);
//ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.onStart([]() {
    //Serial.println("OTA upload start");
    // switch strip off in case OTA fails
  

  });

  ArduinoOTA.onEnd([]() {
    //Serial.println("OTA upload end");
    //Serial.println("Restarting...");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    
    //Serial.printf("Error[%u]: ", error);
  //  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    //else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    //else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    //else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
   // else if (error == OTA_END_ERROR) Serial.println("End Failed");
    
  });

  ArduinoOTA.begin();
 // Serial.println("OTA initialized");
}
