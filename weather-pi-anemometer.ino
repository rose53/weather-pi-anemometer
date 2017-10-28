#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

const char* ssid         = "xxx";
const char* password     = "xxx";
const char* mqttServer   = "192.168.0.115";
const char* mqttUser     = "weatherpi";
const char* mqttPassword = "xxx";

const char* place         = "ANEMOMETER";
const char* sensordata    = "sensordata";
const char* typeWindspeed = "WINDSPEED";

const char* topic = "sensordata/anemometer/windspeed";

const unsigned int deepSleepTimeNoConnection = 15 * 60000000; //

const byte             anemometerPin = 4;
const unsigned long    measureInterval = 5000;
volatile unsigned long pulses = 0;

const unsigned long    maxNoChangeTime = 10 * 60 * 1000;
unsigned long          previousMillis  = 0;     

struct {
  uint32_t crc32;
  float last;
  float act;
  bool  send;
  bool  enableRF;
} rtcData;

WiFiClient espClient;
PubSubClient client(espClient);

char json[256];

uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

/**************************************************************************/
/*
    Try to connect to the MQTT broker, after 10 tries, we return false
*/
/**************************************************************************/
boolean mqttConnect(void) {
  client.setServer(mqttServer, 1883);

  int  tries = 10;
  while (!client.connected()) {
    tries--;
    if (tries == 0) {
      return false;
    }
    if (!client.connect(place, mqttUser, mqttPassword)) {
      delay(5000);
    }
  }
  return true;
}

/**************************************************************************/
/*
    Try to connect to the WIFI
*/
/**************************************************************************/
boolean wifiConnect(void) {
  WiFi.begin(ssid, password);
  // Wait for connection
  int tries = 10;
  boolean retVal = true;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tries--;
    if (tries <= 0) {
      ESP.deepSleep(deepSleepTimeNoConnection, WAKE_RF_DEFAULT);
    }
  }
  return retVal;
}


void sendWindspeed(float windspeed) {

  StaticJsonBuffer<200> jsonBuffer;
    
  JsonObject& obj = jsonBuffer.createObject();

  obj.set("place", place);
    
  obj.set("sensor", "ELTAKO_WS");
  obj.set("type", typeWindspeed);
  obj.set("windspeed", windspeed, 2);

  obj.printTo(json, sizeof(json));
  client.publish(topic, json);


  delay(250);
}

void pulseCallback() {
  pulses++;
}

float getWindSpeed() {

  attachInterrupt(digitalPinToInterrupt(anemometerPin), pulseCallback, RISING);

  pulses = 0;
  unsigned long current = millis();
  unsigned long start = micros();

  while (millis() - current < measureInterval) {
    yield();    
  }
 
  unsigned long diff = micros() - start;
  detachInterrupt(digitalPinToInterrupt(anemometerPin));

  if (diff == 0) {
    return 0.0;
  }
  return (pulses / (diff / (1000.0f * 1000.0f) + 2 )) / 3.0f;
}


void updateWindSpeedRtcData() {
  float act = getWindSpeed();

  if (abs(rtcData.act - act) >= 0.1) {
    rtcData.last     = rtcData.act;
    rtcData.act      = act;
    rtcData.send     = true;      
  } else {
    rtcData.send     = false;      
  }
  rtcData.crc32    = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
}

void deepSleepRFDisabled() {
  // go to sleep with WLAN disabled for 10 sec
  rtcData.enableRF = true; // if we need to send data, we first have to enable RF
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
  ESP.deepSleep(10 * 1000000, WAKE_RF_DISABLED);
}

void enableRF() {
  rtcData.enableRF = false; // if we need to send data, we fisrt have to enable RF
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
  ESP.deepSleep(1000000, WAKE_RF_DEFAULT);
}
void setup() {

  // check data in RTC
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    uint32_t crcOfData = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    if (crcOfData != rtcData.crc32) {
      // crc error, we init new, maybe first startup
      rtcData.last     = 0.0;
      rtcData.act      = 0.0;
      rtcData.send     = false;
      rtcData.enableRF = true;
      rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    }
  }
  yield(); 
  // check, if we need to send the data
  if (!rtcData.send) {
    // get new windspeed, if it differs from the last, set send to true and fall asleep
    updateWindSpeedRtcData();
  }
  yield(); 
  if (!rtcData.send) {
    // go to sleep with WLAN disabled for 10 sec
    deepSleepRFDisabled();
  }
  if (rtcData.enableRF) {
    // we want to send data, but we first have to enable RF
    enableRF();
  }
  yield(); 
  // RF enabled, changed data, ready to send

  // Connect to WiFi network
  wifiConnect();

  if (!mqttConnect()) {
    ESP.deepSleep(30000000, WAKE_RF_DEFAULT);
  }
  // send windspeed, we just waked up, so first send the old value
  sendWindspeed(rtcData.last);
}

void loop() {
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= maxNoChangeTime) {
        // no change since maxNoChangeTime 
        deepSleepRFDisabled();
    }

    if (rtcData.send && mqttConnect()) {
      sendWindspeed(rtcData.act);      
      previousMillis = currentMillis;
    } 
    updateWindSpeedRtcData();
    yield();
}
