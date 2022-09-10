#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>
#include <unordered_map> 

//#define _MQTT_ENABLE_
#ifdef _MQTT_ENABLE_
#include <PubSubClient.h>

#define MQTT_BUFFER_SIZE 2048
#define MQTT_HOST "【MQTTブローカーのホスト名】"
#define MQTT_PORT 【MQTTブローカのポート番号】
#define MQTT_TOPIC_PUBLISH  "bledevicescan"
#define MQTT_CONNECT_TRY_COUNT 5

static char client_name[18];

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
#endif

#define _HTTP_ENABLE_
#ifdef _HTTP_ENABLE_
#include <HTTPClient.h>

#define HTTP_PUT_URL   "http://【Node.jsサーバのホスト名】/bledevice-put"
#endif

#define _HAS_PIXELS_
#ifdef _HAS_PIXELS_
#include <Adafruit_NeoPixel.h>

#define PIXELS_PORT  2
#define NUM_OF_PIXELS 1
static Adafruit_NeoPixel pixels(NUM_OF_PIXELS, PIXELS_PORT, NEO_GRB + NEO_KHZ800);
#endif

#define SCAN_DURATION_SEC  30
#define PUBLISH_INTERVAL  60000
#define EXPIRE_DURATION_SEC 60

#define JSON_BUFFER_SIZE  2048
const int capacity = JSON_BUFFER_SIZE;
StaticJsonDocument<capacity> json_request;
char json_buffer[JSON_BUFFER_SIZE];

static BLEUUID sticknfind_discoverServiceUUID("bec26202-a8d8-4a94-80fc-9ac1de37daa6");
static BLEUUID linking_discoverServiceUUID("b3b36901-50d3-4044-808d-50835b13a6cd");
static BLEUUID immediate_alert_discoverServiceUUID((uint16_t)0x1802);
static BLEUUID line_things_discoverServiceUUID((uint16_t)0xfe6f);

#define DEVICE_TYPE_UNKNOWN         0
#define DEVICE_TYPE_STICKNFIND      1
#define DEVICE_TYPE_PROJECT_LINKING 2
#define DEVICE_TYPE_LINE_THINGS     3
#define DEVICE_TYPE_IMMEDIATE_ALERT 4

typedef struct{
  uint32_t updated_at;
  bool connected;
  uint8_t device_type;
  uint8_t address_type;
  int rssi;
  std::string name;
} BLE_DEVICE_ENTRY;
static std::unordered_map<std::string, BLE_DEVICE_ENTRY> device_list;

static BLEAdvertisedDevice* myDevice;
static BLEScan* pBLEScan;

static uint8_t wifi_mac_address[6];
static uint8_t wifi_ip_address[4];
static uint8_t ble_mac_address[6];
static time_t start_time;
static uint32_t start_tim;
static uint32_t prev_tim;
static uint8_t last_day;

//#define WIFI_SSID "【固定のWiFiアクセスポイントのSSID】" // WiFiアクセスポイントのSSID
//#define WIFI_PASSWORD "【固定のWiFIアクセスポイントのパスワード】" // WiFIアクセスポイントのパスワード
#define WIFI_SSID NULL // WiFiアクセスポイントのSSID
#define WIFI_PASSWORD NULL // WiFIアクセスポイントのパスワード
#define WIFI_TIMEOUT  10000
#define SERIAL_TIMEOUT1  10000
#define SERIAL_TIMEOUT2  20000
static long wifi_try_connect(bool infinit_loop);

static long httpPost(const char *p_url, const char *p_payload);

static time_t getTime(void)
{
  for( int i = 0 ; i < 10 ; i++ ){
    time_t t = time(NULL);
    if( t > 0 )
      return t;
  }

  return -1;
}

static uint32_t getCurrentTime(void)
{
  uint32_t now = millis();
  uint32_t diff = now - start_tim;
  return start_time + diff / 1000;
}

static void updateStartTime(void)
{
  start_time = getTime();
  start_tim = millis();
}

#ifdef _MQTT_ENABLE_
static long mqttPublish(const char *p_message)
{
  bool ret = mqttClient.publish(MQTT_TOPIC_PUBLISH, p_message);
  return ret ? 0 : -1;
}
#endif

void myScanCompleteCallback(BLEScanResults results)
{
  Serial.println("counting start");

  long ret;

  time_t current_time = getCurrentTime();
  char address[18];
  json_request.clear();
  sprintf(address, "%02x:%02x:%02x:%02x:%02x:%02x", ble_mac_address[0], ble_mac_address[1], ble_mac_address[2], ble_mac_address[3], ble_mac_address[4], ble_mac_address[5]);
  json_request["ble_mac_address"] = address;
  sprintf(address, "%02x:%02x:%02x:%02x:%02x:%02x", wifi_mac_address[0], wifi_mac_address[1], wifi_mac_address[2], wifi_mac_address[3], wifi_mac_address[4], wifi_mac_address[5]);
  json_request["wifi_mac_address"] = address;
  sprintf(address, "%d.%d.%d.%d", wifi_ip_address[0], wifi_ip_address[1], wifi_ip_address[2], wifi_ip_address[3]);
  json_request["wifi_ip_address"] = address;

  int index = 0;
  for (std::unordered_map<std::string, BLE_DEVICE_ENTRY>::iterator iterator = device_list.begin(); iterator != device_list.end(); iterator++){
    std::pair<std::string, BLE_DEVICE_ENTRY> element = *iterator;
    std::string mac_address = element.first;

    if( device_list[mac_address].connected && device_list[mac_address].updated_at < (current_time - EXPIRE_DURATION_SEC) ){
      device_list[mac_address].updated_at = getCurrentTime();
      device_list[mac_address].connected = false;
    }

    BLE_DEVICE_ENTRY entry = element.second;
    json_request["device_list"][index]["mac_address"] = (char*)mac_address.c_str(),
    json_request["device_list"][index]["name"] = (char*)entry.name.c_str(),
    json_request["device_list"][index]["connected"] = entry.connected;
    json_request["device_list"][index]["device_type"] = entry.device_type;
    json_request["device_list"][index]["address_type"] = entry.address_type;
    json_request["device_list"][index]["rssi"] = entry.rssi;
    json_request["device_list"][index]["updated_at"] = entry.updated_at;
    index++;
  }
  if( serializeJson(json_request, json_buffer, sizeof(json_buffer)) <= 0 ){
    Serial.println("serializeJson error");
    return;
  }

#ifdef _MQTT_ENABLE_
  ret = mqttPublish(json_buffer);
  if( ret != 0 )
    Serial.println("mqtt publish failed");
#endif

#ifdef _HTTP_ENABLE_
  ret = httpPost(HTTP_PUT_URL, json_buffer);
  if( ret != 0 )
    Serial.println("httpPost error");
#endif
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.haveServiceUUID() ){
      uint8_t device_type = DEVICE_TYPE_UNKNOWN;
      if(advertisedDevice.isAdvertisingService(sticknfind_discoverServiceUUID))
        device_type = DEVICE_TYPE_STICKNFIND;
      else if(advertisedDevice.isAdvertisingService(immediate_alert_discoverServiceUUID))
        device_type = DEVICE_TYPE_IMMEDIATE_ALERT;
      else if(advertisedDevice.isAdvertisingService(linking_discoverServiceUUID))
        device_type = DEVICE_TYPE_PROJECT_LINKING;
      else if(advertisedDevice.isAdvertisingService(line_things_discoverServiceUUID))
        device_type = DEVICE_TYPE_LINE_THINGS;
      else
        return;

      BLE_DEVICE_ENTRY entry = { getCurrentTime(), true, device_type,advertisedDevice.getAddressType(),
        advertisedDevice.haveRSSI() ? advertisedDevice.getRSSI(): 0, advertisedDevice.haveName() ? advertisedDevice.getName() : "" };
      device_list[advertisedDevice.getAddress().toString()] = entry;

      Serial.print("*** BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());
    }
  }
}; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

#ifdef _HAS_PIXELS_
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, 0x00ff00);
  pixels.show();
#endif

  long ret;

  ret = wifi_try_connect(true);
  if( ret != 0 ){
    Serial.println("WiFi cant connect");
    while(1);
  }

  esp_read_mac(ble_mac_address, ESP_MAC_BT);
  WiFi.macAddress(wifi_mac_address);
  IPAddress address = WiFi.localIP();
  for( int i = 0 ; i < 4 ; i++ )
    wifi_ip_address[i] = address[i];

  Serial.printf("ble_mac_address=%02x:%02x:%02x:%02x:%02x:%02x\n", ble_mac_address[0], ble_mac_address[1], ble_mac_address[2], ble_mac_address[3], ble_mac_address[4], ble_mac_address[5]);
  Serial.printf("wifi_mac_address=%02x:%02x:%02x:%02x:%02x:%02x\n", wifi_mac_address[0], wifi_mac_address[1], wifi_mac_address[2], wifi_mac_address[3], wifi_mac_address[4], wifi_mac_address[5]);
  Serial.printf("wifi_ip_address=%d.%d.%d.%d\n", wifi_ip_address[0], wifi_ip_address[1], wifi_ip_address[2], wifi_ip_address[3]);

  configTzTime("JST-9", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
  struct tm timeInfo;
  getLocalTime(&timeInfo);
  Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n", timeInfo.tm_year + 1900, timeInfo.tm_mon + 1 , timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  last_day = timeInfo.tm_mday;

#ifdef _MQTT_ENABLE_
  sprintf(client_name, "%02x:%02x:%02x:%02x:%02x:%02x", client_mac_address[0], client_mac_address[1], client_mac_address[2], client_mac_address[3], client_mac_address[4], client_mac_address[5] );
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
#endif

  updateStartTime();
  prev_tim = millis();

  BLEDevice::init("NearbyBleDevice");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//  pBLEScan->setInterval(1349);
//  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  pBLEScan->start(SCAN_DURATION_SEC, myScanCompleteCallback);

  Serial.println("setup finished\n");
}

void loop() {
  // put your main code here, to run repeatedly:
#ifdef _MQTT_ENABLE_
  mqttClient.loop();
  for( int i = 0 ; !mqttClient.connected() && i < MQTT_CONNECT_TRY_COUNT ; i++ ){
    Serial.println("Mqtt Reconnecting");
    if (mqttClient.connect(client_name)){
      Serial.println("Mqtt Reconnected");
      break;
    }

    delay(500);
  }
#endif

  uint32_t now = millis();
  if( (now - prev_tim) > PUBLISH_INTERVAL ){
    prev_tim = now;

    struct tm timeInfo;
    getLocalTime(&timeInfo);
    if( timeInfo.tm_mday != last_day ){
      updateStartTime();
      last_day = timeInfo.tm_mday;
    }

    pBLEScan->start(SCAN_DURATION_SEC, myScanCompleteCallback);
  }

  delay(1000);  
}

static long wifi_connect(const char *ssid, const char *password, unsigned long timeout)
{
  Serial.println("");
  Serial.print("WiFi Connenting");

  if( ssid == NULL && password == NULL )
    WiFi.begin();
  else
    WiFi.begin(ssid, password);
  unsigned long past = 0;
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
    past += 500;
    if( past > timeout ){
      Serial.println("\nCan't Connect");
      return -1;
    }
  }
  Serial.print("\nConnected : IP=");
  Serial.print(WiFi.localIP());
  Serial.print(" Mac=");
  Serial.println(WiFi.macAddress());

  return 0;
}

static long wifi_try_connect(bool infinit_loop)
{
  long ret = -1;
  do{
    ret = wifi_connect(WIFI_PASSWORD, WIFI_PASSWORD, WIFI_TIMEOUT);
    if( ret == 0 )
      return ret;

    Serial.print("\ninput SSID:");
    Serial.setTimeout(SERIAL_TIMEOUT1);
    char ssid[32 + 1] = {'\0'};
    ret = Serial.readBytesUntil('\r', ssid, sizeof(ssid) - 1);
    if( ret <= 0 )
      continue;

    delay(10);
    Serial.read();
    Serial.print("\ninput PASSWORD:");
    Serial.setTimeout(SERIAL_TIMEOUT2);
    char password[32 + 1] = {'\0'};
    ret = Serial.readBytesUntil('\r', password, sizeof(password) - 1);
    if( ret <= 0 )
      continue;

    delay(10);
    Serial.read();
    Serial.printf("\nSSID=%s PASSWORD=", ssid);
    for( int i = 0 ; i < strlen(password); i++ )
      Serial.print("*");
    Serial.println("");

    ret = wifi_connect(ssid, password, WIFI_TIMEOUT);
    if( ret == 0 )
      return ret;
  }while(infinit_loop);

  return ret;
}

#ifdef _HTTP_ENABLE_
static long httpPost(const char *p_url, const char *p_payload)
{
  HTTPClient http;
  http.begin(p_url);
  http.addHeader("Content-Type", "application/json");
  int status_code = http.POST(p_payload);
  Serial.printf("status_code=%d\r\n", status_code);
  if( status_code != 200 ){
    http.end();
    return -1;
  }
  Serial.println(http.getString());
  http.end();

  return 0;
}
#endif
