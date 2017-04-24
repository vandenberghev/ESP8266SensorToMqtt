#include <SoftwareSerial.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define INTERVAL 30000
#define MH_Z19_RX 12 //D6
#define MH_Z19_TX 13 //D7
#define DHT22_PIN 14 //D5
#define MAX_MISREADS_MHZ19 5

const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";
const char* MQTT_SERVER = "10.0.0.2";

const char* MQTT_IN_TOPIC = "WeMosD1/living/in";
const char* MQTT_OUT_TOPIC_GENERAL = "WeMosD1/living/out";
const char* MQTT_OUT_TOPIC_CO2 = "home/co2";
const char* MQTT_OUT_TOPIC_TEMP = "home/temperature";
const char* MQTT_OUT_TOPIC_RH = "home/relativehumidity";


unsigned long previousMillis = 0;
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);
DHT dht22Sensor;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
char message[128];
unsigned int esp_chipID = 0;
unsigned int mhz19_misread = 0;

bool getTempAndHumidity(double& oTemp, double& oRelativeHumidity)
{
  oTemp = -1;
  oRelativeHumidity = -1;

  uint32_t start = micros();
  double temp = dht22Sensor.getTemperature();
  double humidity = dht22Sensor.getHumidity();
  DHT::DHT_ERROR_t result = dht22Sensor.getStatus();
  uint32_t stop = micros();

  Serial.println("DHT22 data read in " + String(stop - start) + "us");
  Serial.println("DHT22 result: [" + String(result) + "]");

  switch (result)
  {
    case DHT::ERROR_NONE:
      oTemp = temp;
      oRelativeHumidity = humidity;
      return true;

    case DHT::ERROR_CHECKSUM:
      Serial.println("DHT sensor checksum verification error");
      break;
    case DHT::ERROR_TIMEOUT:
      Serial.println("DHT sensor time-out error");
      break;
    default:
        Serial.println("Unknown error");
        break;
  }

  return false;
}



bool getCO2andTemp(int& oCO2, int& oTemp)
{
  //No need to recalculate the checksum; the command always stays the same...
  byte getMeasurementCmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9];

  oCO2 = -1;
  oTemp = -1;

  co2Serial.readBytes(response, 9); //Clear buffer
  co2Serial.write(getMeasurementCmd, 9);
  co2Serial.readBytes(response, 9);

  Serial.println("Data returned by sensor:");
  for (int i = 0; i < sizeof(response); i++) {
    Serial.print(String(i));
    Serial.print(": [");
    Serial.print(String(response[i], HEX));
    Serial.print("] ");
  }
  Serial.println();

  if (response[0] != 0xFF || response[1] != 0x86)
  {
    Serial.println("B0 or B1 unknown");

    if (++mhz19_misread > MAX_MISREADS_MHZ19)
    {
      //Once in a while, the serial input get shifted by one or more bytes...
	  //Lazy fix: if we don't get valid data for MAX_MISREADS_MHZ19 iterations, just reboot.
      Serial.println("Maximum number of misreads (" + String(MAX_MISREADS_MHZ19) + " reached, resetting...");
      ESP.reset();
    }
    
    return false;
  }

  byte stability = response[5];
  if (stability < 0x40)
  {
    Serial.println("Stability too low: " + String(stability, BIN) + " (0x" + String(stability, HEX) + ")");
    return false;
  }

  int ppm =  (response[2] << 8) | response[3];
  int temp = response[4] - 40;

  //  Serial.println();
  //  Serial.println("BEFORE CHECKSUM!");
  //  Serial.println(" PPM: " + String(ppm));
  //  Serial.println(" Temp: " + String(temp));

  //Checksum is made with byte 1-7 (in the 0-based, 9 byte array)
  byte checksum = getChecksum(response, 1, sizeof(response) - 2);

  //Serial.println();
  //Serial.println(" Checksum - Calculated [" + String(checksum, HEX) + "] vs Received [" + String(response[8], HEX)+ "]");

  if (checksum == response[8])
  {
    oCO2 = ppm;
    oTemp = temp;

    return true;
  }

  return false;
}

byte getChecksum(byte* packet, int startIndex, int len)
{
  byte checksum = 0;

  for (byte i = startIndex; i < startIndex + len; i++) checksum += packet[i];
  checksum = ~checksum + 1;

  return checksum;
}

bool PublishCO2(int iCO2PPM)
{
  if (!mqttClient.connected()) MQTT_Reconnect();
  mqttClient.loop();

  //CO2
  snprintf(message, sizeof(message), "{ \"co2\" : \"%lu\" }", iCO2PPM);
  Serial.println("Publishing [" + String(message) + "] on [" + String(MQTT_OUT_TOPIC_RH) + "]");
  mqttClient.publish(MQTT_OUT_TOPIC_CO2, message);

  return true;
}

bool PublishTemperature(double iTemperature)
{
  if (!mqttClient.connected()) MQTT_Reconnect();
  mqttClient.loop();

  char str_temp[10];
  dtostrf(iTemperature, 4, 1, str_temp);

  //Temperature
  snprintf(message, sizeof(message), "{ \"temperature\" : \"%s\" }", str_temp);
  Serial.println("Publishing [" + String(message) + "] on [" + String(MQTT_OUT_TOPIC_TEMP) + "]");
  mqttClient.publish(MQTT_OUT_TOPIC_TEMP, message);

  return true;
}

bool PublishRH(double iRelativeHumidity)
{
  if (!mqttClient.connected()) MQTT_Reconnect();
  mqttClient.loop();

  char str_rh[10];
  dtostrf(iRelativeHumidity, 4, 1, str_rh);

  //Humidity
  snprintf(message, sizeof(message), "{ \"rh\" : \"%s\" }", str_rh);
  Serial.println("Publishing [" + String(message) + "] on [" + String(MQTT_OUT_TOPIC_RH) + "]");
  mqttClient.publish(MQTT_OUT_TOPIC_RH, message);

  return true;
}

void SetupWifi()
{
  delay(10);
  Serial.println("Connecting to SSID [" + String(WIFI_SSID) + "]");

  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected - IP address: " + WiFi.localIP());
}

void MQTT_Callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived @ [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void MQTT_Reconnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(("WeMosD1_" + String(esp_chipID)).c_str()))
    {
      Serial.println(" CONNECTED!");
      mqttClient.loop();
      mqttClient.subscribe(MQTT_IN_TOPIC);
      mqttClient.publish(MQTT_OUT_TOPIC_GENERAL, "Hi!");
    }
    else
    {
      Serial.print(" FAILED - RC=");
      Serial.print(mqttClient.state());
      Serial.println(". Trying again in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up...");

  esp_chipID = ESP.getChipId();
  Serial.println("Chip ID: " + String(esp_chipID));

  previousMillis = millis();

  dht22Sensor.setup(DHT22_PIN, DHT::DHT22);
  co2Serial.begin(9600);

  SetupWifi();

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(MQTT_Callback);

  Serial.println("Warming up sensors for 10 seconds...");
  delay(10000);
}

void loop()
{
  if (!mqttClient.connected()) {
    MQTT_Reconnect();
  }
  mqttClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < INTERVAL) return;
  previousMillis = currentMillis;

  int ppm = 0;
  int temp = 0;

  double dhttemp;
  double rh;

  Serial.println();

  bool mhz19result = getCO2andTemp(ppm, temp);

  if (mhz19result && (ppm < 5000))
  {
    Serial.println(" PPM: [" + String(ppm) + "] - Temp: [" + String(temp) + "]");
    //Serial.println("Publishing CO2 result...");
    PublishCO2(ppm);
  }
  else Serial.println(" Could not read; discarding data...");

  //DHT22 read-out fails if we read immediately... WHY?
  delay(1000);

  bool dht22result = getTempAndHumidity(dhttemp, rh);

  if (dht22result)
  {
    Serial.println(" Temp: [" + String(dhttemp) + "] - RH: [" + String(rh) + "]");

    PublishTemperature(dhttemp);
    PublishRH(rh);
  }
  else Serial.println(" Error, data could not be read");

  Serial.println();
  Serial.println("---------------------------------------------");
}
