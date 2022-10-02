#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#include "DHT.h"
#define DHTPIN 14     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);


#include "MHZ19.h"
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial
// pin for pwm reading
#define CO2_IN 10

// pin for uart reading
#define RX_PIN D6                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D7                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);

#include "secrets.h"
/* Includes the following:
 * #define WIFI_SSID "..."
 * #define WIFI_PASSWORD "******"
 * #define INFLUXDB_URL "..."
 * #define INFLUXDB_TOKEN "..."
 * #define INFLUXDB_ORG "..."
 * #define INFLUXDB_BUCKET "..."
 */

#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"

// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Data point
Point CO2sensor("CO2_Sensor");
Point DHTsensor("DHT11");
bool sendCO2;
bool sendDHT;

bool errorFlag = false; // If WiFi problems: blink LED

unsigned long startTime = millis ();
unsigned long interval = 60000;

void setup() {
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  //  Serial.println("Preheating 180s");
  //  delay(180000);
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin().

  dht.begin();

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); 
  }
  Serial.println();


  // Add tags
  //CO2sensor.addTag("device", DEVICE);
  //DHTsensor.addTag("device", DEVICE);

  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
void loop() {
  if(errorFlag) {
    digitalWrite(LED_BUILTIN,millis()%500>250);
  }

  if (millis () - startTime >= interval) {
    errorFlag = false;
    digitalWrite(LED_BUILTIN,HIGH);
    startTime = millis ();
    sendCO2 = true;
    sendDHT = true;
    // Clear fields for reusing the point. Tags will remain untouched
    CO2sensor.clearFields();
    DHTsensor.clearFields();
  
    //MH-19Z
    int CO2;
    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
  
    Serial.print("CO2 (ppm): ");
    Serial.println(CO2);
    if (CO2 == 0) {
      sendCO2 = false;
    }
  
    int8_t Temp;
    Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");
    Serial.println(Temp);
  
    // DHT11
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      sendDHT = false;
    }
    float HI = dht.computeHeatIndex(t, h, false);
  
    // Check WiFi connection and reconnect if needed
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Wifi connection lost");
      errorFlag = true;
    }
  
    if (sendCO2) {
      CO2sensor.addField("CO2", CO2);
      CO2sensor.addField("Temperature", Temp);
      Serial.print("Writing CO2: ");
      Serial.println(CO2sensor.toLineProtocol());
      if (!client.writePoint(CO2sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
        errorFlag = true;
      }
  
    }
  
    if (sendDHT) {
      DHTsensor.addField("Temperature", t);
      DHTsensor.addField("Humidity", h);
      DHTsensor.addField("Heat Index", HI);
      Serial.print("Writing DHT11: ");
      Serial.println(DHTsensor.toLineProtocol());
      if (!client.writePoint(DHTsensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
        errorFlag = true;
      }
    }

    Serial.println("Wait for delay");
  }
}
