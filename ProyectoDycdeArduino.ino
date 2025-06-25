#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <FastLED.h>

// WiFi
const char* ssid = "Leonel";
const char* password = "rkhmnuxffq56cm6";

// MQTT
const char* mqtt_server = "galiot.galileo.edu";
const char* passwd = "MONair2023";
//const char* mqtt_server = "test.mosquitto.org";
//const char* passwd = "";
const char *user = "monair";
WiFiClient espClient;
PubSubClient client(espClient);

// Pines sensores
#define TEMP_OUT_PIN     19   // DS18B20
#define A_OUT_GAS_PIN    14   // MQ-2 analógica
#define BME_SDA          21
#define BME_SCL          22

// Pines de salida
#define OUT_RELE_GAS_PIN   2
#define OUT_RELETEMP_PIN  13
#define ALERT_GAS_PIN     18

// LED Neopixels con FastLED
#define LEDS_TEMP_PIN     12
#define LEDS_GAS_PIN      4
#define NUMPIXELS         2
CRGB ledsTemp[NUMPIXELS];
CRGB ledsGas[NUMPIXELS];

// Umbrales
const int GAS_LIMIT_PERCENTAGE = 40;
const float TEMP_BME_HIGH = 40.0;   // °C
const float TEMP_BME_LOW  = 30.0;   // °C

// --- DS18B20
OneWire oneWire(TEMP_OUT_PIN);
DallasTemperature ds18b20(&oneWire);
float tempDS18B20 = 0.0;

// --- BME680
Adafruit_BME680 bme;
float tempBME = 0.0, humBME = 0.0, presBME = 0.0, altBME = 0.0;
bool bmeOk = false;

// --- MQ-2
int gasAnalogValue = 0;
float gasPercentage = 0.0;

// --- Funciones
void setupWiFi() {
  delay(10);
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Leo",user,passwd)) {
      Serial.println("Conectado a MQTT");
    } else {
      Serial.print(" fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 1s");
      delay(1000);
    }
  }
}

void setupOutputs() {
  pinMode(OUT_RELE_GAS_PIN, OUTPUT);
  pinMode(OUT_RELETEMP_PIN, OUTPUT);
  pinMode(ALERT_GAS_PIN, OUTPUT);
  pinMode(A_OUT_GAS_PIN, INPUT);
}

void setupDS18B20() {
  ds18b20.begin();
}

bool setupBME680() {
  Wire.begin(BME_SDA, BME_SCL);
  if (!bme.begin(0x76,&Wire)){
    Serial.println("No se encontró el BME680, :(");
    while(1);
  } // si tu sensor está en 0x76
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  return true;
}

bool readBME680() {
  if (!bme.performReading()) return false;
  tempBME = bme.temperature;
  humBME = bme.humidity;
  presBME = bme.pressure / 100.0;
  altBME = 44330.0 * (1.0 - pow(presBME / 1013.25, 1.0 / 5.255));
  return true;
}

void readDS18B20() {
  ds18b20.requestTemperatures();
  tempDS18B20 = ds18b20.getTempCByIndex(0);
}

void readGasMQ2() {
  gasAnalogValue = analogRead(A_OUT_GAS_PIN);
  gasPercentage = map(gasAnalogValue, 0, 4095, 0, 100);
}

void setupLEDs() {
  FastLED.addLeds<NEOPIXEL, LEDS_TEMP_PIN>(ledsTemp, NUMPIXELS);
  FastLED.addLeds<NEOPIXEL, LEDS_GAS_PIN>(ledsGas, NUMPIXELS);
  FastLED.clear();
  FastLED.show();
}

void setStripColor(CRGB* leds, CRGB color) {
  for (int i = 0; i < NUMPIXELS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void controlOutputs() {
  // --- GAS
  if (gasPercentage >= GAS_LIMIT_PERCENTAGE) {
    digitalWrite(OUT_RELE_GAS_PIN, HIGH);
    digitalWrite(ALERT_GAS_PIN, HIGH);
    setStripColor(ledsGas, CRGB::Red);
  } else {
    digitalWrite(OUT_RELE_GAS_PIN, LOW);
    digitalWrite(ALERT_GAS_PIN, LOW);
    setStripColor(ledsGas, CRGB::Green);
  }

  // --- TEMP BME680
  if (bmeOk) {
    if (tempBME >= TEMP_BME_HIGH) {
      digitalWrite(OUT_RELETEMP_PIN, HIGH);
      setStripColor(ledsTemp, CRGB::Red);
    } else if (tempBME <= TEMP_BME_LOW) {
      digitalWrite(OUT_RELETEMP_PIN, LOW);
      setStripColor(ledsTemp, CRGB::Blue);
    } else {
      digitalWrite(OUT_RELETEMP_PIN, LOW);
      setStripColor(ledsTemp, CRGB::Green);
    }
  }
}

void publishMQTT() {
  char buffer[50];

  snprintf(buffer, sizeof(buffer), "%.2f", tempDS18B20);
  client.publish("sensor/temp_out", buffer);

  if (bmeOk) {
    snprintf(buffer, sizeof(buffer), "%.2f", tempBME);
    client.publish("sensor/bme680/temperature", buffer);

    snprintf(buffer, sizeof(buffer), "%.2f", humBME);
    client.publish("sensor/bme680/humidity", buffer);

    snprintf(buffer, sizeof(buffer), "%.2f", presBME);
    client.publish("sensor/bme680/pressure", buffer);

    snprintf(buffer, sizeof(buffer), "%.2f", altBME);
    client.publish("sensor/bme680/altitude", buffer);
  }

  snprintf(buffer, sizeof(buffer), "%.2f", gasPercentage);
  client.publish("sensor/gas_percentage", buffer);
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  client.setServer(mqtt_server, 1883);
  setupOutputs();
  setupDS18B20();
  setupLEDs();

  bmeOk = setupBME680();
  if (!bmeOk) {
    Serial.println("Error al iniciar BME680");
  }
}


void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  readDS18B20();
  if (bmeOk) readBME680();
  readGasMQ2();
  controlOutputs();
  publishMQTT();

  delay(2000);  // Cada 2 segundos
}