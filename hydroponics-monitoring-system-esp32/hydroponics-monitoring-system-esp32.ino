#include "Adafruit_HTU21DF.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define ONE_WIRE_BUS 26
#define TdsSensorPin 39

#define VREF 3.3   // analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // sum of sample point

#define analogPin 26  // pH

Adafruit_HTU21DF htu = Adafruit_HTU21DF();
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char* ssid = "#####";
const char* password = "#####";
const char* mqttServer = "#####";
const char* mqttUser = "#####";
const char* mqttPassword = "#####";
const char* mqttTopic = "#####";
const char* device = "#####";
const int mqttPort = 1883;

const int led_pin = 14;
const int mqtt_pin = 27;
const int ph_pin = 16;

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
int pHSense = 35;
int samples = 10;

float adc_resolution = 4095.0;  //ESP 32 ADC Resolution
float averageVoltage = 0;
float tdsValue = 0;
float ECValue = 0;
float temp = 0;
float tempS = 0;
float hum = 0;
float pHcomp = 0;

TaskHandle_t mqttWifiTaskHandle = NULL;

void setup() {
  Serial.begin(115200);
  sensors.begin();
  pinMode(TdsSensorPin, INPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(mqtt_pin, OUTPUT);
  pinMode(ph_pin, OUTPUT);
  digitalWrite(ph_pin, LOW);
  digitalWrite(mqtt_pin, LOW);
  digitalWrite(led_pin, HIGH);

  xTaskCreatePinnedToCore(
    mqttWifiTask,         // Task function
    "mqttWifiTask",       // Task name
    10000,                // Stack size
    NULL,                 // Task parameters
    1,                    // Task priority
    &mqttWifiTaskHandle,  // Task handle
    0                     // Core number
  );

  if (!htu.begin()) {
    Serial.println("Check circuit. HTU21D not found!");
    while (1)
      ;
  }
}

float ph(float voltage) {
  return 7 + ((2.50 - voltage) / 0.18);
}


void loop() {
  readTempSolution(tempS);
  readGY21(temp, hum);
  readPH(pHcomp);
  getTDSValue(tdsValue);
  ECValue = tdsValue / 500;

  Serial.print("voltage:");
  Serial.print(averageVoltage, 2);
  Serial.print("\t");

  Serial.print("Solution Temperature(°C): ");
  Serial.print(tempS);
  Serial.print("\t");

  Serial.print("EC: ");
  Serial.print(ECValue, 3);
  Serial.print("\t");

  // Serial.print("TDS Value(ppm): ");
  // Serial.print(tdsValue, 0);
  // Serial.print("\t");

  Serial.print("Temperature(°C): ");
  Serial.print(temp);
  Serial.print("\t");

  Serial.print("Humidity(%): ");
  Serial.print(hum);
  Serial.print("\t");

  Serial.print("pH: ");
  Serial.println(pHcomp);


  delay(1000);
}

void readTempSolution(float& tempS) {
  sensors.requestTemperatures();
  float newTemp = sensors.getTempCByIndex(0);
  if (newTemp == DEVICE_DISCONNECTED_C) {
    tempS = 22;  // Set temperature to 22°C if sensor is disconnected
  } else {
    tempS = newTemp;
  }
}

void readGY21(float& temp, float& hum) {
  temp = htu.readTemperature();
  hum = htu.readHumidity();
}

void readPH(float& pHcomp) {
  int num_samples = 60;  // number of samples to take
  int readings[num_samples];
  int total_readings = 0;
  for (int i = 0; i < num_samples; i++) {
    readings[i] = analogRead(pHSense);
    delay(10);
    total_readings += readings[i];
  }
  float voltage = 3.3 / adc_resolution * total_readings / num_samples;
  float pHvalue = ph(voltage);
  pHcomp = pHvalue + 0.03 * (tempS - 25.0);

  if (pHcomp < 5.5 || pHcomp > 6.5) { // LED pH alarm
    digitalWrite(ph_pin, HIGH);
  } else {
    digitalWrite(ph_pin, LOW);
  }
}

void getTDSValue(float& tdsValue) {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1 + 0.02 * (tempS - 25);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;
      //convert voltage value to tds value
      tdsValue = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;
    }
  }
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void mqttWifiTask(void* parameter) {
  initWifi();
  initMqtt();
  while (1) {
    if (!mqttClient.connected()) {
      Serial.println("MQTT connection lost. Reconnecting...");
      initMqtt(); // try to reconnect to the MQTT broker
    } else {
      mqttClient.loop();
      
    // String data = "{\"sol_temperature\": " + String(tempS) + ", \"tds\": " + String(tdsValue, 0) + ", \"ph\": " + String(pHcomp) + ", \"temperature\": " + String(temp) + ", \"humidity\": " + String(hum) + ", \"device_name\": \"" + String(device) + "\"}";
    String data = "{\"sol_temperature\": " + String(tempS) + ", \"ec\": " + String(ECValue, 3) + ", \"ph\": " + String(pHcomp) + ", \"temperature\": " + String(temp) + ", \"humidity\": " + String(hum) + ", \"device_name\": \"" + String(device) + "\"}";

    mqttClient.publish(mqttTopic, data.c_str());
    Serial.println("Data sent to MQTT broker: " + data);
    digitalWrite(mqtt_pin, HIGH);
    delay(100);
    digitalWrite(mqtt_pin, LOW);

    delay(4900);
    }    
  }
}

void initWifi() {
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);  // enable auto-reconnect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void initMqtt() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);  // set keep alive time
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker.");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle MQTT callback if needed
}
