/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cloud-mqtt-broker-sim800l/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L
//#define TINY_GSM_MODEM_SIM7000
//#define TINY_GSM_MODEM_SIM7000SSL

//#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "m2m.entel.cl";
const char gprsUser[] = "entelpcs";
const char gprsPass[] = "entelpcs";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";

// MQTT details
const char* broker = "broker.emqx.io"; // Public IP address or domain name
const char* mqttUsername = "";  // MQTT username
const char* mqttPassword = "";  // MQTT password
const char* mqtt_client_id = "ESP32_SIM7000";
const int mqtt_port = 8883;
const char* topicLed       = "GsmClientTest/led";
const char* topicInit      = "GsmClientTest/init";
const char* topicLedStatus = "GsmClientTest/ledStatus";
const char* topicTemperature = "esp/temperature";
const char* topicCan = "GsmClientTest/CAN";

// load DigiCert Global Root CA ca_cert
const char* ca_cert= \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=" \
"-----END CERTIFICATE-----\n";



// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
//#include <Wifi.h>  //try if tinygsmclientsecure doesn't work
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// ----------------------------------------------------------Libraries--------------------------


// EVS CANbus libraries and pins
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <CAN_evs.h>
#include <HardwareSerial.h>

// ESP SD and Json libraries
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

// ----------------------------------------------------------Pins--------------------------
#define CANRX GPIO_NUM_35 // Define CAN RX pin (4)
#define CANTX GPIO_NUM_32 // Define CAN TX pin (5)
#define CANSPEED 125E3  //define can speed

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
#define LED_PIN 13

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

float temperature = 0;
float humidity = 0;
long lastMsg = 0;

const int canSpeed = 500000;

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("ESP32_SIM800l", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.publish(topicInit, "GsmClientTest started");
  mqtt.subscribe(topicLed);

  return mqtt.connected();
}

// ----------------------------------------------------------void setup--------------------------

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  pinMode(LED_PIN, OUTPUT);

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();
  delay(6000);
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setCACert(ca_cert);
  mqtt.setServer(broker, mqtt_port);
  mqtt.setCallback(mqttCallback);

  // ----------------------------------------------------ESP32 CANbus Setup--------------------
  //Serial.begin(115200);
  //Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(20);
  while (!Serial);

  Serial.println("CAN Receiver Callback");
  Serial.print("Setting up CAN interface for ");
  Serial.print((int)CANSPEED);
  Serial.print(" baud rate \r\n");

  CAN.setPins(CANRX, CANTX);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(CANSPEED)) {
    Serial.println("Starting CAN failed!");
    while (1);
  } else {
    Serial.println("CAN bus initialized");
  }

  CAN.filter(0x106);



}

// ----------------------------------------------------------void loop--------------------------

void loop() {

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  // register the receive callback
  CAN.onReceive(onReceive);
  mqtt.loop();
}

void onReceive(int packetSize) {
  packetSize = CAN.parsePacket();

  Serial.print(CAN.packetId(), HEX);
  while (CAN.available()) {
    Serial.print(CAN.read(), HEX);
    // Read data from CAN bus
//    int canId;
//    byte canData[8];
//    byte len = 0;
//    if (CAN.parsePacket()) {
//      canId = CAN.packetId();
//      if (packetSize) {
//        int n = 0;
//        while (n < 8) {
//          if (Serial1.available()) {
//            canData[n++] = CAN.read();
//          } else {
//            break;
//          }
//
//          // Convert data to JSON
//          DynamicJsonDocument doc(200);
//          doc["canId"] = canId;
//          JsonArray dataArray = doc.createNestedArray("data");
//          for (int i = 0; i < len; i++) {
//            dataArray.add(canData[i]);
//          }
//          char json[200];
//          serializeJson(doc, json);
//          mqtt.publish(topicCan, json);
//        }
//      }
//    }
  }
  Serial.println();
}
