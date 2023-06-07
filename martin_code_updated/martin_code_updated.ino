/**************************************************************
 *https://github.com/govorox/SSLClient/blob/master/examples/Esp32/mqtt_secure_gsm_SIM7000/mqtt_secure_gsm_SIM7000.ino
 * ESP32 LilyGO-T-SIM7000G Example
 *
 * MQQT (TLS/SLL) with CA Certificate via "TinyGsm.h": https://github.com/vshymanskyy/TinyGSM
 * Tested on Version 20200415
 *
 * About board: https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G
 * About Version 20200415: https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/Historical/SIM7000G_20200415/README.MD
 * Base example:  https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/Arduino_TinyGSM/AllFunctions/AllFunctions.ino
 *                https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/Arduino_NetworkTest/Arduino_NetworkTest.ino
 *                https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/Arduino_Azure_IoTHub/Arduino_Azure_IoTHub.ino
 *                https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_basic/mqtt_basic.ino
 * 
 **************************************************************/
#include "SSLClient.h"
//MQTT Client lib: https://github.com/knolleary/pubsubclient
#include <PubSubClient.h>
#include <Wire.h>
#include <Arduino.h>

//Please enter your CA certificate in ca_cert.h
#include "ca_cert.h"

// ESP32 LilyGO-T-SIM7000G pins definition
// #define MODEM_UART_BAUD 9600
// #define MODEM_DTR 25
// #define MODEM_TX 27
// #define MODEM_RX 26
// #define MODEM_PWKEY 4
// #define LED_PIN 12

////////////////////
#define CANRX GPIO_NUM_35 // Define CAN RX pin (4)
#define CANTX GPIO_NUM_32 // Define CAN TX pin (5)
#define CANSPEED 125E3  //define can speed
#define MODEM_UART_BAUD 9600
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
#define LED_PIN 12

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;
long lastMsg = 0;


// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for AT commands (to the SIM7000 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM7000  // Modem is SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Include after TinyGSM definitions
#include <TinyGsmClient.h>

// Your GPRS credentials (leave empty, if missing)
const char apn[] = "m2m.entel.cl";       // Your APN
const char gprs_user[] = "entelpcs"; // User
const char gprs_pass[] = "entelpcs"; // Password
const char simPIN[] = "";    // SIM card PIN code, if any

// MQTT Config
// EMQX Free Secure Broker for test https://www.emqx.io/mqtt/public-mqtt5-broker
// Check the website to find out more about the available options
// And obtain the current certificate
const char mqtt_client_id[] = "MyEsp32";
const char mqtt_broker[] = "broker.emqx.io";
// TCP-TLS Port
const char* broker = "broker.emqx.io"; // Public IP address or domain name
const char* mqttUsername = "";  // MQTT username
const char* mqttPassword = "";  // MQTT password
int secure_port = 8883; 



const char* topicLed       = "GsmClientTest/led";
const char* topicInit      = "GsmClientTest/init";
const char* topicLedStatus = "GsmClientTest/ledStatus";
const char* topicTemperature = "esp/temperature";
const char* topicCan = "ev01/CAN";

// Layers stack
TinyGsm sim_modem(SerialAT);
TinyGsmClient gsm_transpor_layer(sim_modem);
SSLClient secure_presentation_layer(&gsm_transpor_layer);
PubSubClient mqtt(secure_presentation_layer);

// For read the MQTT events
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// To connect to the broker
void reconnect()
{
  // Loop until we're reconnected
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(mqtt_client_id,mqttUsername, mqttPassword))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqtt.publish("outTopic", "hello world");
      // ... and resubscribe
      mqtt.subscribe("inTopic");
      Serial.println(" success");
      mqtt.publish(topicInit, "GsmClientTest started");
      mqtt.subscribe(topicLed);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println("...try again in 5 seconds");
      delay(5000);
    }
  }
}


void setup()
{
  SerialMon.begin(9600);
  delay(100);
//   pinMode(LED_PIN, OUTPUT);
     // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  SerialMon.println("Wait...");

  // Set SIM module baud rate and UART pins
  SerialAT.begin(MODEM_UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  //Add CA Certificate
  secure_presentation_layer.setCACert(root_ca);



  // MQTT init
  mqtt.setServer(mqtt_broker, secure_port);
  mqtt.setCallback(mqttCallback);
}

void loop()
{
  SerialMon.print("Initializing modem...");
  if (!sim_modem.init())
  {
    SerialMon.print(" fail... restarting modem...");
    
    // Restart takes quite some time
    // Use modem.init() if you don't need the complete restart
    if (!sim_modem.restart())
    {
      SerialMon.println(" fail... even after restart");
      return;
    }
  }
  SerialMon.println(" OK");

  // General information
  String name = sim_modem.getModemName();
  Serial.println("Modem Name: " + name);
  String modem_info = sim_modem.getModemInfo();
  Serial.println("Modem Info: " + modem_info);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && sim_modem.getSimStatus() != 3)
  {
    sim_modem.simUnlock(simPIN);
  }

  sim_modem.setNetworkMode(2);
  delay(3000);

  sim_modem.setPreferredMode(3);
  delay(3000);

  // Wait for network availability
  SerialMon.print("Waiting for network...");
  if (!sim_modem.waitForNetwork())
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  // Connect to the GPRS network
  SerialMon.print("Connecting to network...");
  if (!sim_modem.isNetworkConnected())
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  // Connect to APN
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!sim_modem.gprsConnect(apn, gprs_user, gprs_pass))
  {
    SerialMon.println(" fail");
    return;
  }
  digitalWrite(LED_PIN, HIGH);
  SerialMon.println(" OK");

  // More info..
  Serial.println("");
  String ccid = sim_modem.getSimCCID();
  Serial.println("CCID: " + ccid);
  String imei = sim_modem.getIMEI();
  Serial.println("IMEI: " + imei);
  String cop = sim_modem.getOperator();
  Serial.println("Operator: " + cop);
  IPAddress local = sim_modem.localIP();
  Serial.println("Local IP: " + String(local));
  int csq = sim_modem.getSignalQuality();
  Serial.println("Signal quality: " + String(csq));

  // MQTT Test loop
  // As long as we have connectivity
  while (sim_modem.isGprsConnected())
  {
    // We maintain connectivity with the broker
    if (!mqtt.connected())
    {
      reconnect();
    }

    if (mqtt.publish(topicCan, "Hi EMQX I'm ESP32 ^^")) {
        Serial.println("Message sent successfully");
    } else {
        Serial.println("Message sending failed");
    }
    // We are listening to the events
    mqtt.loop();
  }

  delay(15000);
}
