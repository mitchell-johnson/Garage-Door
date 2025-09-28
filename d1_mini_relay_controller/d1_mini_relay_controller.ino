/*
 * Wemos D1 Mini Pro - Relay Controller for Home Assistant
 * * A robust and non-blocking sketch to control a relay via MQTT, with
 * automatic discovery and state reporting for seamless Home Assistant integration.
 * * Hardware:
 * - Wemos D1 Mini Pro (ESP8266)
 * - Single-channel relay module (active LOW) connected to pin D1.
 * * Version: 1.4
 * Date: 2024-09-29
 */

// =================================================================
// LIBRARIES
// =================================================================
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =================================================================
// CONFIGURATION - UPDATE THESE VALUES
// =================================================================

// --- Serial Debugging ---
// Uncomment the next line to enable detailed serial logging (115200 baud)
#define SERIAL_DEBUG

// --- WiFi Credentials ---
const char* WIFI_SSID = "Johnson";
const char* WIFI_PASSWORD = "aaaaaaaa";

// --- MQTT Broker Configuration ---
const char* MQTT_BROKER_IP = "192.168.1.243";
const int   MQTT_PORT = 1883;
const char* MQTT_USERNAME = ""; // Leave blank if not used
const char* MQTT_PASSWORD = ""; // Leave blank if not used

// --- Home Assistant Device Configuration ---
// This ID must be unique across your Home Assistant instance
const char* DEVICE_UNIQUE_ID = "garage_door"; 
const char* DEVICE_NAME = "Garage Door";

// =================================================================
// HARDWARE & GLOBAL CONSTANTS
// =================================================================

// --- Pinout ---
// Most relay modules are active LOW (LOW turns relay ON).
const int RELAY_PIN = D1; // D1 is GPIO5 on Wemos D1 Mini

// --- Payloads ---
const char* PAYLOAD_ON = "ON";
const char* PAYLOAD_OFF = "OFF";
const char* PAYLOAD_AVAILABLE = "online";
const char* PAYLOAD_NOT_AVAILABLE = "offline";


// --- Timers ---
const unsigned long RECONNECT_INTERVAL_MS = 5000; // Attempt reconnection every 5 seconds

// =================================================================
// GLOBAL VARIABLES
// =================================================================

// --- Network Clients ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- MQTT Topics ---
// These will be constructed in setup() based on DEVICE_UNIQUE_ID
char discoveryTopic[128];
char commandTopic[128];
char stateTopic[128];
char availabilityTopic[128];

// --- State Tracking ---
bool relayState = false; // false = OFF, true = ON
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;

// =================================================================
// FUNCTION DECLARATIONS
// =================================================================

void setupWifi();
void checkConnections();
void reconnectMqtt();
void publishDiscovery();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void setRelayState(bool newState);
void publishRelayState();

// =================================================================
// SETUP
// =================================================================

void setup() {
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
    delay(500); // Allow serial to initialize
    Serial.println("\n[INFO] Booting D1 Mini Relay Controller...");
  #endif

  // --- Configure Relay Pin ---
  pinMode(RELAY_PIN, OUTPUT);
  setRelayState(false); // Set initial state to OFF

  // --- Generate MQTT Topics ---
  snprintf(discoveryTopic, sizeof(discoveryTopic), "homeassistant/switch/%s/config", DEVICE_UNIQUE_ID);
  snprintf(commandTopic, sizeof(commandTopic), "homeassistant/switch/%s/set", DEVICE_UNIQUE_ID);
  snprintf(stateTopic, sizeof(stateTopic), "homeassistant/switch/%s/state", DEVICE_UNIQUE_ID);
  snprintf(availabilityTopic, sizeof(availabilityTopic), "homeassistant/switch/%s/status", DEVICE_UNIQUE_ID);

  #ifdef SERIAL_DEBUG
    Serial.printf("[INFO] Device Name: %s\n", DEVICE_NAME);
    Serial.printf("[INFO] Unique ID:   %s\n", DEVICE_UNIQUE_ID);
    Serial.printf("[INFO] Disc. Topic: %s\n", discoveryTopic);
    Serial.printf("[INFO] Cmd. Topic:  %s\n", commandTopic);
    Serial.printf("[INFO] State Topic: %s\n", stateTopic);
    Serial.printf("[INFO] Avail. Topic: %s\n", availabilityTopic);
  #endif

  // --- Initialize Network ---
  setupWifi();
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  // Increase buffer size to accommodate the large discovery payload
  mqttClient.setBufferSize(1024);

  #ifdef SERIAL_DEBUG
    Serial.println("[INFO] Setup complete. Entering main loop.");
  #endif
}

// =================================================================
// MAIN LOOP
// =================================================================

void loop() {
  // Check WiFi and MQTT connections, and attempt to reconnect if necessary.
  checkConnections();

  // Allow the MQTT client to process incoming messages and maintain its connection.
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

// =================================================================
// NETWORK & CONNECTION FUNCTIONS
// =================================================================

/**
 * @brief Initializes the WiFi connection in a blocking manner.
 */
void setupWifi() {
  #ifdef SERIAL_DEBUG
    Serial.printf("[WIFI] Connecting to SSID: %s\n", WIFI_SSID);
  #endif
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef SERIAL_DEBUG
      Serial.print(".");
    #endif
  }

  #ifdef SERIAL_DEBUG
    Serial.println("\n[WIFI] WiFi connected!");
    Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());
  #endif
}

/**
 * @brief Non-blocking connection manager for WiFi and MQTT.
 * Should be called continuously from the main loop.
 */
void checkConnections() {
  // --- Check WiFi Connection ---
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastWifiReconnectAttempt = millis();
      #ifdef SERIAL_DEBUG
        Serial.println("[WIFI] Connection lost. Reconnecting...");
      #endif
      WiFi.reconnect();
    }
    return; // Stop here if WiFi is not connected
  }

  // --- Check MQTT Connection ---
  if (!mqttClient.connected()) {
    if (millis() - lastMqttReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastMqttReconnectAttempt = millis();
      reconnectMqtt();
    }
  }
}

/**
 * @brief Attempts to connect to the MQTT broker and subscribes to topics.
 */
void reconnectMqtt() {
  #ifdef SERIAL_DEBUG
    Serial.print("[MQTT] Attempting connection...");
  #endif

  bool connected;
  const char* clientId = DEVICE_UNIQUE_ID;

  // LWT will publish "offline" to the availability topic if connection is lost
  if (strlen(MQTT_USERNAME) > 0) {
    connected = mqttClient.connect(clientId, MQTT_USERNAME, MQTT_PASSWORD, availabilityTopic, 0, true, PAYLOAD_NOT_AVAILABLE);
  } else {
    connected = mqttClient.connect(clientId, availabilityTopic, 0, true, PAYLOAD_NOT_AVAILABLE);
  }

  if (connected) {
    #ifdef SERIAL_DEBUG
      Serial.println(" connected!");
      Serial.println("[MQTT-DBG] Connection successful. Beginning post-connection tasks.");
    #endif
    
    yield();

    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT-DBG] Task: Subscribing to command topic...");
    #endif
    if (mqttClient.subscribe(commandTopic)) {
      #ifdef SERIAL_DEBUG
        Serial.printf("[MQTT] Subscribed to: %s\n", commandTopic);
      #endif
    } else {
      #ifdef SERIAL_DEBUG
        Serial.println("[MQTT] ERROR: Subscription failed!");
      #endif
    }
    
    yield();

    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT-DBG] Task: Publishing availability...");
    #endif
    mqttClient.publish(availabilityTopic, PAYLOAD_AVAILABLE, true);
    
    yield();

    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT-DBG] Task: Publishing discovery payload...");
    #endif
    publishDiscovery();

    yield();

    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT-DBG] Task: Publishing initial state...");
    #endif
    publishRelayState();

    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT-DBG] Post-connection tasks complete.");
    #endif

  } else {
    #ifdef SERIAL_DEBUG
      Serial.printf(" failed, rc=%d. Will try again in %d seconds.\n", mqttClient.state(), RECONNECT_INTERVAL_MS / 1000);
    #endif
  }
}

// =================================================================
// MQTT PUBLISH & CALLBACK FUNCTIONS
// =================================================================

/**
 * @brief Publishes the Home Assistant autodiscovery configuration payload.
 */
void publishDiscovery() {
  // Use StaticJsonDocument for fixed-size payloads to avoid heap fragmentation.
  StaticJsonDocument<1024> doc;

  // Configure the switch entity
  doc["name"] = DEVICE_NAME;
  doc["unique_id"] = DEVICE_UNIQUE_ID;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["availability_topic"] = availabilityTopic;
  doc["payload_on"] = PAYLOAD_ON;
  doc["payload_off"] = PAYLOAD_OFF;
  doc["optimistic"] = false;

  // Add minimal device information
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = DEVICE_UNIQUE_ID;
  device["name"] = DEVICE_NAME;
  
  String payload;
  serializeJson(doc, payload);

  #ifdef SERIAL_DEBUG
    Serial.println("[MQTT] Publishing discovery payload:");
    Serial.println(payload);
  #endif
  
  if (!mqttClient.publish(discoveryTopic, payload.c_str(), true)) {
    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT] ERROR: Failed to publish discovery payload!");
    #endif
  }
}

/**
 * @brief Handles incoming MQTT messages.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Null-terminate the payload to safely treat it as a C-string.
  payload[length] = '\0';
  char* message = (char*)payload;

  #ifdef SERIAL_DEBUG
    Serial.printf("[MQTT] Message received on topic: %s\n", topic);
    Serial.printf("[MQTT] Payload: %s\n", message);
  #endif

  // Check if the message is for our command topic
  if (strcmp(topic, commandTopic) == 0) {
    if (strcmp(message, PAYLOAD_ON) == 0) {
      setRelayState(true);
    } else if (strcmp(message, PAYLOAD_OFF) == 0) {
      setRelayState(false);
    } else {
      #ifdef SERIAL_DEBUG
        Serial.printf("[MQTT] WARNING: Unknown payload received: %s\n", message);
      #endif
    }
  }
}

/**
 * @brief Publishes the current relay state to the state topic.
 */
void publishRelayState() {
  if (!mqttClient.connected()) return;

  const char* currentState = relayState ? PAYLOAD_ON : PAYLOAD_OFF;
  
  // Create JSON state payload
  StaticJsonDocument<64> stateDoc;
  stateDoc["state"] = currentState;
  
  String statePayload;
  serializeJson(stateDoc, statePayload);
  
  #ifdef SERIAL_DEBUG
    Serial.printf("[MQTT] Publishing state: %s\n", statePayload.c_str());
  #endif
  
  if (!mqttClient.publish(stateTopic, statePayload.c_str(), true)) {
     #ifdef SERIAL_DEBUG
      Serial.println("[MQTT] ERROR: Failed to publish state!");
    #endif
  }
}

// =================================================================
// RELAY CONTROL
// =================================================================

/**
 * @brief Sets the physical relay state and publishes the change.
 * @param newState The desired state: true for ON, false for OFF.
 */
void setRelayState(bool newState) {
  // Only act if the state is actually changing
  if (newState == relayState) {
    return;
  }
  
  relayState = newState;
  
  // Most relay modules are "active LOW", meaning a LOW signal turns them ON.
  digitalWrite(RELAY_PIN, relayState ? LOW : HIGH);
  
  #ifdef SERIAL_DEBUG
    Serial.printf("[RELAY] State changed to: %s\n", relayState ? "ON" : "OFF");
  #endif

  // Report the new state back to Home Assistant
  publishRelayState();
}

