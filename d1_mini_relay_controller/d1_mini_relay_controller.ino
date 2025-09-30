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
// CONFIGURATION
// =================================================================

// Try to include local config.h file (not tracked in git)
// If it doesn't exist, you'll get a compilation error with instructions
#if __has_include("config.h")
  #include "config.h"
#else
  #error "config.h not found! Please copy config.example.h to config.h and update with your settings"
#endif

// =================================================================
// HARDWARE & GLOBAL CONSTANTS
// =================================================================

// --- Pinout ---
// This relay shield is active HIGH (HIGH turns relay ON).
const int RELAY_PIN = D1; // D1 is GPIO5 on Wemos D1 Mini

// --- Payloads ---
const char* PAYLOAD_OPEN = "open";
const char* PAYLOAD_CLOSE = "close";
const char* PAYLOAD_STOP = "stop";
const char* STATE_OPEN = "open";
const char* STATE_CLOSED = "closed";
const char* STATE_OPENING = "opening";
const char* STATE_CLOSING = "closing";
const char* PAYLOAD_AVAILABLE = "online";
const char* PAYLOAD_NOT_AVAILABLE = "offline";


// --- Timers ---
const unsigned long RECONNECT_INTERVAL_MS = 5000; // Attempt reconnection every 5 seconds
const unsigned long TRANSITION_TIMEOUT_MS = 60000; // 1 minute timeout for opening/closing states

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
String coverState = STATE_CLOSED; // Current cover state: open, closed, opening, closing
String lastConfirmedState = STATE_CLOSED; // Last confirmed state from door sensor
unsigned long transitionStartTime = 0; // Time when transition started
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
void publishCoverState();
void handleCoverCommand(const char* command);
void handleDoorSensorUpdate(const char* sensorState);
void checkTransitionTimeout();

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
  snprintf(discoveryTopic, sizeof(discoveryTopic), "homeassistant/cover/%s/config", DEVICE_UNIQUE_ID);
  snprintf(commandTopic, sizeof(commandTopic), "homeassistant/cover/%s/set", DEVICE_UNIQUE_ID);
  snprintf(stateTopic, sizeof(stateTopic), "homeassistant/cover/%s/state", DEVICE_UNIQUE_ID);
  snprintf(availabilityTopic, sizeof(availabilityTopic), "homeassistant/cover/%s/status", DEVICE_UNIQUE_ID);

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

  // Check if we've been in a transitional state too long
  checkTransitionTimeout();
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
      Serial.println("[MQTT-DBG] Task: Subscribing to door sensor topic...");
    #endif
    if (mqttClient.subscribe(DOOR_SENSOR_TOPIC)) {
      #ifdef SERIAL_DEBUG
        Serial.printf("[MQTT] Subscribed to: %s\n", DOOR_SENSOR_TOPIC);
      #endif
    } else {
      #ifdef SERIAL_DEBUG
        Serial.println("[MQTT] ERROR: Door sensor subscription failed!");
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
    publishCoverState();

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

  // Configure the cover entity
  doc["name"] = DEVICE_NAME;
  doc["unique_id"] = DEVICE_UNIQUE_ID;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["availability_topic"] = availabilityTopic;
  doc["payload_open"] = PAYLOAD_OPEN;
  doc["payload_close"] = PAYLOAD_CLOSE;
  doc["payload_stop"] = PAYLOAD_STOP;
  doc["state_open"] = STATE_OPEN;
  doc["state_closed"] = STATE_CLOSED;
  doc["state_opening"] = STATE_OPENING;
  doc["state_closing"] = STATE_CLOSING;
  doc["device_class"] = "garage";
  doc["optimistic"] = false;
  doc["retain"] = true;

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
  // Create a safe buffer for the payload to avoid overwriting
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  #ifdef SERIAL_DEBUG
    Serial.printf("[MQTT] Message received on topic: %s\n", topic);
    Serial.printf("[MQTT] Payload: %s\n", message);
  #endif

  // Check if the message is for our command topic
  if (strcmp(topic, commandTopic) == 0) {
    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT] Command topic matched, processing cover command...");
    #endif
    handleCoverCommand(message);
  }
  // Check if the message is from the door sensor
  else if (strcmp(topic, DOOR_SENSOR_TOPIC) == 0) {
    #ifdef SERIAL_DEBUG
      Serial.println("[MQTT] Door sensor topic matched, processing sensor update...");
    #endif

    // Parse JSON: {"device": "binary_sensor.ranch_slider_door", "state_reported": "off", "STATE": "CLOSED"}
    StaticJsonDocument<256> sensorDoc;
    DeserializationError error = deserializeJson(sensorDoc, message);

    if (error) {
      #ifdef SERIAL_DEBUG
        Serial.printf("[MQTT] JSON parse error: %s\n", error.c_str());
      #endif
      return;
    }

    const char* sensorState = sensorDoc["STATE"];
    if (sensorState) {
      handleDoorSensorUpdate(sensorState);
    } else {
      #ifdef SERIAL_DEBUG
        Serial.println("[MQTT] WARNING: No 'STATE' field in door sensor JSON");
      #endif
    }
  }
}

/**
 * @brief Publishes the current cover state to the state topic.
 */
void publishCoverState() {
  if (!mqttClient.connected()) return;

  #ifdef SERIAL_DEBUG
    Serial.printf("[MQTT] Publishing cover state: %s\n", coverState.c_str());
  #endif

  // Publish with retain=true
  if (!mqttClient.publish(stateTopic, coverState.c_str(), true)) {
     #ifdef SERIAL_DEBUG
      Serial.println("[MQTT] ERROR: Failed to publish state!");
    #endif
  }
}

/**
 * @brief Handles cover commands from Home Assistant (open/close/stop).
 */
void handleCoverCommand(const char* command) {
  #ifdef SERIAL_DEBUG
    Serial.printf("[COVER] Received command: %s\n", command);
  #endif

  if (strcmp(command, PAYLOAD_OPEN) == 0) {
    // Only trigger if not already open or opening
    if (coverState != STATE_OPEN && coverState != STATE_OPENING) {
      #ifdef SERIAL_DEBUG
        Serial.println("[COVER] Opening door...");
      #endif
      coverState = STATE_OPENING;
      transitionStartTime = millis(); // Start transition timer
      publishCoverState();
      setRelayState(true);
      // Relay will pulse briefly (turned off in setRelayState logic)
    }
  }
  else if (strcmp(command, PAYLOAD_CLOSE) == 0) {
    // Only trigger if not already closed or closing
    if (coverState != STATE_CLOSED && coverState != STATE_CLOSING) {
      #ifdef SERIAL_DEBUG
        Serial.println("[COVER] Closing door...");
      #endif
      coverState = STATE_CLOSING;
      transitionStartTime = millis(); // Start transition timer
      publishCoverState();
      setRelayState(true);
      // Relay will pulse briefly (turned off in setRelayState logic)
    }
  }
  else if (strcmp(command, PAYLOAD_STOP) == 0) {
    #ifdef SERIAL_DEBUG
      Serial.println("[COVER] Stop command - triggering relay pulse");
    #endif
    setRelayState(true);
  }
}

/**
 * @brief Handles door sensor state updates and syncs cover state.
 */
void handleDoorSensorUpdate(const char* sensorState) {
  #ifdef SERIAL_DEBUG
    Serial.printf("[SENSOR] Door sensor state: %s\n", sensorState);
  #endif

  // Convert sensor STATE to cover state
  String newState;
  if (strcmp(sensorState, "OPEN") == 0) {
    newState = STATE_OPEN;
  } else if (strcmp(sensorState, "CLOSED") == 0) {
    newState = STATE_CLOSED;
  } else {
    #ifdef SERIAL_DEBUG
      Serial.printf("[SENSOR] WARNING: Unknown sensor state: %s\n", sensorState);
    #endif
    return;
  }

  // Update last confirmed state
  lastConfirmedState = newState;

  // If we're in a transitional state, move to the confirmed state
  if (coverState == STATE_OPENING || coverState == STATE_CLOSING) {
    #ifdef SERIAL_DEBUG
      Serial.printf("[SENSOR] Transitioning from %s to %s\n", coverState.c_str(), newState.c_str());
    #endif
    coverState = newState;
    transitionStartTime = 0; // Clear transition timer
    publishCoverState();
  }
  // If sensor state differs from current state, update to sensor state
  else if (coverState != newState) {
    #ifdef SERIAL_DEBUG
      Serial.printf("[SENSOR] State mismatch! Updating from %s to %s\n", coverState.c_str(), newState.c_str());
    #endif
    coverState = newState;
    publishCoverState();
  }
}

/**
 * @brief Checks if we've been in a transitional state too long and times out to sensor state.
 */
void checkTransitionTimeout() {
  // Only check if we're in a transitional state
  if (coverState != STATE_OPENING && coverState != STATE_CLOSING) {
    return;
  }

  // Check if transition timer is set and has expired
  if (transitionStartTime > 0 && (millis() - transitionStartTime > TRANSITION_TIMEOUT_MS)) {
    #ifdef SERIAL_DEBUG
      Serial.printf("[TIMEOUT] Transition timeout! Moving from %s to sensor state: %s\n",
                    coverState.c_str(), lastConfirmedState.c_str());
    #endif

    coverState = lastConfirmedState;
    transitionStartTime = 0; // Clear transition timer
    publishCoverState();
  }
}

// =================================================================
// RELAY CONTROL
// =================================================================

/**
 * @brief Sets the physical relay state (momentary pulse for garage door).
 * @param newState The desired state: true for ON, false for OFF.
 */
void setRelayState(bool newState) {
  #ifdef SERIAL_DEBUG
    Serial.printf("[RELAY] Setting relay state to %s\n", newState ? "ON" : "OFF");
  #endif

  relayState = newState;

  // This relay shield is "active HIGH", meaning a HIGH signal turns it ON.
  int pinValue = relayState ? HIGH : LOW;
  digitalWrite(RELAY_PIN, pinValue);

  #ifdef SERIAL_DEBUG
    Serial.printf("[RELAY] Pin D1 (GPIO5) set to: %s\n", pinValue == LOW ? "LOW" : "HIGH");
  #endif

  // For garage door opener, we want a momentary pulse
  // If turning ON, schedule to turn OFF after a brief delay
  if (relayState) {
    delay(500); // 500ms pulse
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    #ifdef SERIAL_DEBUG
      Serial.println("[RELAY] Pulse complete, relay turned OFF");
    #endif
  }
}

