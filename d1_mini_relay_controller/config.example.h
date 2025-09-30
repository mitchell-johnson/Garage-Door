/*
 * Example Configuration File
 * Copy this file to config.h and update with your settings
 *
 * Instructions:
 * 1. Copy this file: cp config.example.h config.h
 * 2. Edit config.h with your WiFi, MQTT, and device settings
 * 3. Upload to your ESP8266 device
 */

#ifndef CONFIG_H
#define CONFIG_H

// =================================================================
// SERIAL DEBUGGING
// =================================================================

// Uncomment the next line to enable detailed serial logging (115200 baud)
// This is helpful for troubleshooting but can be disabled in production
#define SERIAL_DEBUG

// =================================================================
// WIFI CREDENTIALS
// =================================================================

const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// =================================================================
// MQTT BROKER CONFIGURATION
// =================================================================

const char* MQTT_BROKER_IP = "192.168.1.100";  // IP address of your MQTT broker
const int   MQTT_PORT = 1883;                   // Default MQTT port
const char* MQTT_USERNAME = "";                 // Leave blank if authentication not required
const char* MQTT_PASSWORD = "";                 // Leave blank if authentication not required

// =================================================================
// HOME ASSISTANT DEVICE CONFIGURATION
// =================================================================

// This ID must be unique across your Home Assistant instance
// It will be used to generate MQTT topics and identify the device
const char* DEVICE_UNIQUE_ID = "garage_door";
const char* DEVICE_NAME = "Garage Door";

// =================================================================
// DOOR SENSOR CONFIGURATION
// =================================================================

// MQTT topic for the door sensor that reports the actual door state
// Expected JSON format: {"device": "binary_sensor.door", "state_reported": "off", "STATE": "CLOSED"}
// STATE values: "OPEN" or "CLOSED"
const char* DOOR_SENSOR_TOPIC = "homeassistant/binary_sensor/binary_sensor.contact_sensor_door/state";

#endif
