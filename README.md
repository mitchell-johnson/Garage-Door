# Garage Door Controller

A robust, non-blocking ESP8266-based garage door controller with Home Assistant integration via MQTT. This project provides reliable remote control of your garage door with real-time state monitoring and automatic discovery.

## Features

- **ESP8266 Based**: Uses Wemos D1 Mini Pro for WiFi connectivity and relay control
- **Home Assistant Integration**: Full MQTT autodiscovery support for seamless integration
- **Real-time State Monitoring**: Door sensor integration for accurate open/closed status
- **Robust Error Handling**: Automatic reconnection for WiFi and MQTT connections
- **Non-blocking Architecture**: Maintains responsiveness during network operations
- **Momentary Relay Control**: Safe pulse-based operation for garage door openers
- **Timeout Protection**: Automatic state recovery if transitions take too long

## Hardware Requirements

### Core Components
- **Wemos D1 Mini Pro** (ESP8266-based microcontroller)
- **Single-channel relay module** (active HIGH configuration)
- **Door sensor** (optional but recommended for state feedback)

### Wiring
- **Relay Control**: Connect relay signal pin to `D1` (GPIO5)
- **Power**: 5V and GND connections to relay module
- **Door Sensor**: Separate sensor system reporting to MQTT (not directly wired to ESP8266)

## Software Dependencies

### Arduino Libraries
- `ESP8266WiFi.h` - WiFi connectivity
- `PubSubClient.h` - MQTT client functionality
- `ArduinoJson.h` - JSON parsing and generation for MQTT payloads

## Configuration

### WiFi Settings
```cpp
const char* WIFI_SSID = "YourNetworkName";
const char* WIFI_PASSWORD = "YourPassword";
```

### MQTT Broker Configuration
```cpp
const char* MQTT_BROKER_IP = "192.168.1.243";  // Your MQTT broker IP
const int   MQTT_PORT = 1883;
const char* MQTT_USERNAME = "";  // Leave blank if not using authentication
const char* MQTT_PASSWORD = "";  // Leave blank if not using authentication
```

### Device Identity
```cpp
const char* DEVICE_UNIQUE_ID = "garage_door";  // Must be unique in Home Assistant
const char* DEVICE_NAME = "Garage Door";       // Friendly name displayed in HA
```

### Door Sensor Integration
```cpp
const char* DOOR_SENSOR_TOPIC = "homeassistant/binary_sensor/binary_sensor.contact_sensor_door/state";
```

## MQTT Implementation Details

### Topic Structure
The system automatically generates MQTT topics based on the `DEVICE_UNIQUE_ID`:

- **Discovery Topic**: `homeassistant/cover/{DEVICE_UNIQUE_ID}/config`
- **Command Topic**: `homeassistant/cover/{DEVICE_UNIQUE_ID}/set`
- **State Topic**: `homeassistant/cover/{DEVICE_UNIQUE_ID}/state`
- **Availability Topic**: `homeassistant/cover/{DEVICE_UNIQUE_ID}/status`

### Home Assistant Autodiscovery
The system publishes a comprehensive discovery payload that includes:

```json
{
  "name": "Garage Door",
  "unique_id": "garage_door",
  "command_topic": "homeassistant/cover/garage_door/set",
  "state_topic": "homeassistant/cover/garage_door/state",
  "availability_topic": "homeassistant/cover/garage_door/status",
  "payload_open": "open",
  "payload_close": "close",
  "payload_stop": "stop",
  "state_open": "open",
  "state_closed": "closed",
  "state_opening": "opening",
  "state_closing": "closing",
  "device_class": "garage",
  "optimistic": false,
  "retain": true,
  "device": {
    "identifiers": ["garage_door"],
    "name": "Garage Door"
  }
}
```

### Connection Management
The system implements robust connection handling:

- **WiFi Reconnection**: Automatic reconnection every 5 seconds if connection is lost
- **MQTT Reconnection**: Non-blocking reconnection attempts with exponential backoff
- **Last Will Testament**: Publishes "offline" to availability topic if connection is lost unexpectedly

## Open/Close State Logic

### State Machine Overview
The controller implements a comprehensive state machine with four primary states:

1. **`open`** - Door is fully open and confirmed by sensor
2. **`closed`** - Door is fully closed and confirmed by sensor
3. **`opening`** - Door is transitioning from closed to open
4. **`closing`** - Door is transitioning from open to closed

### State Transitions

#### Command Processing
When receiving commands via MQTT:

**OPEN Command (`"open"`):**
```cpp
if (coverState != STATE_OPEN && coverState != STATE_OPENING) {
    coverState = STATE_OPENING;
    transitionStartTime = millis();  // Start timeout timer
    publishCoverState();             // Notify Home Assistant
    setRelayState(true);            // Pulse relay (500ms)
}
```

**CLOSE Command (`"close"`):**
```cpp
if (coverState != STATE_CLOSED && coverState != STATE_CLOSING) {
    coverState = STATE_CLOSING;
    transitionStartTime = millis();  // Start timeout timer
    publishCoverState();             // Notify Home Assistant
    setRelayState(true);            // Pulse relay (500ms)
}
```

**STOP Command (`"stop"`):**
```cpp
// Immediately pulse relay regardless of current state
setRelayState(true);  // 500ms pulse to stop/start door movement
```

#### Sensor Feedback Integration
The system subscribes to door sensor updates and processes state changes:

```cpp
void handleDoorSensorUpdate(const char* sensorState) {
    // Convert sensor "OPEN"/"CLOSED" to cover states
    String newState = (strcmp(sensorState, "OPEN") == 0) ? STATE_OPEN : STATE_CLOSED;

    lastConfirmedState = newState;  // Track last known physical state

    // If transitioning, move to confirmed state
    if (coverState == STATE_OPENING || coverState == STATE_CLOSING) {
        coverState = newState;
        transitionStartTime = 0;  // Clear timeout timer
        publishCoverState();
    }
    // If state mismatch, sync to sensor state
    else if (coverState != newState) {
        coverState = newState;
        publishCoverState();
    }
}
```

#### Timeout Protection
To prevent the system from getting stuck in transitional states:

```cpp
void checkTransitionTimeout() {
    if ((coverState == STATE_OPENING || coverState == STATE_CLOSING) &&
        transitionStartTime > 0 &&
        (millis() - transitionStartTime > TRANSITION_TIMEOUT_MS)) {

        // Timeout reached - revert to last confirmed sensor state
        coverState = lastConfirmedState;
        transitionStartTime = 0;
        publishCoverState();
    }
}
```

**Timeout Configuration:**
- **Transition Timeout**: 60 seconds (`TRANSITION_TIMEOUT_MS = 60000`)
- **Reconnection Interval**: 5 seconds (`RECONNECT_INTERVAL_MS = 5000`)

### Relay Control Logic
The relay operates in momentary pulse mode for safety:

```cpp
void setRelayState(bool newState) {
    if (newState) {
        digitalWrite(RELAY_PIN, HIGH);  // Activate relay
        delay(500);                     // 500ms pulse
        digitalWrite(RELAY_PIN, LOW);   // Deactivate relay
        relayState = false;             // Update internal state
    }
}
```

**Key Safety Features:**
- **Momentary Operation**: Relay never stays on continuously
- **500ms Pulse Duration**: Sufficient to trigger garage door opener
- **Active HIGH Configuration**: HIGH signal activates relay

### Error Handling and Recovery

#### Network Resilience
- **WiFi Monitoring**: Continuous connection status monitoring
- **MQTT Health Checks**: Regular connection verification
- **Graceful Degradation**: System continues operating locally if network fails

#### State Synchronization
- **Sensor Priority**: Physical sensor state takes precedence over commanded state
- **State Validation**: Cross-reference between commanded and sensed states
- **Automatic Correction**: System self-corrects state mismatches

#### Debugging and Monitoring
Enable detailed logging with `#define SERIAL_DEBUG`:

```cpp
[MQTT] Message received on topic: homeassistant/cover/garage_door/set
[MQTT] Payload: open
[COVER] Received command: open
[COVER] Opening door...
[RELAY] Setting relay state to ON
[RELAY] Pin D1 (GPIO5) set to: HIGH
[RELAY] Pulse complete, relay turned OFF
[MQTT] Publishing cover state: opening
[SENSOR] Door sensor state: OPEN
[SENSOR] Transitioning from opening to open
[MQTT] Publishing cover state: open
```

## Installation and Setup

### 1. Hardware Assembly
1. Connect the relay module to the Wemos D1 Mini Pro
2. Wire the relay output to your garage door opener's control terminals
3. Install and configure a separate door sensor system (Z-Wave, Zigbee, etc.)

### 2. Arduino IDE Setup
1. Install the ESP8266 board package
2. Install required libraries: `PubSubClient`, `ArduinoJson`
3. Load the sketch and configure your network settings

### 3. Home Assistant Configuration
The device will automatically appear in Home Assistant via MQTT discovery. Ensure your MQTT integration is configured and the broker is accessible.

### 4. Door Sensor Integration
Configure your door sensor to publish state updates to the specified MQTT topic in JSON format:
```json
{
  "device": "binary_sensor.contact_sensor_door",
  "state_reported": "off",
  "STATE": "CLOSED"
}
```

## Troubleshooting

### Common Issues
- **Device Not Discovered**: Check MQTT broker connectivity and credentials
- **State Mismatches**: Verify door sensor topic and JSON format
- **Relay Not Activating**: Confirm wiring and relay module compatibility
- **Connection Drops**: Check WiFi signal strength and network stability

### Debug Mode
Enable `#define SERIAL_DEBUG` for detailed logging output at 115200 baud.

## Version History

- **v1.4** (2024-09-29): Current version with robust state management and error handling

## License

This project is open source and available for modification and distribution.