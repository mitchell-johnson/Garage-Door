# Garage Door Controller

A robust ESP8266-based garage door controller with Home Assistant integration via MQTT. This project provides remote garage door control with automatic discovery, state reporting, and door sensor feedback.

## Hardware Requirements

- **Wemos D1 Mini Pro** (ESP8266-based microcontroller)
- **Single-channel relay module** (active HIGH)
- **Door sensor** (compatible with Home Assistant binary sensor)
- **Garage door opener** with momentary contact trigger

### Wiring Configuration

| Component | Pin | GPIO | Description |
|-----------|-----|------|-------------|
| Relay Module | D1 | GPIO5 | Controls garage door opener relay |
| Power | 3.3V/5V | - | Power supply for D1 Mini |
| Ground | GND | - | Common ground |

**Important**: This relay module is configured as **active HIGH** - a HIGH signal turns the relay ON.

## Software Dependencies

### Arduino Libraries Required

```cpp
#include <ESP8266WiFi.h>    // WiFi connectivity
#include <PubSubClient.h>   // MQTT client
#include <ArduinoJson.h>    // JSON parsing for MQTT payloads
```

Install these libraries through the Arduino IDE Library Manager.

## Configuration

### Initial Setup

1. **Copy the example configuration file:**
   ```bash
   cd d1_mini_relay_controller
   cp config.example.h config.h
   ```

2. **Edit `config.h` with your settings:**

#### WiFi Configuration
```cpp
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";
```

#### MQTT Broker Configuration
```cpp
const char* MQTT_BROKER_IP = "192.168.1.100";  // Your MQTT broker IP
const int   MQTT_PORT = 1883;                   // Default MQTT port
const char* MQTT_USERNAME = "";                 // Optional authentication
const char* MQTT_PASSWORD = "";                 // Optional authentication
```

#### Device Configuration
```cpp
const char* DEVICE_UNIQUE_ID = "garage_door";   // Must be unique in Home Assistant
const char* DEVICE_NAME = "Garage Door";        // Friendly name
```

#### Door Sensor Integration
```cpp
const char* DOOR_SENSOR_TOPIC = "homeassistant/binary_sensor/binary_sensor.contact_sensor_door/state";
```

**Expected sensor JSON format:**
```json
{
  "device": "binary_sensor.contact_sensor_door",
  "state_reported": "off",
  "STATE": "CLOSED"
}
```

The `STATE` field should be either `"OPEN"` or `"CLOSED"`.

### Debug Mode

To enable detailed serial logging at 115200 baud, uncomment in `config.h`:
```cpp
#define SERIAL_DEBUG
```

**Important:** The `config.h` file is excluded from version control to protect your credentials. Never commit this file to a public repository.

## MQTT Implementation Details

### Topic Structure

The controller uses the following MQTT topic structure:

| Topic Type | Pattern | Example |
|------------|---------|---------|
| Discovery | `homeassistant/cover/{device_id}/config` | `homeassistant/cover/garage_door/config` |
| Commands | `homeassistant/cover/{device_id}/set` | `homeassistant/cover/garage_door/set` |
| State | `homeassistant/cover/{device_id}/state` | `homeassistant/cover/garage_door/state` |
| Availability | `homeassistant/cover/{device_id}/status` | `homeassistant/cover/garage_door/status` |

### Home Assistant Auto-Discovery

The device automatically publishes a discovery payload with the following configuration:

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
  "retain": true
}
```

### MQTT Connection Management

- **Automatic Reconnection**: Attempts reconnection every 5 seconds if connection is lost
- **Last Will Testament (LWT)**: Publishes "offline" to availability topic on disconnect
- **Buffer Size**: Increased to 1024 bytes to accommodate large discovery payloads
- **Retained Messages**: State and availability messages are retained

## Open/Close State Logic

### State Machine

The controller implements a 4-state state machine:

| State | Description | Transitions |
|-------|-------------|-------------|
| `closed` | Door is fully closed | → `opening` (on open command) |
| `open` | Door is fully open | → `closing` (on close command) |
| `opening` | Door is currently opening | → `open` (sensor feedback or timeout) |
| `closing` | Door is currently closing | → `closed` (sensor feedback or timeout) |

### Command Processing

#### Open Command (`"open"`)
```cpp
if (coverState != STATE_OPEN && coverState != STATE_OPENING) {
    coverState = STATE_OPENING;
    transitionStartTime = millis(); // Start timeout timer
    publishCoverState();
    setRelayState(true); // Trigger 500ms pulse
}
```

#### Close Command (`"close"`)
```cpp
if (coverState != STATE_CLOSED && coverState != STATE_CLOSING) {
    coverState = STATE_CLOSING;
    transitionStartTime = millis(); // Start timeout timer
    publishCoverState();
    setRelayState(true); // Trigger 500ms pulse
}
```

#### Stop Command (`"stop"`)
```cpp
// Always triggers relay pulse regardless of current state
setRelayState(true); // Trigger 500ms pulse
```

### Door Sensor Feedback Integration

The controller subscribes to a door sensor topic and processes JSON payloads:

```json
{
  "device": "binary_sensor.ranch_slider_door",
  "state_reported": "off",
  "STATE": "CLOSED"
}
```

#### State Synchronization Logic

1. **During Transitions**: If in `opening`/`closing` state, immediately moves to sensor-confirmed state
2. **State Mismatch**: If current state differs from sensor state, updates to match sensor
3. **Confirmed State Tracking**: Maintains `lastConfirmedState` for timeout recovery

### Timeout Protection

- **Timeout Duration**: 60 seconds (60,000 milliseconds)
- **Timeout Behavior**: If stuck in `opening`/`closing` state beyond timeout, reverts to last confirmed sensor state
- **Safety Feature**: Prevents indefinite transitional states if sensor feedback fails

```cpp
if (transitionStartTime > 0 && (millis() - transitionStartTime > TRANSITION_TIMEOUT_MS)) {
    coverState = lastConfirmedState; // Revert to sensor state
    transitionStartTime = 0; // Clear timer
    publishCoverState();
}
```

## Relay Control Implementation

### Momentary Pulse Operation

The relay operates in **momentary pulse mode** for safety:

```cpp
void setRelayState(bool newState) {
    digitalWrite(RELAY_PIN, newState ? HIGH : LOW);

    if (newState) {
        delay(500); // 500ms pulse duration
        digitalWrite(RELAY_PIN, LOW); // Turn off
        relayState = false;
    }
}
```

### Safety Features

- **Non-blocking Design**: Main loop continues during relay operation
- **Pulse Duration**: Fixed 500ms pulse prevents continuous relay activation
- **Active HIGH Configuration**: Compatible with standard relay modules

## Network Resilience

### WiFi Connection Management

- **Non-blocking Reconnection**: Attempts WiFi reconnection every 5 seconds
- **Status Monitoring**: Continuously monitors `WiFi.status()`
- **Graceful Degradation**: MQTT operations suspended during WiFi outages

### Error Recovery

- **Connection State Tracking**: Separate timers for WiFi and MQTT reconnection attempts
- **Automatic Recovery**: System automatically resumes normal operation when connections restore
- **Debug Logging**: Comprehensive serial debugging available (115200 baud)

## Installation & Setup

### 1. Hardware Setup
1. Connect relay module to D1 pin (GPIO5)
2. Wire relay to garage door opener's momentary contact terminals
3. Install door sensor and configure in Home Assistant
4. Power the D1 Mini Pro

### 2. Software Setup
1. Install required Arduino libraries via Arduino IDE Library Manager
2. Copy `config.example.h` to `config.h`
3. Update configuration values in `config.h` (WiFi, MQTT, device settings)
4. Upload sketch to D1 Mini Pro
5. Monitor serial output at 115200 baud for debugging (if `SERIAL_DEBUG` enabled)

### 3. Home Assistant Integration
1. Ensure MQTT broker is running and accessible
2. Device will auto-discover in Home Assistant
3. Configure door sensor entity and update topic in sketch
4. Test functionality using Home Assistant interface

## Troubleshooting

### Common Issues

**Device not appearing in Home Assistant:**
- Check MQTT broker connectivity
- Verify discovery topic publishing in logs
- Ensure Home Assistant MQTT integration is configured

**State not updating:**
- Verify door sensor topic configuration
- Check sensor JSON payload format
- Monitor serial output for sensor updates

**Relay not triggering:**
- Check wiring connections to D1 pin
- Verify relay module power supply
- Test relay activation with multimeter

**Connection issues:**
- Verify WiFi credentials and signal strength
- Check MQTT broker IP and port configuration
- Monitor reconnection attempts in serial output

### Debug Mode

Enable detailed logging by uncommenting in `config.h`:
```cpp
#define SERIAL_DEBUG
```

This provides comprehensive logging of:
- WiFi connection status
- MQTT connection and message details
- State transitions and command processing
- Relay activation and sensor updates

## Technical Specifications

- **Operating Voltage**: 3.3V (D1 Mini Pro)
- **WiFi**: 802.11 b/g/n (2.4GHz)
- **MQTT Protocol**: v3.1.1
- **Relay Type**: Active HIGH, momentary pulse
- **Timeout Protection**: 60-second transition limit
- **Reconnection Interval**: 5 seconds
- **Pulse Duration**: 500 milliseconds
- **Buffer Size**: 1024 bytes (MQTT)

---

*Generated with [Claude Code](https://claude.ai/code)*