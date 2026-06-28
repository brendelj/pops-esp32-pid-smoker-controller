# Pop's PID Smoker Controller

This repo contains the reconstructed split-board Pop's PID controller architecture.

## Projects

### `Nano_Controller_MAX6675/`
Arduino Nano real-time controller.

Responsibilities:
- MAX6675 pit/chamber thermocouple
- Optional DS18B20 meat probes
- PID heat control
- Cooling servo/damper control
- Heat relay/PWM output
- 20x4 I2C LCD
- Rotary encoder local menu
- EEPROM configuration
- Safety shutdowns
- Serial telemetry and command protocol

### `ESP32_Companion_Bridge/`
ESP32 WROOM32 companion bridge.

Responsibilities:
- Wi-Fi access point or station mode
- Web dashboard
- REST API
- SD card cook logging
- UART bridge to Nano
- mDNS name `popspid.local`

## Serial protocol

Nano sends newline-terminated JSON telemetry:

```json
{"tmpC":107.2,"tgtC":107.2,"p1C":66.1,"p2C":0,"heatPct":45,"coolPct":0,"servoDeg":0,"state":"RUN","alarm":"OK"}
```

ESP32 sends newline-terminated JSON commands:

```json
{"cmd":"start"}
{"cmd":"stop"}
{"cmd":"set","tgtF":225}
{"cmd":"set","tgtC":107.2}
{"cmd":"pid","kp":6.0,"ki":0.08,"kd":15.0}
{"cmd":"save"}
```

## Build notes

These are the split-project final candidates reconstructed from the previous Pop's PID controller experiments. The Nano should remain the dependable controller and continue cooking if Wi-Fi or the ESP32 bridge is down.
