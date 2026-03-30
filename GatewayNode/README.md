# ESP32 LoRa → Adafruit IO Gateway

This is a separate PlatformIO project intended for a **second ESP32** acting as a gateway:

- Receives newline-delimited telemetry JSON from the robot via LoRa E32 on `Serial2` (`RX=14`, `TX=4`, `9600 baud`)
- Publishes parsed values to Adafruit IO via MQTT (`io.adafruit.com:1883`)

## Adafruit IO feeds used

Create these feed keys in Adafruit IO (lowercase recommended):

- `temp`
- `hum`
- `safetyindex`
- `state`

## Configure credentials

Edit `GatewayNode/platformio.ini` and set `build_flags`:

- `-DWIFI_SSID="..."`
- `-DWIFI_PASS="..."`
- `-DIO_USERNAME="kichsatst"`
- `-DIO_KEY="..."`

