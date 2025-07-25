# ESP32-Gyro

Accelerometer / Gyro, can report via regular UDP messages and MQTT messages including.

- X/Y/Z Accellerometer
- X/Y/Z Gyro
- G (gravity, normally 1.0 when stable)
- RPM (up to around 330 rpm)

This is reported by JSON when UDP and MQTT and format is obvious.

Also reports as BLE advertisement with following format (values are LSB first):-

- Manufacturer specific with manufacturer `0E9C`, i.e. `FF`, `9C`, `0E`
- Message type `S` `1`
- 16 bit signed raw accellerometer data X
- 16 bit signed raw accellerometer data Y
- 16 bit signed raw accellerometer data Z
- 16 bit sigend raw gyro data X
- 16 bit sigend raw gyro data Y
- 16 bit sigend raw gyro data Z
- 16 bit unsigned rpm, multiplied by 100
- 16 bit unsigend G, multipled by 100

The configured hostname is also sent (truncated if needed, and on active scan).
