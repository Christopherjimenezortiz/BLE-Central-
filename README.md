# BLE Central Receiver

Wireless receiver for 6-axis IMU sensor data via BLE.

## Features
- BLE client (GATT central)
- Scans for ESP32_Sensor nodes
- Connects and discovers characteristics
- Reads 6 IMU values (Accel X/Y/Z, Gyro X/Y/Z)
- Formatted real-time display

## Hardware
- ESP32-C3 Super Mini
- NimBLE stack (ESP-IDF)

## Works with
- [ble-sensor-node](https://github.com/Christopherjimenezortiz/ble-sensor-node)

## Status
✅ Scanning and connection working
✅ Service/characteristic discovery
✅ Reading all 6 sensor values
✅ Real-time display (1Hz update)
🚧 Notifications (future optimization)

## Part of
6-node wireless motion capture system for AI-driven movement analysis.