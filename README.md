# Smart-Greenhouse-Dual-Controller
Advanced smart greenhouse system using an ESP32 master and STM32 slave. Includes a responsive local web UI, Telegram bot integration, multi-tier data logging, and automated vent/fan cooling.
# 🌱 Smart Greenhouse: ESP32 & STM32 Dual-Controller System

An advanced, fault-tolerant smart greenhouse monitoring and automation platform. This project utilizes a dual-controller architecture, separating the heavy data-management and networking tasks (ESP32) from the physical hardware actuation (STM32F103C8T6).

## 🚀 Key Features
*   **Dual-Controller Architecture:** ESP32 handles Wi-Fi, UI, and sensors; STM32 acts as a dedicated actuator slave via a custom UART packet bridge.
*   **Offline-First Web Dashboard:** A responsive HTML/JS dashboard hosted directly on the ESP32. Displays real-time sensor data, VPD (Vapor Pressure Deficit), and plant health scores.
*   **Multi-Tier Data Logging:** Uses LittleFS to store environmental data persistently across 30-min, 24-hour, 7-day, and 30-day resolutions.
*   **Telegram Bot Integration:** Sends instant alerts for out-of-bounds sensor readings and accepts two-way commands (e.g., `/fan_on`, `/status`).
*   **Automated & Manual Override:** Automatic temperature-based cooling with a built-in 10-minute manual override lock to prevent command conflicts.

## 🛠️ Hardware Components
*   **Controllers:** ESP32 Dev Module (Master) & STM32F103C8T6 Blue Pill (Slave)
*   **Sensors:** DHT22 (Temp/Humidity), BH1750 (I2C Ambient Light), Analog Capacitive Soil Moisture Sensor
*   **Actuators:** 12V PC Fan (driven by IRLZ44N MOSFET), SG90 Servo Motor (Vents), 5V Relay (Water Pump)
*   **Display:** 128x64 I2C SSD1306 OLED Display

## 🔌 System Wiring & UART Bridge
The boards communicate asynchronously via UART at 115200 baud. 
*   **ESP32 TX2 (GPIO 17)** ➔ **STM32 RX (PA10)**
*   **ESP32 RX2 (GPIO 16)** ➔ **STM32 TX (PA9)**
*   *Note: Both controllers and external power supplies must share a common Ground.*

Check the `/Schematics` folder for the full wiring diagram and system data flow block diagram.

## 💻 Installation & Setup
1. Clone this repository.
2. Open `ESP32_Master_Firmware/greenhouse_master.ino` in the Arduino IDE.
3. Update the Wi-Fi credentials and Telegram Bot Token:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   const char* TG_TOKEN      = "YOUR_TELEGRAM_BOT_TOKEN";
   const char* TG_CHAT       = "YOUR_CHAT_ID";
