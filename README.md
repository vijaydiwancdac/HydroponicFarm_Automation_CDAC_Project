# 🌿 Real-Time Hydroponic Farm Automation Using STM32 with FreeRTOS and IoT-Based Monitoring System

## 📌 Overview

This project presents a **real-time automated hydroponic farming system** built using the **STM32F407** microcontroller running **FreeRTOS**, integrated with **ESP32** for **IoT connectivity via Blynk**. The system enables intelligent environmental control and nutrient management based on real-time sensor data and farmer inputs through a mobile app.

It is designed as a low-cost, smart agriculture solution with **modular architecture**, supporting remote threshold configuration, real-time sensor monitoring, actuator control, and accurate nutrient dosing.

---

## 🧠 Core Features

- **Remote Threshold Update** for TDS through Blynk App
- **TDS Monitoring** and **nutrient dosing suggestion**
- **Temperature & Humidity Control** using DHT22 and relay-based fan
- **Water Level Monitoring** using an ultrasonic sensor
- **Real-Time Alerts & Data Visualization** via LCD and Blynk
- **Modular RTOS-based design** for multitasking and scalability

---

## 🧰 Hardware Components

| Component         | Role                                                        |
|------------------|-------------------------------------------------------------|
| STM32F407        | Central controller running FreeRTOS and interfacing sensors |
| ESP32            | Wi-Fi module for IoT connectivity (Blynk platform)          |
| TDS Sensor       | Monitors nutrient concentration (ppm)                       |
| DHT22            | Measures temperature and humidity                           |
| Ultrasonic Sensor| Calculates water level (volume) in nutrient tank            |
| LCD Display      | Displays sensor data and alerts                             |
| Relay Module     | Controls the fan based on environment conditions            |
| Fan              | Cools system when temperature or humidity crosses threshold |

---

## 🔧 Software Architecture

### 📍 STM32 Side (FreeRTOS)
- Periodic tasks for:
  - Sensor polling (DHT22, Ultrasonic, TDS)
  - LCD updates
  - UART communication with ESP32
  - Fan control based on threshold logic
- Nutrient calculation logic based on real-time volume and TDS
- Display alerts and suggestions on LCD

### 📍 ESP32 Side (Arduino + Blynk)
- Receives TDS threshold from Blynk app
- Sends updated threshold to STM32 via UART
- Forwards alerts and sensor values to Blynk

---

## 🔬 TDS Nutrient Calculation Logic

Example rule:

> *1 liter of nutrient + 1000 liters of water = 500 ppm*

If current ppm is below the threshold:
- Calculate percentage difference
- Adjust nutrient amount based on current tank volume
- Display dosage (in mL) on LCD and send via Blynk

✔️ All logic is dynamic and calculated in real time.

---

## 📲 Cloud Monitoring via Blynk

- Live data view: Temperature, Humidity, Water Level, TDS
- Remote configuration: Set TDS threshold for different plant stages
- Alert display: Warnings for high temperature or low TDS

---

## 📡 Communication Overview

| Module  | Functionality                                                 |
|---------|---------------------------------------------------------------|
| STM32   | Sensors, control logic, dosing logic, LCD, UART comm.         |
| ESP32   | Wi-Fi connectivity, Blynk cloud interface                     |
| Blynk   | Remote threshold input, data visualization                    |

---

## 🔄 Customization

TDS thresholds can be adjusted based on the plant stage:

- Seedling Stage
- Vegetative Stage
- Flowering or Harvest Stage

Farmer updates threshold via Blynk → ESP32 sends to STM32 → Logic adapts.

---

## 📁 Repository Structure

```bash
HydroponicFarm_Automation/
├── STM32_Firmware/             # STM32 + FreeRTOS source code
│   ├── Core/                   # main.c, sensor tasks, UART handler
│   ├── Drivers/                # HAL/LL drivers
│   ├── Inc/                    # Header files
│   ├── Src/                    # Source files
│   ├── Startup/                # Startup assembly, system files
│   ├── FreeRTOS/               # RTOS config and CMSIS OS files
│   └── Project_Config/         # .ioc and project metadata
│
├── ESP32_Firmware/             # Arduino-based ESP32 source
│   ├── src/                    # Main .ino file and logic
│   ├── include/                # Optional headers
│   └── lib/                    # Custom Blynk/UART libraries if used
│
├── Hardware_Design/            # Circuit diagrams, PCB layout, BOM
│   ├── Schematic/              # Fritzing/KiCad/etc.
│
├── Documentation/              # Detailed documentation
│   ├── System_Architecture.md  # Data flow, task structure
│   ├── Setup_Guide.md          # How to compile, flash, connect
│   └── User_Manual.md          # Instructions for farmer/app usage
│
├── Media/                      # Visuals and demo content
│   ├── UI_Screenshots/         # Blynk app UI and LCD photos
│   └── Demo_Video/             # Video walkthrough of working project
│
├── Scripts/                    # Flashing, data parsing scripts
│   └── flash_upload.py
│
├── .gitignore                  # Excludes build files, backups, etc.
├── LICENSE                     # Choose: MIT / Apache 2.0 / GPL, etc.
├── CONTRIBUTING.md             # Guidelines for team collaboration
└── README.md                   # You're here!
