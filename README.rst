# Oxygen2 – Real-Time Water Quality Monitoring System

This embedded system project is designed to monitor water quality in real-time, targeting use cases such as the transportation and welfare monitoring of wild salmon and sea trout. It retrieves, parses, and displays data from an **Aanderaa Oxygen Optode 4531 sensor** using a **nRF52 microcontroller**.

---

## 📦 Project Structure

oxygen2/
├── boards/ # Board configuration (e.g., overlay files)
├── build/ # Build artifacts (ignored by Git)
├── src/ # Source code (.c and .h files)
├── fonts.h # Font data for LCD
├── CMakeLists.txt # CMake build config
├── prj.conf # Zephyr project configuration
├── README.md # Project documentation
├── sample.yaml # Zephyr sample metadata
├── .gitignore # Files and folders to ignore in Git

---

## ✅ Features Implemented

- ✅ **UART Integration** with Oxygen Optode sensor using MAX3232 IC  
- ✅ **Live display of:**
  - Oxygen concentration (in µM)
  - Air saturation (in %)
  - Temperature (in °C)
- ✅ **Data parsing** and formatting from noisy UART input  
- ✅ **Display integration** using ST7565R-compatible 128x64 LCD  
- ✅ **Real-time data shown on both terminal and LCD**

---

## 📈 Sensor Details

The Aanderaa Oxygen Optode sensor provides:
- **O2 Concentration** in µM  
- **Air Saturation** in %  
- **Temperature** in °C  

These values are critical for assessing fish health and stress levels during transport. In future work, µM concentration will be converted to mg/L for regulatory compliance and comparison with scientific literature.

---

## 🔜 Next Steps

- [ ] Integrate **wireless communication** (Bluetooth/Wi-Fi)
- [ ] Add **buzzer/LED alerts** for unsafe thresholds
- [ ] Include **turbidity sensor** and process its readings
- [ ] Measure **power consumption** using Nordic Power Profiler Kit
- [ ] Add **remote threshold adjustment** and basic calibration UI
- [ ] Expand to log data for future cloud/database upload

---

## 🔧 Setup

- Platform: **nRF52832 DK**
- RTOS: **Zephyr RTOS**
- Toolchain: **west / nRF Connect SDK**

Build using:

```bash
west build -b nrf52dk_nrf52832
west flash
