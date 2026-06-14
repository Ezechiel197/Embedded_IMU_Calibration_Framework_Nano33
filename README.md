# Embedded-IMU-Calibration-Framework-Nano33


A modern, object-oriented C++ library wrapper and application firmware for the **Bosch BMI270** (6-DoF IMU) and **Bosch BMM150** (Geomagnetic Sensor). This project provides a clean abstraction layer over the native Bosch Sensortec C drivers, featuring runtime calibration, non-blocking serial command execution, and hardware-specific axis remapping (optimized for the Arduino Nano 33 BLE Sense).

## 🚀 Features

- **Object-Oriented Design:** Clean C++ encapsulation (`BmImu` class) wrapping low-level Bosch C APIs.
- **Dual Mode Support:** Flexible switching between `Continuous Mode` (FIFO-buffered) and `One-Shot Mode`.
- **Hardware Abstraction:** Dynamic $\text{I}^2\text{C}$ bus assignment supporting multiple bus instances (e.g., `Wire`, `Wire1`).
- **Advanced Calibration Pipeline:** On-the-fly correction of raw sensor values using factory/user offsets and scaling factors stored in flash memory.
- **Coordinate System Alignment:** Automatic axis remapping and sign correction to guarantee a consistent right-handed coordinate system across decoupled sensor ICs.
- **Non-Blocking Architecture:** Asynchronous serial command interface (`cmdInterface`) for real-time mode switching and data streaming.
- **Mbed OS Integration:** Optional thread-safe interrupt handling using Mbed OS `EventQueue` to offload the ISR context.

---

## 🛠️ Hardware & Architecture

This library is primarily optimized for boards featuring the Bosch BMI270 and BMM150 sensor combo connected via $\text{I}^2\text{C}$ (Fast Mode @ 400 kHz), such as the **Arduino Nano 33 BLE Sense**.

### Axis Remapping Matrix
Since decoupled sensor ICs can be mechanically rotated relative to each other on the PCB, the firmware automatically applies a transformation matrix to align the coordinate systems:
* **Accelerometer/Gyroscope:** Inverts and swaps axes dynamically when compiled for `TARGET\_ARDUINO\_NANO33BLE`.
* **Magnetometer:** Performs a $90^\circ$ 2D coplanar rotation ($X_{\text{new}} = Y_{\text{old}}$, $Y_{\text{new}} = -X_{\text{old}}$) to match the IMU frame.

---

## 💻 Software Structure

* **`BmImu.h` / `BmImu.cpp`**: Core library class managing I2C communication wrappers, sensor registers, FIFO configurations, and raw data ingestion.
* **`main.cpp`**: Application entry point implementing the state-machine-driven data streaming loop, flash preferences loading, and the linear calibration formula:
  $$\text{Value}_{\text{calibrated}} = \frac{\text{Value}_{\text{raw}} - \text{Offset}}{\text{Scale}}$$

---

## ⚙️ Serial Command Interface

The application features a non-blocking command interpreter. You can switch between different streaming states via the serial interface:

| Command State | Output Description | Format |
| :--- | :--- | :--- |
| `SAMPLING\_G` | Calibrated Gyroscope Data [°/s] | `G \\t X \\t Y \\t Z` |
| `SAMPLING\_A` | Calibrated Accelerometer Data [g] | `A \\t X \\t Y \\t Z` |
| `SAMPLING\_M` | Calibrated Magnetometer Data [µT] | `M \\t X \\t Y \\t Z` |

---

## 📦 Getting Started

1. Clone this repository into your Arduino libraries folder or include it in your PlatformIO project.
2. Ensure the official Bosch `bmi270` and `bmm150` driver headers are present in your include path.
3. Flash the provided example sktech/`main.cpp` onto your microcontroller.
4. Open a Serial Monitor at **9600 Baud** to interact with the device.

## 📝 License

This library is distributed under the terms of the GNU Lesser General Public License (LGPL v2.1). Base library components derived from Arduino SA (2019).

---

## :man_technologist: **Author**

**Ezechiel Tonkeme**     


<img width="1600" height="1025" alt="WhatsApp Image 2026-06-13 at 20 04 41" src="https://github.com/user-attachments/assets/380b7b61-5602-4ffc-bac7-46dbfb94d041" />

