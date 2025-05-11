# 🌡️ PID Temperature Control System with MSP430 + MATLAB GUI

This project implements a **Proportional-Integral-Derivative (PID)** controller using an **MSP430 microcontroller** and a **MATLAB graphical user interface (GUI)** to regulate the temperature of a **thermo-electric cooler (TEC)**. It demonstrates how embedded hardware and desktop software can work together for real-time temperature monitoring and control.

---

## 📦 Project Overview

The goal of this project was to develop a closed-loop control system capable of maintaining a user-defined temperature using feedback from a temperature sensor. A PID algorithm was implemented on the MSP430 MCU to control the power to a TEC. Users could set the desired temperature and tune the PID gains through a MATLAB GUI that also displayed the real-time temperature profile.

---

## 🎯 Features

- Real-time temperature monitoring and logging via MATLAB GUI
- User-adjustable PID parameters (K<sub>P</sub>, K<sub>I</sub>, K<sub>D</sub>)
- Temperature setpoint input and tracking
- Serial communication between MATLAB and MSP430
- Ziegler-Nichols tuning method for parameter estimation

---

## ⚙️ Hardware & Software

### Hardware
- **MSP430 Microcontroller**
- **Thermo-Electric Cooler (TEC)**
- **Temperature Sensor (e.g., thermistor or LM35)**
- **Power driver circuit for TEC**

### Software
- **MSP430 C firmware (Code Composer Studio)**
- **MATLAB GUI for PID tuning and data plotting**
- **Serial communication protocol for data exchange**

---

## 🔧 PID Tuning

The PID parameters were determined using the **Ziegler-Nichols tuning method**:

- **K<sub>P</sub> = 540**  
- **K<sub>I</sub> = 7**  
- **K<sub>D</sub> = 1.75**

These parameters provided a well-tuned response with good stability and minimal overshoot.
