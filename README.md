![](images/car.png)
# [E-Design Contest] Ti SMPG03507 Black-Line Following Car

## 🎯 Project Introduction

This project is based on the TI SMPG03507 control board, combined with Ganwei non-MCU grayscale sensors and MPU6050 gyroscope.
Using the PID algorithm, it dynamically adjusts the left and right wheel motor speeds to achieve black-line tracking.
The car can detect black lines and follow them stably, suitable for applications such as intelligent transportation and robotics.

⸻

## 🛠️ Hardware List
- Main Board: Ti SMPG03507
- Grayscale Sensor: Ganwei non-MCU grayscale sensor
- Motor Driver: TB6612
- Motors: Lunqu G310
- Gyroscope: MPU6050
- Display: OLED

⸻

## 🧩 Project Features
- Modular Design: Independent integration of sensors, motor drivers, and motors for easier debugging and replacement
- PID Control Loop: Smooth speed regulation through speed-loop PID
- Real-Time Hardware Adjustment: Uses MPU6050 gyroscope to enhance chassis stability
- High Compatibility: Supports secondary development and expansion

⸻

## ⚙️ Control Logic
- The grayscale sensor array outputs deviation information for the left and right sides
- The controller calculates left-right grayscale difference as PID input
- MPU6050 measures angular velocity to assist motor output adjustment
- PID output controls the TB6612 motor driver, adjusting left and right motor speeds
- This ensures the car moves stably along the black line

⸻

## 📺 Line-Following Demo

<div align="center">
  <img src="images/car.gif" alt="Line-Following Demo" width="1000">
</div>



⸻

📧[choucisan@gmail.com]
