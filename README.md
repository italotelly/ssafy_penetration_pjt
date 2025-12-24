# Intelligent Robotic Sorting System with PdM
![OS](https://img.shields.io/badge/OS-Ubuntu%2022.04-E95420)
![OS](https://img.shields.io/badge/OS-Windows%2011-blue)

![Language](https://img.shields.io/badge/Language-C-lightgrey)
![Language](https://img.shields.io/badge/Language-Python%203.8%2B-3776AB)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Robotics](https://img.shields.io/badge/Robotics-SLAM-green)
![Robotics](https://img.shields.io/badge/Robotics-Hand--Eye%20Calibration-orange)
![AI](https://img.shields.io/badge/AI-LSTM%20AutoEncoder-red)

![Vision](https://img.shields.io/badge/Vision-OpenCV%20%2B%20RGB--D-green)

![Embedded](https://img.shields.io/badge/Embedded-STM32-orange)
![Embedded](https://img.shields.io/badge/Embedded-ESP32-green)
![Embedded](https://img.shields.io/badge/Embedded-Raspberry%20Pi-red)
![Protocol](https://img.shields.io/badge/Protocol-Modbus%20TCP-blue)

![Frontend](https://img.shields.io/badge/Frontend-Vue.js-42b883)
![Backend](https://img.shields.io/badge/Backend-Node.js-339933)
![Network](https://img.shields.io/badge/Network-WebSocket-purple)

<p align="center">
  <img src="./assets/system_overview.png" alt="System Overview">
</p>

This project implements an **intelligent robotic sorting system** that integrates **RGB-D vision, industrial robot control, embedded systems, predictive maintenance (PdM), and web-based monitoring**.

A Dobot manipulator performs **color-based object sorting** using an Intel RealSense D435i camera, while STM32/ESP32-based embedded devices handle **real-time signaling and safety control**. The overall system is coordinated through a **Modbus TCP–based industrial communication layer**, with additional support for **mobile robot (TurtleBot) interaction**.

In addition, a **web-based dashboard** provides **real-time visualization of system logs, Dobot status, and TurtleBot status** via WebSocket communication, enabling intuitive monitoring and operational awareness.

Furthermore, vibration data collected from the robot base is analyzed using an **LSTM AutoEncoder** to detect early signs of abnormal behavior, enabling **data-driven preventive maintenance** in a smart factory environment.

## Table of Contents
- [Team & Development Period](#team--development-period)
- [System Flow](#system-flow)
- [Hardware](#hardware)
- [Installation](#installation)
- [Execution](#execution)
- [Run](#run)


<a name="team--development-period"></a>
## Team & Development Period
- Development Period: 2025.11.10 – 2025.12.25
<table align="center">
  <tr>
    <td align="center">
      <img src="./assets/team1.jpg" width="300"><br>
      <sub><b>SSAFY_14TH_권순재</b></sub>
    </td>
    <td align="center">
      <img src="./assets/team2.jpg" width="300"><br>
      <sub><b>SSAFY_14TH_류광철</b></sub>
    </td>
  </tr>
</table>

<a name="system-flow"></a>
## System Flow
```mermaid
flowchart TD
    %% =====================
    %% System Initialization
    %% =====================
    A[System Power ON] --> B[Initialize System]

    B --> B1[Modbus TCP Client]
    B --> B2[Dobot Robot]
    B --> B3[UART STM32]
    B --> B4[RealSense D435i]
    B --> B5[Hand–Eye Calibration]

    B --> C[WAIT_START]

    %% =====================
    %% Normal Operation Flow
    %% =====================
    C -->|Start Signal (110)| D[START_PROCESS]
    D --> E[Conveyor ON]

    E --> F[Object Detection]
    F -->|RGB-D + OpenCV| G{Detected Color?}

    G -->|RED / GREEN / BLUE| H[Conveyor OFF]
    H --> I[Send Color Code to STM32]
    I --> J[WAIT_CLASSIFY]

    J -->|Classify Ack (101)| K[Robot Pick & Place]
    K --> L[COMPLETE_TASK]

    L -->|Continue| F
    L -->|Finish Signal (100)| M[FINISH_PROCESS]

    M --> N[Robot Home]
    N --> O[Export Logs]
    O --> C

    %% =====================
    %% Yellow Object & TurtleBot
    %% =====================
    G -->|YELLOW| T[Increase Yellow Count]
    T -->|Threshold Reached| U[TurtleBot Start]
    U --> V[Monitor TurtleBot Status]
    V -->|Returned| F

    %% =====================
    %% Emergency Handling
    %% =====================
    F -->|Emergency (111)| X[EMERGENCY STOP]
    K -->|Emergency (111)| X

    X --> Y[Robot Stop & Conveyor OFF]
    Y -->|Emergency Release (000)| Z[Restore Previous State]
    Z --> F

    %% =====================
    %% Web Dashboard Monitoring
    %% =====================
    F --> W[Web Dashboard]
    K --> W
    L --> W
    X --> W

    W -->|WebSocket| W1[Real-time Logs]
    W -->|WebSocket| W2[Dobot Status]
    W -->|WebSocket| W3[TurtleBot Status]

    %% =====================
    %% Predictive Maintenance
    %% =====================
    B2 --> P[Collect Vibration Data]
    P --> Q[LSTM AutoEncoder]
    Q -->|Reconstruction Error| R{Anomaly Detected?}
    R -->|Yes| W
    R -->|No| Continue[Normal Operation]

    %% =====================
    %% Styling
    %% =====================
    classDef emergency fill:#ffe6e6,stroke:#ff0000,stroke-width:2px;
    class X,Y,Z emergency;
```