# 🚀 PENDULUM ROBOT 2025

Welcome to **PENDULUM ROBOT 2025** — an Inverted Pendulum two wheeled robot as next version to 2024 model. This repository contains the source code and resources for it. Whether you're a developer, contributor, or just curious, this README will help you get started.

<img width="966" height="751" alt="IMG Pend-2025-2" src="IMG Pend-2025-2.png" />

## 📦 Features

- 🏗️ 3D printed inverted pendulum body
- 💻 Custom PCB with MCU, PSU, Motor Drivers etc.
- 🚦 80MHz ARM Cortex-M0+ MCU by Texas Instruments
- ✈️ IMU based on MPU6050
- 🎛️ Traditional PID (PDF+damping) controller implemented
- 🧮 GUI provided for fast parameters and gains tuning
- 🪒 Model based controller design (Matlab)
- 📶 Bluetooth (HC-05 module) used to transmit 28 signals
- 🚗 2x 12V POLOLU DC motors for each wheel
- 🔋 12V Li-Ion battery 1300mAh

## 📺 GUI Screenshot
<img width="966" height="768" alt="IMG Pend-2025-2" src="IMG GUI POWER-BOT-2025.png" />

## 🕹 PCB Detail
<img width="966" height="488" alt="IMG Pend-2025-3" src="IMG TOP POWER-BOT-2025.png" />

## 🔧 Body Diagram
<img width="966" height="1021" alt="IMG Pend-2025-4" src="IMG BODIA PEND-2025.png" />

### System Dynamics

The robot's behavior is defined by three coupled differential equations set to zero (D'Alembert form):

1.  **Linear Force ($x$):**
    $$0 = M_A \ddot{x} - M_B \ddot{\theta} + B_{eff} \dot{x} - \frac{N K_t}{R_w} I_a$$
2.  **Rotational Torque ($\theta$):**
    $$0 = J_{tot} \ddot{\theta} - M_B \ddot{x} - K_g \theta + N K_t I_a + (N B_{rot}) \dot{\theta}$$
3.  **Electrical Current ($I_a$):**
    $$0 = L_a \dot{I}_a + R_a I_a + K_b N \left( \dot{\theta} - \frac{\dot{x}}{R_w} \right) - V_{in}$$

> **Note:** $M_B$ represents the "Drunken Sailor" coupling, and $N B_{rot}$ accounts for significant gearbox damping.
