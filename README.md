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

### System Dynamics (N-E Coupled Model)

The robot's behavior is defined by three coupled differential equations (D'Alembert form):

**1. Linear Force Equation ($x$):**

$$0 = M_A \ddot{x} - M_B \ddot{\theta} + B_{eff} \dot{x} - \frac{N K_t}{R_w} I_a$$

**2. Rotational Torque Equation ($\theta$):**

$$0 = J_{tot} \ddot{\theta} - M_B \ddot{x} - K_g \theta + N K_t I_a + (N B_{rot}) \dot{\theta}$$

**3. Electrical Row (Current $I_a$):**

$$0 = L_{a} \dot{I}_{a} + R_{a} I_{a} + K_{b} \left( N \dot{\theta} - \frac{N}{R_{w}} \dot{x} \right) - V_{a}$$

> **Model Parameters:**
> 
> $M_{A}$: &nbsp;&nbsp;&nbsp;&nbsp; **Total Effective Mass [kg]** ($M_{total} + \frac{N \cdot J_{m}}{R_{w}^2}$)  
> $M_{B}$: &nbsp;&nbsp;&nbsp;&nbsp; **Drunken Sailor Coupling [kg.m]** ($m_{p} \cdot L_{tot}$)  
> $J_{tot}$: &nbsp;&nbsp; **Total Pendulum Inertia [kg.m^2]** ($J_{p} + m_{p} \cdot L_{tot}^2$)  
> $K_{g}$: &nbsp;&nbsp;&nbsp;&nbsp; **Gravity Stiffness [N.m/rad]** ($m_{p} \cdot g \cdot L_{tot}$)  
> $B_{eff}$: &nbsp;&nbsp; **Chassis Damping [N.s/m]** (Linear/Rolling Friction)  
> $N B_{rot}$: &nbsp;&nbsp; **Rotary Damping [N.m.s/rad]** (Gearbox/Viscous Friction)  
> $N$: &nbsp;&nbsp;&nbsp;&nbsp; **Number of Motors** (2)  
