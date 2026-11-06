# INAV-ADRC: Next-Generation Active Disturbance Rejection & Adaptive Flight Control

[![INAV Base](https://img.shields.io/badge/INAV%20Base-10.0.0--rc2-blue.svg)](https://github.com/iNavFlight/inav)
[![Control Algorithms](https://img.shields.io/badge/Algorithms-LADRC%20%7C%20SMC%20%7C%20L1--AC%20%7C%20CMAC-cyan.svg)](#core-control-algorithms)
[![Platform](https://img.shields.io/badge/Platform-STM32%20F7%20%7C%20H7%20%7C%20AT32%20%7C%20RP2350-orange.svg)](#supported-hardware)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-green.svg)](https://www.gnu.org/licenses/gpl-3.0)

A high-performance fork of **INAV (v10.0.0-rc2)** engineered to replace 100-year-old classical PID loops with modern state-space aerospace control architectures: **Linear Active Disturbance Rejection Control (LADRC)**, **Super-Twisting Sliding Mode Control (SMC)**, **L1-Adaptive Control (L1-AC)**, and **Cerebellar Model Articulation Controller (CMAC) Neural Networks**.

Repository: [https://github.com/Boyyt357/ADRC-inav](https://github.com/Boyyt357/ADRC-inav)

---

## Table of Contents
1. [Overview: Beyond Traditional PID](#overview-beyond-traditional-pid)
2. [Core Control Algorithms](#core-control-algorithms)
   - [1. Linear Active Disturbance Rejection Control (LADRC)](#1-linear-active-disturbance-rejection-control-ladrc)
   - [2. Super-Twisting Sliding Mode Control (SMC)](#2-super-twisting-sliding-mode-control-smc)
   - [3. L1-Adaptive Control (L1-AC)](#3-l1-adaptive-control-l1-ac)
   - [4. ADRC + CMAC Neural Network Auto-Trim](#4-adrc--cmac-neural-network-auto-trim)
3. [PID Tuning Tab Reimagined: Slider Mapping Guide](#pid-tuning-tab-reimagined-slider-mapping-guide)
4. [Step-by-Step WSL Build & Installation Guide](#step-by-step-wsl-build--installation-guide)
   - [Prerequisites (Ubuntu on WSL)](#1-prerequisites-ubuntu-on-wsl)
   - [Cloning & Building the Firmware](#2-cloning--building-the-firmware)
   - [Flashing via INAV Configurator](#3-flashing-via-inav-configurator)
5. [Quick Start & CLI Configuration](#quick-start--cli-configuration)
6. [Tuning Guidelines](#tuning-guidelines)
7. [Blackbox Telemetry & Logging](#blackbox-telemetry--logging)
8. [Credits & License](#credits--license)

---

## Overview: Beyond Traditional PID

Classical PID control is **reactive**. It must wait for an external disturbance (crosswind gust, prop-wash turbulence, voltage drop, or payload shift) to deflect the aircraft from its target angle before calculating error and fighting back. This lag often leads to bounce-back, phase delay, D-term noise amplification, and motor overheating.

**INAV-ADRC** implements an entirely proactive philosophy:
* **Real-Time State & Disturbance Observation:** An internal **Discrete Extended State Observer (DESO)** monitors gyro rates and actuator commands at up to 8 kHz.
* **Lumped Disturbance Estimation:** All unknown interior and exterior dynamics—aerodynamic drag, airframe flex, prop wash, battery voltage drop, and motor torque variations—are bundled into a single generalized disturbance state ($z_3$).
* **Feedforward Cancellation:** The algorithm algebraically cancels out $z_3$ in real time *before* the disturbance can divert the airframe from its commanded trajectory.

```
       Pilot Stick Command r(t)
                  │
                  ▼
         ┌─────────────────┐
         │ Virtual PD Law  │
         │   (Bandwidth)   │
         └────────┬────────┘
                  │ u0(t)
                  ▼
         ┌─────────────────┐       Real Gyro Feedback y(t)
  u(t) ──┤ Disturbance     │◄───────────────────┐
         │ Cancellation    │                    │
         │ (u0 - z3) / b0  │                    │
         └────────┬────────┘                    │
                  │                             │
                  ▼                             │
         ┌─────────────────┐           ┌────────┴────────┐
         │ Motors & Plant  │──────────►│ Discrete ESO    │
         └─────────────────┘   u(t)    │ Estimates:      │
                                       │ z1: Virtual Rate│
                                       │ z2: Angular Acc │
                                       │ z3: Disturbance ├──► Cancels z3
                                       └─────────────────┘
```

---

## Core Control Algorithms

This firmware introduces four selectable control architectures in INAV's rate loop (`pid_type`):

### 1. Linear Active Disturbance Rejection Control (LADRC)
* **CLI Setting:** `set pid_type = ADRC`
* **Observer:** Discrete Extended State Observer (DESO) with Zero-Order Hold (ZOH) discretization:
  $$r = e^{-\omega_o \cdot \Delta t}$$
  $$l_1 = 3(1 - r), \quad l_2 = \frac{(1 - r)^2 (5 + r)}{2 \Delta t}, \quad l_3 = \frac{(1 - r)^3}{\Delta t^2}$$
* **Virtual PD Law:**
  $$u_0 = k_p (r_{\text{target}} - z_1) - k_d z_2$$
  where $k_p = \omega_c^2$ and $k_d = 2\omega_c$ guarantee a **critically damped response** ($\zeta = 1.0$) with zero overshoot.
* **Control Output:**
  $$u(t) = \frac{u_0 - z_3}{b_0}$$
* **Key Benefits:** Rock-solid tracking out of the box, zero high-frequency D-term noise, and effortless prop-wash cancellation on quads and fixed wings.

### 2. Super-Twisting Sliding Mode Control (SMC)
* **CLI Setting:** `set pid_type = SMC`
* **Theory:** Defines a 2nd-order sliding surface (an ideal flight hyperplane track):
  $$s = e + \lambda \int e \, dt$$
  Uses continuous 2nd-order Super-Twisting integration to drive the state onto the sliding manifold without high-frequency control chattering:
  $$u = -k_1 |s|^{1/2} \mathrm{sgn}(s) + v, \quad \dot{v} = -k_2 \mathrm{sgn}(s)$$
* **Key Benefits:** Extreme robustness against violent aerodynamic buffeting, sudden payload shifts, and severe frame asymmetries.

### 3. L1-Adaptive Control (L1-AC)
* **CLI Setting:** `set pid_type = L1AC`
* **Theory:** Originally developed for NASA aerospace systems to guarantee uniform transient response under abrupt physical damage or parameter variations.
* **Decoupled Architecture:** Employs a rapid state predictor ($\Gamma$) coupled with a dedicated **low-pass filter (Butterworth)** in the adaptation loop.
* **Key Benefits:** Adapts rapidly to airframe changes while completely shielding the ESCs and motors from high-frequency noise and saturation.

### 4. ADRC + CMAC Neural Network Auto-Trim
* **CLI Setting:** `set pid_type = CMAC`
* **Theory:** Inspired by human neurobiology:
  - **ADRC** acts as the *spinal cord reflex*, delivering instantaneous millisecond disturbance rejection.
  - **CMAC (Cerebellar Model Articulation Controller)** acts as the *cerebellum*, storing learned airframe non-linearities and battery center-of-gravity offsets in an on-chip associative memory table.
* **Key Benefits:** Automatically learns and trims frame drag, motor imbalances, and asymmetric LiPo placement during the first 2–5 seconds of flight.

---

## PID Tuning Tab Reimagined: Slider Mapping Guide

When running `ADRC`, `SMC`, `L1AC`, or `CMAC`, the sliders in the **INAV Configurator PID Tuning tab** map to modern state-space parameters:

| Configurator Slider | Linear ADRC (`ADRC`) | CMAC Neural (`CMAC`) | L1-Adaptive (`L1AC`) | Sliding Mode (`SMC`) |
|---|---|---|---|---|
| **P Slider** | **Controller Bandwidth ($\omega_c$)**<br>Stick tracking sharpness & responsiveness | **Controller Bandwidth ($\omega_c$)**<br>Virtual PD tracking speed | **Baseline State Gain ($k_P$)**<br>Baseline reference model tracking | **Sliding Gain ($k_1$)**<br>Switching manifold velocity |
| **I Slider** | **Observer Bandwidth ($\omega_o$)**<br>Speed of wind & disturbance estimation | **Observer Bandwidth ($\omega_o$)**<br>DESO disturbance estimation speed | **Adaptation Rate ($\Gamma$)**<br>Fast parameter adaptation speed | **Integral Gain ($k_2$)**<br>Super-twisting integral drive |
| **D Slider** | **Motor Gain Scale ($b_0$)**<br>Physical thrust authority ($\text{Torque}/\text{Inertia}$) | **Motor Gain Scale ($b_0$)**<br>Physical thrust calibration | **LPF Cutoff Frequency (Hz)**<br>Filters motor noise from adaptation | **Manifold Slope ($\lambda$)**<br>Sliding surface tracking slope |
| **FeedForward** | **Stick Lead ($k_{CD}$)**<br>Pilot stick feedforward command | **Learning Rate ($\eta$)**<br>Cerebellar weight adaptation step | **Stick Lead ($k_{CD}$)**<br>Feedforward command | **Stick Lead ($k_{CD}$)**<br>Feedforward command |

---

## Step-by-Step WSL Build & Installation Guide

Building INAV on Windows 10 or 11 using **WSL (Windows Subsystem for Linux)** with Ubuntu is fast, robust, and reproducible.

### 1. Prerequisites (Ubuntu on WSL)

Open Windows PowerShell as Administrator and ensure WSL is installed:
```powershell
wsl --install -d Ubuntu
```
*(If already installed, simply launch **Ubuntu** from your Windows Start Menu).*

Inside the Ubuntu terminal, update your package lists and install the build tools:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git make ruby cmake gcc build-essential -y
```

> [!NOTE]
> **ARM Cross-Compiler:** You do **not** need to manually install `gcc-arm-none-eabi`. When you run `cmake ..`, INAV's build system automatically detects if an ARM toolchain is present; if not, it automatically downloads and verifies the official ARM GCC cross-compiler toolchain into your build directory.

---

### 2. Cloning & Building the Firmware

#### Single-Line Build Command (Example for `RADIOLINKF722`):
```bash
git clone https://github.com/Boyyt357/ADRC-inav.git && cd ADRC-inav && mkdir build && cd build && cmake .. && make RADIOLINKF722
```

#### Step-by-Step Breakdown:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Boyyt357/ADRC-inav.git
   cd ADRC-inav
   ```
   *(Note: If working with a pre-extracted release archive such as `inav-10.0.0-rc2`, enter that directory instead: `cd inav-10.0.0-rc2`).*

2. **Prepare the CMake build directory (out-of-source build):**
   ```bash
   mkdir build
   cd build
   cmake ..
   ```

3. **Compile your specific flight controller target:**
   ```bash
   make RADIOLINKF722
   ```
   *(Tip: Speed up compilation using all available CPU threads: `make -j$(nproc) RADIOLINKF722`).*

4. **Locate your compiled `.hex` binary:**
   Upon successful compilation, your firmware file will be ready at:
   ```bash
   ls -lh bin/inav_*_RADIOLINKF722.hex
   ```

5. **Access the `.hex` file from Windows:**
   In Windows Explorer, navigate to:
   `\\wsl$\Ubuntu\home\<your-user>\ADRC-inav\build\bin\`
   *(or copy it directly to your Windows drive: `cp bin/*.hex /mnt/c/Users/<YourUser>/Desktop/`).*

---

### 3. Flashing via INAV Configurator

1. Open **INAV Configurator** (version 7.x or 8.x+).
2. Connect your flight controller via USB in DFU mode (or click **Reboot to DFU**).
3. Navigate to the **Firmware Flasher** tab.
4. Click **Load Firmware [Local]** at the bottom right.
5. Select the compiled `.hex` file (e.g., `inav_10.0.0_RADIOLINKF722.hex`).
6. Toggle **Full chip erase** (recommended for major updates).
7. Click **Flash Firmware**.

---

## Quick Start & CLI Configuration

Once flashed, configure the controller type via the INAV CLI tab:

### Activate Linear ADRC:
```text
# Switch rate controller to ADRC
set pid_type = ADRC

# Recommended default bandwidths (Roll & Pitch)
set mc_p_pitch = 45      # Controller Bandwidth wc (rad/s)
set mc_i_pitch = 180     # Observer Bandwidth wo (rad/s)
set mc_d_pitch = 50      # Motor Gain Scale b0 (50 * 10 = 500)
set mc_cd_pitch = 15     # Stick Feedforward

set mc_p_roll = 45
set mc_i_roll = 180
set mc_d_roll = 50
set mc_cd_roll = 15

# Yaw axis
set mc_p_yaw = 35
set mc_i_yaw = 120
set mc_d_yaw = 50

save
```

### Activate Other Controllers:
* **Super-Twisting Sliding Mode:** `set pid_type = SMC`
* **L1-Adaptive Control:** `set pid_type = L1AC`
* **ADRC + CMAC Neural Network:** `set pid_type = CMAC`

---

## Tuning Guidelines

### Linear ADRC 3-Step Tuning Procedure
1. **Calibrate $b_0$ (D Slider):**
   - Start around default `50` (internal $b_0 = 500$).
   - If motors run hot or oscillate rapidly at high throttle, increase $b_0$ (tells the controller your motors produce higher thrust per unit command).
   - If response feels weak, decrease $b_0$.
2. **Set Controller Bandwidth $\omega_c$ (P Slider):**
   - Increase $\omega_c$ (default `40`–`65`) until stick feel is sharp, crisp, and locked in.
   - Because $k_d = 2\omega_c$, the algorithm automatically maintains a critical damping ratio ($\zeta = 1.0$), avoiding classical P-term bounce.
3. **Set Observer Bandwidth $\omega_o$ (I Slider):**
   - Rule of thumb: $\omega_o \approx (3 \sim 5) \times \omega_c$.
   - Increase $\omega_o$ (default `150`–`350`) to accelerate wind gust cancellation and eliminate prop wash in dirty air.

---

## Blackbox Telemetry & Logging

When `pid_type = ADRC` is active, Blackbox logs all internal DESO states transparently:
* **`axisPID_P`**: Virtual state tracking error $(k_p \cdot (r_{\text{target}} - z_1)) / b_0$
* **`axisPID_D`**: Virtual rate damping $(-k_d \cdot z_2) / b_0$
* **`axisPID_I`**: Real-time total disturbance cancellation $(-z_3) / b_0$
* **`axisPID_F`**: Stick Feedforward command ($k_{CD}$)

In INAV Blackbox Explorer, watch the **I-term trace**: it represents the exact real-time aerodynamic force and physical disturbance that the observer is counteracting!

---

## Safety Features

* **Disarm State Reset:** All observer states ($z_1, z_2, z_3$) and previous outputs automatically reset to neutral upon disarm, preventing aggressive motor spool-up or jump upon re-arming.
* **Disturbance Anti-Windup Clamping:** The estimated disturbance $z_3$ is clamped by `pidItermLimitPercent` to prevent observer windup during sustained motor saturation.
* **Antigravity Boost Integration:** Integrates with INAV's `USE_ANTIGRAVITY` feature—during rapid throttle punch-outs, $k_p$ and $k_d$ scale smoothly to preserve attitude lock.

---

## Credits & License

* **Developer:** Godwin Pious ([@Boyyt357](https://github.com/Boyyt357))
* **Base Project:** [INAV Flight Controller](https://github.com/iNavFlight/inav) (v10.0.0-rc2)
* **License:** GNU General Public License v3.0 ([GPL-3.0](LICENSE))
