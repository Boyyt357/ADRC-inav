# **ADRC-INAV 🚀**

Welcome to **ADRC-INAV**, a developmental fork of INAV that introduces **Active Disturbance Rejection Control (ADRC)** to flight stabilization and navigation.

Instead of reacting entirely to past errors like traditional PID loops, ADRC utilizes an **Extended State Observer (ESO)**. The ESO estimates real-time external disturbances (wind gusts, voltage drops, aerodynamical shifts, center-of-gravity changes) as a single "total disturbance" value and actively cancels it out instantly.

## **🧠 Why ADRC? (ADRC vs. Traditional PID)**

* **Simplified Tuning:** No more balancing P, I, and D gains against each other. With ADRC, you are primarily tuning a single, intuitive parameter: the controller's **bandwidth**.  
* **Aggressive Disturbance Rejection:** The aircraft fights wind and turbulent environments much more actively, ensuring a locked-in and ultra-stable flight experience.  
* **Smooth Stick Feel:** Exceptional tracking response that translates to incredibly natural and linear control inputs.

## **💾 The Two Firmware Variants**

Depending on your hardware setup and tuning preferences, this project offers two distinct operational modes:

### **1\. Basic ADRC (Half ADRC) — *Main Branch***

* **Core Flight Axes:** ADRC strictly handles **Pitch, Roll, and Yaw**.  
* **Navigation & Alt-Hold:** Automatically falls back to standard, highly-optimized INAV PID controls for Barometer altitude and GPS position tracking.  
* **Why use it?** INAV’s default PID parameters for altitude and GPS tracking are already incredibly well-optimized. If you want a reliable, rock-solid cruiser with incredibly smooth manual stick feel without overcomplicating autonomous modes, this is the default version.

### **2\. Full ADRC — *Experimental (Available in Releases)***

* **Core & Nav Axes:** ADRC handles absolutely everything (Pitch, Roll, Yaw, Baro Altitude Hold, and GPS Position Hold).  
* **The Catch:** Barometers and GPS sensors are inherently noisy. Because ADRC constantly tries to estimate and correct for instantaneous errors, noisy sensor data can cause the controller to over-correct ("chase ghosts").  
* **Why use it?** For control theory nerds and developers who love pushing the boundaries of autonomous estimation and filtering. You can download the compiled full-ADRC binaries directly from the [**Releases**](http://docs.google.com/releases) tab.

## **🛠️ Compilation & Installation (WSL / Ubuntu)**

You can clone the repository, install all necessary system packages, configure the workspace, and compile the custom firmware for the **DAKE FPV F405** flight controller using this single automated terminal command:

git clone \[https://github.com/Boyyt357/ADRC-inav.git\](https://github.com/Boyyt357/ADRC-inav.git) && sudo apt update && sudo apt upgrade \-y && sudo apt install git make ruby cmake gcc build-essential \-y && cd ADRC-inav && mkdir build && cd build && cmake .. && make DAKEFPVF405

Once compilation completes, you will find your custom .hex binary ready for flashing inside the build/ directory.

## **⚠️ Critical Upstream PSAs (Must Read\!)**

If you are upgrading from an older flight controller setup or older firmware, take note of these key changes:

### **1\. STM32 F411 Deprecation**

\[\!WARNING\]

INAV no longer officially accepts targets based on the STM32 F411 MCU. INAV 7 was the final release for these boards. This ADRC branch targets more modern MCUs (like the F405 on the **DAKE FPV F405**).

### **2\. ICM426xx IMU Filtering**

\[\!IMPORTANT\]

The filtering settings for the ICM426xx IMU match the advanced defaults used by Ardupilot and Betaflight. When upgrading from older INAV setups, **you must recalibrate your Accelerometer** and verify your tune.

### **3\. GPS Module Requirements (M8 or Newer)**

\[\!WARNING\]

Legacy GPS modules (UBLOX M6, M7, and older) are deprecated. This project requires GPS units supporting **UBLOX Protocol version 15.00 or newer** (UBLOX M8, M9, or M10).

You can verify your module protocol version by running the status command in the INAV CLI.

## **🧰 Official Ecosystem Tools**

Configure and monitor your ADRC flights using the standard ecosystem tools:

* [**INAV Configurator**](https://github.com/iNavFlight/inav-configurator/releases): The desktop application used for flashing and configuring settings.  
* [**INAV Blackbox Explorer**](https://github.com/iNavFlight/blackbox-log-viewer/releases): Essential tool for visualizing state estimator variables and gyro tracking performance.  
* [**EdgeTX/OpenTX Telemetry Widget**](https://github.com/iNavFlight/OpenTX-Telemetry-Widget): Highly customizable real-time telemetry display on your radio transmitter.

## **🤝 Contributing & Feedback**

This is an actively developing project. Bug reports, logic optimization pull requests, and flight logs are highly appreciated\!

Feel free to open an **Issue** or submit a **Pull Request** if you have improvements for the Extended State Observer algorithms or new target definitions.

*Fly safe, fly smart, and enjoy the power of Active Disturbance Rejection Control\!*
