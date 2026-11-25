# LUBRITORQUE

Automated Electro-Mechanical Lubricant Condition Diagnostic Platform.

## Overview
**LubriTorque** evaluates grease degradation by executing a controlled 1-revolution spread test on a 123mm disc. An ESP32 microcontroller captures instantaneous rotational velocity profiles (36 intervals) and motor power metrics, streaming live telemetry to a Supabase backend and a web diagnostic dashboard.

## System Architecture
- **Controller:** ESP32 (Dual-core: Core 1 for microsecond encoder interrupts & INA226 sampling; Core 0 for non-blocking Supabase telemetry streaming)
- **Actuation & Sensing:** BTS7960 motor driver (with electromagnetic braking), quadrature encoder (`3125` CPR Quad / `1893` CPR Single-Ch fallback), and INA226 I2C current/voltage/power sensor (`0.1 Ω` shunt)
- **Backend & Frontend:** Supabase (PostgreSQL + Realtime WebSockets) and a responsive HTML/JS/Chart.js dashboard hosted on Vercel

## Quick Setup
1. **Database (Supabase):** Ensure `device_control`, `lubritorque_logs`, `lubritorque_mech_patterns`, and `calibration_profiles` tables are created with Realtime enabled on `device_control` and `lubritorque_logs`.
2. **Firmware (ESP32):** Update Wi-Fi/Supabase credentials in `supabaseesp32lubritorque.ino` and flash via Arduino IDE (`115200` baud). Serial commands supported: `TEST` (offline 1-rev run), `CAL` (interactive calibration), and `SET <cpr>`.
3. **Web Dashboard (Vercel):** Deploy `index.html` to Vercel (or open locally) and click **RUN 1-ROTATION MEASUREMENT**.

## Workflow
1. **Calibration (`⚙ Calibration`, PIN: `2026`):** Run 1-revolution scans for known **Fresh**, **Semi-Degraded**, and **Degraded** samples, input the $10^\circ$ spread divisions (0–36), and save the reference model.
2. **Diagnostics:** Run a 1-rotation scan on an unknown sample, enter the 10 degree division count, and click **Analyze Grease Condition** to classify lubricant health and inspect the instantaneous RPM vs. Time curve.
