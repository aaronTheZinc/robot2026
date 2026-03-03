# Motor Test Mode

This document describes how to test individual motors using the robot's Test mode. Motor test runs when the robot is **disabled** (e.g. from the dashboard/NetworkTables) or when the Driver Station is in **Test** mode (controller buttons are active).

---

## Driver Station Setup

1. **Enable the robot** (so Test mode can run).
2. **Select Test** in the Driver Station (dropdown: Disabled / Autonomous / Teleop / **Test**).
3. In Test mode, normal drive and intake bindings are disabled. Only motor test bindings and the test commands on SmartDashboard are active.

---

## Controllers

- **Controller 0 (Joystick)** — Drive and steer module tests (drive0–3, steer0–3).
- **Controller 1 (Subsystems)** — Shooter and intake tests (shooter left/right, hood, roller, pivot, hopper).

**Test speed:** Push **Controller 1 (Subsystems) left stick Y** forward for positive speed, pull back for negative. Speed is clamped to each motor’s configured min/max (e.g. hood and pivot are limited). If you don’t move the stick, the motor runs at 0 when the button is held (or use SmartDashboard for fixed speed).

---

## Controller 0 (Joystick) — Drive & Steer

| Button       | Motor   | Description              |
|-------------|---------|--------------------------|
| **A**       | Drive 0 | Front-left drive wheel   |
| **B**       | Drive 1 | Front-right drive wheel  |
| **X**       | Drive 2 | Back-left drive wheel    |
| **Y**       | Drive 3 | Back-right drive wheel   |
| **Left bumper**  | Steer 0 | Front-left steer (azimuth)  |
| **Right bumper** | Steer 1 | Front-right steer (azimuth) |
| **D-pad Up**     | Steer 2 | Back-left steer (azimuth)   |
| **D-pad Down**   | Steer 3 | Back-right steer (azimuth) |

Hold the button to run that motor; release to stop. Drive and steer use a low voltage limit during test.

---

## Controller 1 (Subsystems) — Shooter & Intake

| Button       | Motor        | Description           |
|-------------|--------------|-----------------------|
| **A**       | Shooter Left | Left shooter wheel    |
| **B**       | Shooter Right| Right shooter wheel   |
| **X**       | Hood         | Hood angle motor      |
| **Y**       | Intake Roller| Roller (in/out)       |
| **Left bumper**  | Intake Pivot | Pivot (up/down)   |
| **Right bumper** | Hopper       | Hopper (feed to shooter) |

Hold the button to run that motor. Speed is set by **Controller 1 left stick Y** (forward = positive, back = negative). Hood and pivot have reduced max speed for safety.

---

## SmartDashboard / Shuffleboard

When connected to the robot, open the **SmartDashboard** (or Shuffleboard) tab. Under **Motor Test/** you’ll see one button per motor, for example:

- Motor Test/Shooter Left  
- Motor Test/Shooter Right  
- Motor Test/Hood  
- Motor Test/Intake Roller  
- Motor Test/Intake Pivot  
- Motor Test/Hopper  
- Motor Test/Drive 0 … Drive 3  
- Motor Test/Steer 0 … Steer 3  

**Click a button** to run that motor at a fixed default speed (0.15). Click again (or run another command) to stop. This works when the robot is **disabled** or in **Test** mode.

---

## Inverting a Motor

Invert is applied only during motor test (it does not change normal teleop/auto).

- **From Shuffleboard/SmartDashboard:** Use the `MotorTest/Invert_<motorId>` entry (e.g. `MotorTest/Invert_shooterLeft`). Set to `true` to flip the direction for that motor in test.
- **Defaults** are set in each subsystem’s constants file (ShooterConstants, IntakeConstants, DriveConstants). The robot seeds these to NetworkTables on boot.

---

## Quick Reference

| Step | Action |
|------|--------|
| 1 | Enable robot, then set Driver Station mode to **Test**. |
| 2 | Use **Controller 1 left stick Y** to set test speed (forward/back). |
| 3 | **Controller 0:** A/B/X/Y = drive 0–3; bumpers = steer 0–1; D-pad up/down = steer 2–3. |
| 4 | **Controller 1:** A/B/X/Y = shooter left, shooter right, hood, intake roller; bumpers = intake pivot, hopper. |
| 5 | Or use **Motor Test/** buttons on SmartDashboard for fixed-speed testing. |

---

## Safety Notes

- Hood and pivot have **reduced max speed** in test (see ShooterConstants, IntakeConstants).
- Drive and steer modules use a **low voltage limit** (see DriveConstants) during test.
- Always ensure the robot is **not in a position to drive into people or obstacles** when testing drive or steer motors.
