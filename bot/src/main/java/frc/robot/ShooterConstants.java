// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** CAN IDs and gearing constants for the shooter and hood. */
public final class ShooterConstants {
    private ShooterConstants() {}

    /** CAN ID for the left shooter Talon FX. */
    public static final int kShooterLeftId = 41;
    /** CAN ID for the right shooter Talon FX. */
    public static final int kShooterRightId = 40;
    /** CAN ID for the hood Talon FX. */
    public static final int kHoodMotorId = 50;

    /**
     * Hood motion (degrees) per motor revolution.
     * 10T pinion on 20-tooth rack over 90° → 10 × (90/20) = 45° per motor rev.
     */
    public static final double kHoodDegreesPerMotorRev = 45.0;

    /** Optional soft limit: minimum hood angle in degrees (e.g. 0). */
    public static final double kHoodMinAngleDeg = 0;
    /** Optional soft limit: maximum hood angle in degrees (e.g. 90). */
    public static final double kHoodMaxAngleDeg = 90;

    /** Hood position PID gains (slot 0). Tune for your mechanism. */
    public static final double kHoodKp = 2.0;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 0.1;
    /** Open-loop hood hold/move gain in volts per degree of angle error. */
    public static final double kHoodAngleErrorVoltsPerDeg = 0.2;
    /** Minimum voltage magnitude to overcome hood stiction when moving toward a setpoint. */
    public static final double kHoodAngleControlMinVoltageVolts = 0.4;
    /** Maximum voltage magnitude when driving hood from angle error. */
    public static final double kHoodAngleControlMaxVoltageVolts = 6.0;
    /** Angle error deadband where hood voltage control stops driving. */
    public static final double kHoodAngleToleranceDeg = 0.5;

    /**
     * Homing (real robot only): run hood toward the zero/mechanical-stop position until stall.
     * Voltage (V) applied during homing; negative = toward mechanical stop / hood down.
     */
    public static final double kHoodHomingVoltageVolts = -2.0;
    /**
     * Stator current (A) above which we consider the hood at the mechanical stop.
     * Increase if homing triggers too early; decrease if it never triggers.
     */
    public static final double kHoodStallCurrentAmps = 25.0;
    /** Number of consecutive cycles current must be above threshold to confirm stall. */
    public static final int kHoodStallConfirmCycles = 5;
    /** Hood stator current limit (A) during normal operation and homing. */
    public static final double kHoodStatorCurrentLimitAmps = 35.0;

    /** Max voltage (volts) for converting normalized speed [-1, 1] to voltage. */
    public static final double kMaxVoltageVolts = 12.0;

    /** Test speed [-1, 1] for shooter test commands (e.g. 0.25 for reduced testing). */
    public static final double kTestSpeed = 0.25;

    // ----- Closed-loop shooter velocity (RPM) and dashboard increments -----

    /** Minimum shooter wheel RPM setpoint (closed-loop). */
    public static final double kShooterMinRpm = 0.0;
    /** Maximum shooter wheel RPM setpoint (closed-loop). */
    public static final double kShooterMaxRpm = 6000.0;
    /** Default RPM setpoint when using velocity control (e.g. for testing). */
    public static final double kShooterDefaultRpmSetpoint = 3000.0;

    /** Wing shot profile: hood angle (deg). */
    public static final double kWingShotHoodAngleDeg = 20.0;
    /** Wing shot profile: shooter wheel speed (RPM). */
    public static final double kWingShotRpm = 4100.0;
    /** Left bumper shot profile: hood angle (deg). */
    public static final double kLeftBumperShotHoodAngleDeg = 10.0;
    /** Left bumper shot profile: shooter wheel speed (RPM). */
    public static final double kLeftBumperShotRpm = 4100.0;

    /** Shooter velocity PID gains (slot 0). Tune for your mechanism. */
    public static final double kShooterKp = 0.15;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.01;

    /** Dashboard: hood angle step in degrees for increment/decrement buttons. */
    public static final double kHoodDegreesIncrement = 5.0;
    /** Dashboard: shooter RPM step for increment/decrement buttons. */
    public static final double kShooterRpmIncrement = 100.0;

    // ----- Motor test (all test speeds and inversion for shooter/hood in this file) -----

    /** Motor test: min/max normalized speed and default invert for left shooter. */
    public static final double kShooterLeftTestMinSpeed = -1.0;
    public static final double kShooterLeftTestMaxSpeed = 1.0;
    public static final boolean kShooterLeftTestDefaultInvert = false;

    /** Motor test: min/max normalized speed and default invert for right shooter. */
    public static final double kShooterRightTestMinSpeed = -1.0;
    public static final double kShooterRightTestMaxSpeed = 1.0;
    public static final boolean kShooterRightTestDefaultInvert = false;

    /** Motor test: min/max normalized speed and default invert for hood (limited for safety). */
    public static final double kHoodTestMinSpeed = -0.3;
    public static final double kHoodTestMaxSpeed = 0.3;
    public static final boolean kHoodTestDefaultInvert = false;
}
