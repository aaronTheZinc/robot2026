// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** CAN IDs and gearing constants for the shooter and hood. */
public final class ShooterConstants {
    private ShooterConstants() {}

    /** CAN ID for the left shooter Talon FX. */
    public static final int kShooterLeftId = 40;
    /** CAN ID for the right shooter Talon FX. */
    public static final int kShooterRightId = 41;
    /** CAN ID for the hood Talon FX. */
    public static final int kHoodMotorId = 50;

    /**
     * Hood motion (degrees) per motor revolution.
     * 10T pinion on 20-tooth rack over 90° → 10 × (90/20) = 45° per motor rev.
     */
    public static final double kHoodDegreesPerMotorRev = 45.0;

    /** Optional soft limit: minimum hood angle in degrees (e.g. 0). */
    public static final double kHoodMinAngleDeg = 0.0;
    /** Optional soft limit: maximum hood angle in degrees (e.g. 90). */
    public static final double kHoodMaxAngleDeg = 90.0;

    /** Hood position PID gains (slot 0). Tune for your mechanism. */
    public static final double kHoodKp = 2.0;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 0.1;

    /**
     * Homing (real robot only): run hood backward until stall.
     * Voltage (V) applied during homing; negative = toward mechanical stop.
     */
    public static final double kHoodHomingVoltageVolts = -2.0;
    /**
     * Stator current (A) above which we consider the hood at the mechanical stop.
     * Increase if homing triggers too early; decrease if it never triggers.
     */
    public static final double kHoodStallCurrentAmps = 15.0;
    /** Number of consecutive cycles current must be above threshold to confirm stall. */
    public static final int kHoodStallConfirmCycles = 5;
    /** Hood stator current limit (A) during normal operation and homing. */
    public static final double kHoodStatorCurrentLimitAmps = 25.0;

    /** Max voltage (volts) for converting normalized speed [-1, 1] to voltage. */
    public static final double kMaxVoltageVolts = 12.0;

    /** Test speed [-1, 1] for shooter test commands (e.g. 0.25 for reduced testing). */
    public static final double kTestSpeed = 0.25;
}
