// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** CAN IDs and constants for the intake (4 Spark MAXs: pivot, roller, two hopper). */
public final class IntakeConstants {
    private IntakeConstants() {}

    /** CAN ID for the pivot Spark MAX (up/down). */
    public static final int kPivotId = 33;
    /** SPARK MAX smart current limit (A) for the pivot. Tune with mechanism load / breaker budget. */
    public static final int kPivotSmartCurrentLimitAmps = 20;
    /** CAN ID for the roller Spark MAX (in/out). */
    public static final int kRollerId = 31;
    /** CAN IDs for the two hopper Spark MAXs (run together for feed / spit). */
    public static final int kHopperMotorACanId = 55;
    public static final int kHopperMotorBCanId = 60;

    /**
     * Hopper A SPARK MAX: invert motor direction in firmware so it opposes B for the same signed duty.
     * If motor A does not turn while B does, try flipping this after confirming CAN IDs on the bus.
     */
    public static final boolean kHopperMotorAInverted = true;

    /** Smart current limit (A) for each hopper SPARK MAX. */
    public static final int kHopperSmartCurrentLimitAmps = 40;

    /** Pivot position (motor rotations) for stow (retracted). */
    public static final double kPivotStowRotations = 0.0;
    /** Pivot position (motor rotations) for collect (deployed to pick from floor). */
    public static final double kPivotCollectRotations = 0.5;
    /** Pivot position (motor rotations) for intake (same as collect or slight variation). */
    public static final double kPivotIntakeRotations = 0.5;
    /** Pivot position (motor rotations) for outtake (e.g. toward amp/speaker). */
    public static final double kPivotOuttakeRotations = 0.25;

    /** Pivot PID slot 0 gains. Tune for your mechanism. */
    public static final double kPivotKp = 0.5;
    public static final double kPivotKi = 0.0;
    public static final double kPivotKd = 0.02;
    /** Max normalized output used by software PID for pivot position hold. */
    public static final double kPivotPidMaxOutput = 0.5;
    /** Error tolerance (motor rotations) for considering pivot at setpoint in PID mode. */
    public static final double kPivotPidToleranceRotations = 0.02;

    /** Roller speed [-1, 1] when intaking (positive = pull in). */
    public static final double kRollerIntakeSpeed = 0.8;
    /** Roller speed [-1, 1] when outtaking (negative = push out). */
    public static final double kRollerOuttakeSpeed = -0.8;

    /** Hopper speed [-1, 1] when feeding the shooter. */
    public static final double kHopperFeedSpeed = -0.8;
    /** Hopper speed [-1, 1] when spitting out (reverse feed). */
    public static final double kHopperSpitOutSpeed = 0.8;

    // ----- Pivot stop-based control (mechanical stops, no encoder setpoints) -----
    /** Normalized speed [-1, 1] for homing toward stow (init / Y+B); keep slow for safety. */
    public static final double kPivotHomingSpeed = 0.6;
    /** Normalized speed [-1, 1] for homing toward collect (down) until stall; keep slow for safety. */
    public static final double kPivotCollectHomingSpeed = 0.2;
    /** Normalized speed [-1, 1] for pivot moving toward stow (up). */
    public static final double kPivotStowSpeed = -0.6;
    /** Normalized speed [-1, 1] for pivot moving toward collect (down). */
    public static final double kPivotCollectSpeed = 0.2;
    /** Output current (A) above which pivot is considered at mechanical stop. */
    public static final double kPivotStallCurrentAmps = 25.0;
    /** Consecutive cycles current must be above threshold to confirm stall. */
    public static final int kPivotStallConfirmCycles = 5;
    /** Max time (s) for stow or collect command; safety timeout if stall never triggers. */
    public static final double kPivotStallTimeoutSeconds = 5.0;

    /**
     * Pivot relative position (motor rotations) that counts as "down" for dashboard / logic.
     */
    public static final double kPivotDownPositionRotations = 42.0;
    /** Within ±this many rotations of {@link #kPivotDownPositionRotations}, intake is considered at down. */
    public static final double kPivotDownPositionToleranceRotations = 5.0;

    /** How often {@link frc.robot.subsystems.IntakeSubsystem} pushes intake fields to SmartDashboard (s). */
    public static final double kIntakeDashboardPublishPeriodSeconds = 1.0;

    // ----- Motor test (all test speeds and inversion for intake motors in this file) -----

    /** Motor test: min/max normalized speed and default invert for roller. */
    public static final double kRollerTestMinSpeed = -1.0;
    public static final double kRollerTestMaxSpeed = 1.0;
    public static final boolean kRollerTestDefaultInvert = false;

    /** Motor test: min/max normalized speed and default invert for pivot (limited for safety). */
    public static final double kPivotTestMinSpeed = -0.5;
    public static final double kPivotTestMaxSpeed = 0.5;
    public static final boolean kPivotTestDefaultInvert = false;

    /** Motor test: min/max normalized speed and default invert for hopper. */
    public static final double kHopperTestMinSpeed = 1.0;
    public static final double kHopperTestMaxSpeed = -1.0;
    public static final boolean kHopperTestDefaultInvert = false;
}
