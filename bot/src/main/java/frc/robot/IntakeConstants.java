// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** CAN IDs and constants for the intake (3 Spark MAXs: pivot, roller, hopper). */
public final class IntakeConstants {
    private IntakeConstants() {}

    /** CAN ID for the pivot Spark MAX (up/down). */
    public static final int kPivotId = 51;
    /** CAN ID for the roller Spark MAX (in/out). */
    public static final int kRollerId = 52;
    /** CAN ID for the hopper Spark MAX (feeds shooter). */
    public static final int kHopperId = 53;

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

    /** Roller speed [-1, 1] when intaking (positive = pull in). */
    public static final double kRollerIntakeSpeed = 0.8;
    /** Roller speed [-1, 1] when outtaking (negative = push out). */
    public static final double kRollerOuttakeSpeed = -0.8;

    /** Hopper speed [-1, 1] when feeding the shooter. */
    public static final double kHopperFeedSpeed = 0.7;

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
    public static final double kHopperTestMinSpeed = -1.0;
    public static final double kHopperTestMaxSpeed = 1.0;
    public static final boolean kHopperTestDefaultInvert = false;
}
