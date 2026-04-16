// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** CAN IDs and constants for the intake (4 Spark MAXs: pivot, roller, two hopper). */
public final class IntakeConstants {
    private IntakeConstants() {}

    /**
     * When false, robot enable does not run intake pivot homing and the subsystem Start+R1 homing binding is off;
     * use X/Y manual open-loop until homing is re-enabled.
     */
    public static final boolean kEnableIntakePivotHomingAtEnable = false;

    /** CAN ID for the pivot Spark MAX (up/down). */
    public static final int kPivotId = 33;
    /** SPARK MAX smart current limit (A) for the pivot. Tune with mechanism load / breaker budget. */
    public static final int kPivotSmartCurrentLimitAmps = 20;
    /**
     * After the first successful collect-direction mechanical homing this enable, later collect homing runs use
     * this higher limit so the gearbox can develop enough torque at the down stop.
     */
    public static final int kPivotCollectHomingSmartCurrentLimitAmpsAfterFirst = 40;
    /**
     * While X/Y PID snap runs (after {@link #kPivotPidToleranceRotations} setpoints are valid), pivot uses this
     * high limit so the controller is not starved by the default smart limit.
     */
    public static final int kPivotPidSnapSmartCurrentLimitAmps = 80;
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
    /** Max normalized output for X/Y encoder snap after homing (full range; current limit handles thermal). */
    public static final double kPivotPidSnapMaxOutput = 1.0;
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
    /**
     * Homing toward stow uses this fraction of {@link #kPivotStowSpeed} so direction always matches teleop stow
     * (stall/limit finds the up stop); keep below 1.0 for a gentler approach.
     */
    public static final double kPivotHomingStowDutyFraction = 0.45;
    /**
     * Homing toward collect uses this fraction of {@link #kPivotCollectSpeed} (same direction as teleop collect);
     * range-of-motion is learned when current spikes at the down stop.
     */
    public static final double kPivotHomingCollectDutyFraction = 1.0;
    /** Normalized speed [-1, 1] for pivot moving toward stow (up). */
    public static final double kPivotStowSpeed = -0.6;
    /** Normalized speed [-1, 1] for pivot moving toward collect (down). */
    public static final double kPivotCollectSpeed = 0.5;
    /** Output current (A) above which pivot is considered at mechanical stop (stow homing). */
    public static final double kPivotStallCurrentAmps = 5.0;
    /** Consecutive cycles current must be above threshold to confirm stall (stow homing). */
    public static final int kPivotStallConfirmCycles = 5;
    /**
     * Collect-direction homing stall threshold (A). Lower = finish mechanical down sooner with less draw.
     */
    public static final double kPivotCollectHomingStallCurrentAmps = 8.0;
    /** Consecutive cycles above threshold to confirm down stop — lower = shorter time window. */
    public static final int kPivotCollectHomingStallConfirmCycles = 10;
    /** Cycles before collect homing stall detection starts (ignore inrush). Lower = shorter pre-window. */
    public static final int kPivotCollectHomingIgnoreStallCycles = 8;
    /**
     * Collect setpoint seek: if still short of encoder target but output current stays here (A), treat as wedged
     * at the stop and finish (avoids grinding).
     */
    public static final double kPivotCollectSeekStallStopAmps = 6.0;
    /** Consecutive high-current cycles to finish collect seek when mechanically blocked. */
    public static final int kPivotCollectSeekStallConfirmCycles = 4;
    /** Max time (s) for stow mechanical homing; safety timeout if stall never triggers. */
    public static final double kPivotStallTimeoutSeconds = 5.0;
    /** Max time (s) for collect mechanical homing — allow long travel before the stop loads. */
    public static final double kPivotCollectHomingStallTimeoutSeconds = 10.0;
    /** Max time (s) for X/Y setpoint seek after homing (encoder-only travel). */
    public static final double kPivotSetpointSeekTimeoutSeconds = 10.0;
    /** Open-loop pivot toward collect (down) for this long, then stop (e.g. short deploy pulse). */
    public static final double kPivotTowardCollectPulseSeconds = 0.3;

    /**
     * DIO channel for pivot fully up (stow) limit; {@code < 0} = not wired (stall + encoder only).
     */
    public static final int kPivotUpLimitDioChannel = -1;
    /**
     * DIO channel for pivot fully down (collect) limit; {@code < 0} = not wired (stall + encoder only).
     */
    public static final int kPivotDownLimitDioChannel = -1;

    /**
     * Default pivot "down" position (motor rotations) before homing learns the real value from the stop.
     */
    public static final double kPivotDownPositionRotationsDefault = 42.0;
    /** Within ±this many rotations of the learned down position, intake is considered at down. */
    public static final double kPivotDownPositionToleranceRotations = 5.0;
    /** Within this many rotations of 0, intake is considered at stow after homing. */
    public static final double kPivotStowPositionToleranceRotations = 0.08;

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
