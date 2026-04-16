// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IntakeConstants;

/**
 * Intake with 4 SPARK MAXs (REV 2026 Spark API):
 * - Pivot: up/down via mechanical stops (homing at enable, then stow/collect open-loop + stall detect).
 * - Roller: in/out (duty cycle for intaking/outtaking).
 * - Hopper A/B: two motors (CAN {@link IntakeConstants#kHopperMotorACanId} /
 *   {@link IntakeConstants#kHopperMotorBCanId}) run together to feed the shooter.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor =
            new SparkMax(IntakeConstants.kPivotId, MotorType.kBrushless);
    private final SparkMax rollerMotor =
            new SparkMax(IntakeConstants.kRollerId, MotorType.kBrushless);
    private final SparkMax hopperMotorA =
            new SparkMax(IntakeConstants.kHopperMotorACanId, MotorType.kBrushless);
    private final SparkMax hopperMotorB =
            new SparkMax(IntakeConstants.kHopperMotorBCanId, MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    /** Optional limits; {@code null} when channel &lt; 0. */
    private final DigitalInput pivotUpLimit;

    private final DigitalInput pivotDownLimit;

    /** Encoder rotations at collect (down) stop; updated by full homing or down-only homing. */
    private double pivotDownRotationsLearned = IntakeConstants.kPivotDownPositionRotationsDefault;

    /** True after pivot has homed to stow stop this enable (mirrors hood shooterReady). */
    private boolean pivotReady;

    /**
     * Cleared on disable; after first successful collect-direction mechanical homing, later collect homings use
     * {@link IntakeConstants#kPivotCollectHomingSmartCurrentLimitAmpsAfterFirst}.
     */
    private boolean collectMechanicalHomingSucceededOnce;

    /** Software PID for encoder snap to stow/collect after homing (X/Y). */
    private final PIDController pivotPositionPid =
            new PIDController(
                    IntakeConstants.kPivotKp,
                    IntakeConstants.kPivotKi,
                    IntakeConstants.kPivotKd);

    /** After stall-based travel: true = at stow (up), false = at collect (down), null = unknown. */
    private Boolean pivotStowed = null;

    /** Last FPGA time we published to SmartDashboard; negative so the first loop always publishes. */
    private double lastDashboardPublishSec = -1.0;

    /** Web dashboard NT table {@code Intake} — same pattern as {@code /Shooter/*} debug topics. */
    private final NetworkTable intakeNetworkTable = NetworkTableInstance.getDefault().getTable("Intake");

    public IntakeSubsystem() {
        pivotUpLimit = makeLimitSwitch(IntakeConstants.kPivotUpLimitDioChannel);
        pivotDownLimit = makeLimitSwitch(IntakeConstants.kPivotDownLimitDioChannel);

        pivotPositionPid.setTolerance(IntakeConstants.kPivotPidToleranceRotations);
        var pivotConfigInitial =
                new SparkMaxConfig()
                        .smartCurrentLimit(IntakeConstants.kPivotSmartCurrentLimitAmps)
                        .idleMode(SparkBaseConfig.IdleMode.kCoast);
        pivotMotor.configure(
                pivotConfigInitial, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var rollerConfig = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var hopperBase =
                new SparkMaxConfig()
                        .idleMode(SparkBaseConfig.IdleMode.kBrake)
                        .smartCurrentLimit(IntakeConstants.kHopperSmartCurrentLimitAmps)
                        // Clear stale follower mode from REV Hardware Client so open-loop .set() drives this motor.
                        .disableFollowerMode();
        var hopperConfigA =
                new SparkMaxConfig()
                        .apply(hopperBase)
                        .inverted(IntakeConstants.kHopperMotorAInverted);
        var hopperConfigB = new SparkMaxConfig().apply(hopperBase).inverted(false);
        hopperMotorA.configure(hopperConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperMotorB.configure(hopperConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        publishPivotDashboardNetworkTables();

        double now = Timer.getFPGATimestamp();
        if (now - lastDashboardPublishSec < IntakeConstants.kIntakeDashboardPublishPeriodSeconds) {
            return;
        }
        lastDashboardPublishSec = now;

        double pivotRelRot = getPivotRotations();
        SmartDashboard.putNumber("Intake/Pivot Relative Position (rot)", pivotRelRot);
        SmartDashboard.putNumber("Intake/Pivot Stow Setpoint (rot)", 0.0);
        SmartDashboard.putNumber("Intake/Pivot Collect Setpoint (rot)", pivotDownRotationsLearned);
        SmartDashboard.putNumber("Intake/Pivot Down Learned (rot)", pivotDownRotationsLearned);
        SmartDashboard.putNumber(
                "Intake/Pivot Down Tolerance (rot)", IntakeConstants.kPivotDownPositionToleranceRotations);
        SmartDashboard.putBoolean("Intake/Pivot At Down", isPivotAtDownPosition());
        SmartDashboard.putBoolean("Intake/PivotReady", pivotReady);
        SmartDashboard.putNumber("Intake/RollerCurrentAmps", rollerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/RollerBusVoltage", rollerMotor.getBusVoltage());
        SmartDashboard.putNumber("Intake/HopperAAppliedDuty", hopperMotorA.getAppliedOutput());
        SmartDashboard.putNumber("Intake/HopperBAppliedDuty", hopperMotorB.getAppliedOutput());
        SmartDashboard.putNumber("Intake/HopperACurrentAmps", hopperMotorA.getOutputCurrent());
        SmartDashboard.putNumber("Intake/HopperBCurrentAmps", hopperMotorB.getOutputCurrent());
    }

    /**
     * Publishes pivot encoder + homing setpoints every loop for the web dashboard ({@code /Intake/*}).
     * Stow setpoint is always 0 after mechanical homing; collect setpoint is {@link #pivotDownRotationsLearned}.
     */
    private void publishPivotDashboardNetworkTables() {
        intakeNetworkTable.getEntry("pivotPositionRot").setDouble(getPivotRotations());
        intakeNetworkTable.getEntry("pivotStowSetpointRot").setDouble(0.0);
        intakeNetworkTable.getEntry("pivotCollectSetpointRot").setDouble(pivotDownRotationsLearned);
        intakeNetworkTable.getEntry("pivotReady").setBoolean(pivotReady);
    }

    // ----- Pivot (up/down) — stall homing learns range; teleop seeks encoder setpoints -----

    /** Whether the pivot has been homed this enable (stow stop found and encoder zeroed). */
    public boolean isPivotReady() {
        return pivotReady;
    }

    /** Set by homing command when stow stop is reached; cleared when disabled. */
    public void setPivotReady(boolean ready) {
        pivotReady = ready;
        if (!ready) {
            stopPivot();
            pivotDownRotationsLearned = IntakeConstants.kPivotDownPositionRotationsDefault;
            collectMechanicalHomingSucceededOnce = false;
            pivotPositionPid.reset();
            applyPivotDefaultCurrentLimit();
        }
    }

    private void configurePivotMotor(int smartCurrentLimitAmps) {
        var pivotConfig =
                new SparkMaxConfig()
                        .smartCurrentLimit(smartCurrentLimitAmps)
                        .idleMode(SparkBaseConfig.IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Normal travel / open-loop: default smart current limit. */
    public void applyPivotDefaultCurrentLimit() {
        configurePivotMotor(IntakeConstants.kPivotSmartCurrentLimitAmps);
    }

    /**
     * Call at the start of collect-direction mechanical homing: first run uses default limit; after the first
     * successful collect homing this enable, uses a higher limit.
     */
    public void applyPivotCurrentLimitForCollectHoming() {
        int amps =
                collectMechanicalHomingSucceededOnce
                        ? IntakeConstants.kPivotCollectHomingSmartCurrentLimitAmpsAfterFirst
                        : IntakeConstants.kPivotSmartCurrentLimitAmps;
        configurePivotMotor(amps);
    }

    /** Mark successful collect-direction mechanical homing so the next collect homing can use more current. */
    public void markCollectMechanicalHomingSucceeded() {
        collectMechanicalHomingSucceededOnce = true;
    }

    /** High smart current limit for PID snap (X/Y) so output is not capped by the default limit. */
    public void applyPivotPidSnapCurrentLimit() {
        configurePivotMotor(IntakeConstants.kPivotPidSnapSmartCurrentLimitAmps);
    }

    public void resetPivotPositionPid() {
        pivotPositionPid.reset();
    }

    /**
     * Closed-loop pivot toward {@code setpointRotations} (motor rotations). Call each cycle while holding PID snap
     * current limit.
     */
    public void runPivotPidToSetpoint(double setpointRotations) {
        double output =
                pivotPositionPid.calculate(getPivotRotations(), setpointRotations);
        output =
                MathUtil.clamp(
                        output,
                        -IntakeConstants.kPivotPidSnapMaxOutput,
                        IntakeConstants.kPivotPidSnapMaxOutput);
        pivotMotor.set(output);
    }

    /** True when within {@link IntakeConstants#kPivotPidToleranceRotations} of {@code setpointRotations}. */
    public boolean isPivotPidAtSetpoint(double setpointRotations) {
        return Math.abs(getPivotRotations() - setpointRotations)
                <= IntakeConstants.kPivotPidToleranceRotations;
    }

    private static DigitalInput makeLimitSwitch(int channel) {
        if (channel < 0) {
            return null;
        }
        return new DigitalInput(channel);
    }

    /** True when the up (stow) limit is active, if wired. */
    public boolean isPivotUpLimitPressed() {
        return pivotUpLimit != null && pivotUpLimit.get();
    }

    /** True when the down (collect) limit is active, if wired. */
    public boolean isPivotDownLimitPressed() {
        return pivotDownLimit != null && pivotDownLimit.get();
    }

    /** Rotations recorded at the down stop after homing (used for collect position). */
    public double getPivotDownRotationsLearned() {
        return pivotDownRotationsLearned;
    }

    /** Call when the pivot reaches the mechanical down stop during homing. */
    public void setPivotDownRotationsLearnedFromHoming() {
        pivotDownRotationsLearned = getPivotRotations();
    }

    /** Zero the pivot encoder (call when at a mechanical stop after the matching homing command). */
    public void zeroPivotPosition() {
        pivotEncoder.setPosition(0);
    }

    /** Run pivot slowly toward stow for homing — same sign as {@link #setPivotTowardStow()}. */
    public void setPivotTowardStowHoming() {
        pivotMotor.set(
                IntakeConstants.kPivotStowSpeed * IntakeConstants.kPivotHomingStowDutyFraction);
    }

    /** Run pivot slowly toward collect for homing — same sign as {@link #setPivotTowardCollect()}. */
    public void setPivotTowardCollectHoming() {
        pivotMotor.set(
                IntakeConstants.kPivotCollectSpeed * IntakeConstants.kPivotHomingCollectDutyFraction);
    }

    /** Run pivot toward stow (up) at normal travel speed. */
    public void setPivotTowardStow() {
        // if (!pivotReady) {
        //     return;
        // }
        pivotMotor.set(IntakeConstants.kPivotStowSpeed);
    }

    /** Run pivot toward collect (down) at normal travel speed. */
    public void setPivotTowardCollect() {
        // if (!pivotReady) {
        //     return;
        // }
        pivotMotor.set(IntakeConstants.kPivotCollectSpeed);
    }

    /** Get current pivot position in motor rotations (for telemetry; position control is stop-based). */
    public double getPivotRotations() {
        return pivotEncoder.getPosition();
    }  

    /**
     * True near the learned collect (down) position (from homing), or within tolerance of
     * {@link IntakeConstants#kPivotDownPositionRotationsDefault} before homing has run.
     */
    public boolean isPivotAtDownPosition() {
        double r = getPivotRotations();
        double tol = IntakeConstants.kPivotDownPositionToleranceRotations;
        return Math.abs(r - pivotDownRotationsLearned) <= tol;
    }

    /** True near stow (encoder zero) after homing. */
    public boolean isPivotAtStowPosition() {
        return Math.abs(getPivotRotations()) <= IntakeConstants.kPivotStowPositionToleranceRotations;
    }

    /**
     * Seek stow setpoint (0 rot): run toward stow while encoder is above band, toward collect if below (rare).
     * Stops motor when {@link #isPivotAtStowPosition()}.
     */
    public void runPivotSeekStowSetpoint() {
        double r = getPivotRotations();
        double tol = IntakeConstants.kPivotStowPositionToleranceRotations;
        if (Math.abs(r) <= tol) {
            stopPivot();
            return;
        }
        if (r > tol) {
            setPivotTowardStow();
        } else {
            setPivotTowardCollect();
        }
    }

    /**
     * Seek collect setpoint ({@link #pivotDownRotationsLearned}): run toward collect or stow by encoder error.
     * Stops motor when {@link #isPivotAtDownPosition()}.
     */
    public void runPivotSeekCollectSetpoint() {
        double r = getPivotRotations();
        double target = pivotDownRotationsLearned;
        double tol = IntakeConstants.kPivotDownPositionToleranceRotations;
        if (Math.abs(r - target) <= tol) {
            stopPivot();
            return;
        }
        if (r < target - tol) {
            setPivotTowardCollect();
        } else {
            setPivotTowardStow();
        }
    }

    // ----- Roller (in/out) -----

    /** Run roller to intake (pull game piece in). */
    public void runRollerIntake() {
        rollerMotor.set(IntakeConstants.kRollerIntakeSpeed);
    }

    /** Run roller to outtake (push game piece out). */
    public void runRollerOuttake() {
        rollerMotor.set(IntakeConstants.kRollerOuttakeSpeed);
    }

    /** Stop roller. */
    public void stopRoller() {
        rollerMotor.set(0);
    }

    /**
     * Run roller at normalized speed [-1, 1] for motor test (open-loop).
     */
    public void setRollerSpeed(double speed) {
        rollerMotor.set(Math.max(-1, Math.min(1, speed)));
    }

    /**
     * Run pivot at normalized speed [-1, 1] for motor test (open-loop). Overrides position control.
     */
    public void setPivotSpeed(double speed) {
        pivotMotor.set(Math.max(-1, Math.min(1, speed)));
    }

    /** Stop pivot motor. */
    public void stopPivot() {
        pivotMotor.set(0);
    }

    /** Pivot output current in amps (for stall detection). */
    public double getPivotOutputCurrentAmps() {
        return pivotMotor.getOutputCurrent();
    }

    /** Whether pivot is considered at stow (true), collect (false), or unknown (null). */
    public Boolean isPivotStowed() {
        return pivotStowed;
    }

    /** Mark pivot state after hitting mechanical stop (stow = true, collect = false). */
    public void setPivotStowed(Boolean stowed) {
        pivotStowed = stowed;
    }

    // ----- Hopper (feed to shooter) — two SPARK MAXs, same duty -----

    private void setHopperMotors(double duty) {
        hopperMotorA.set(duty);
        hopperMotorB.set(duty);
    }

    /** Run both hopper motors to feed the shooter. */
    public void runHopper() {
        setHopperMotors(IntakeConstants.kHopperFeedSpeed);
    }

    /** Stop both hopper motors. */
    public void stopHopper() {
        setHopperMotors(0);
    }

    /**
     * Run both hopper motors at normalized speed [-1, 1] for motor test (open-loop).
     */
    public void setHopperSpeed(double speed) {
        setHopperMotors(Math.max(-1, Math.min(1, speed)));
    }

    /** Run both hopper motors in reverse (spit out). */
    public void runHopperSpitOut() {
        setHopperMotors(IntakeConstants.kHopperSpitOutSpeed);
    }

    // ----- Composite actions: intake, outtake, stow, collect -----

    /** Intake: pivot to intake position, run roller in, hopper off (or run if feeding). */
    public void intake() {
        // setPivotTowardCollect();
        runRollerIntake();
        stopHopper();
    }

    /** Outtake: pivot to outtake position, run roller out, hopper off. */
    public void outtake() {
        // setPivotOuttake();
        runRollerOuttake();
        stopHopper();
    }

    /** Stow: stop roller and hopper; pivot motion is done by getPivotToStowCommand(). */
    public void stow() {
        stopRoller();
        stopHopper();
    }

    /** Collect: stop roller and hopper; pivot motion is done by getPivotToCollectCommand(). */
    public void collect() {
        stopRoller();
        stopHopper();
    }

    /** Run hopper to feed shooter (e.g. when shooting). Call after intake has piece. */
    public void feedToShooter() {
        runHopper();
    }

    /**
     * Shot feed: hopper pushes to shooter while roller pulls to index the next piece. Do not use for
     * {@link #intake()} — that stops the hopper each cycle.
     */
    public void feedToShooterWithRollerIntake() {
        runHopper();
        runRollerIntake();
    }

    /** Spit out: roller and hopper run in reverse (left bumper). */
    public void spitOut() {
        runRollerOuttake();
        runHopperSpitOut();
    }

    /** Stop roller, hopper, and pivot (e.g. for motor test). */
    public void stopAll() {
        rollerMotor.set(0);
        setHopperMotors(0);
        pivotMotor.set(0);
    }
}
