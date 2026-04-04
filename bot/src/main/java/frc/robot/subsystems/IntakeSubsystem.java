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

    /** True after pivot has homed to stow stop this enable (mirrors hood shooterReady). */
    private boolean pivotReady;

    /** After stall-based travel: true = at stow (up), false = at collect (down), null = unknown. */
    private Boolean pivotStowed = null;

    /** Last FPGA time we published to SmartDashboard; negative so the first loop always publishes. */
    private double lastDashboardPublishSec = -1.0;

    public IntakeSubsystem() {
        var pivotConfig = new SparkMaxConfig()
                .smartCurrentLimit(IntakeConstants.kPivotSmartCurrentLimitAmps)
                .idleMode(SparkBaseConfig.IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        double now = Timer.getFPGATimestamp();
        if (now - lastDashboardPublishSec < IntakeConstants.kIntakeDashboardPublishPeriodSeconds) {
            return;
        }
        lastDashboardPublishSec = now;

        double pivotRelRot = getPivotRotations();
        SmartDashboard.putNumber("Intake/Pivot Relative Position (rot)", pivotRelRot);
        SmartDashboard.putNumber("Intake/Pivot Down Threshold (rot)", IntakeConstants.kPivotDownPositionRotations);
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

    // ----- Pivot (up/down) — mechanical stops, no encoder setpoints -----

    /** Whether the pivot has been homed this enable (stow stop found and encoder zeroed). */
    public boolean isPivotReady() {
        return pivotReady;
    }

    /** Set by homing command when stow stop is reached; cleared when disabled. */
    public void setPivotReady(boolean ready) {
        pivotReady = ready;
        if (!ready) {
            stopPivot();
        }
    }

    /** Zero the pivot encoder (call when at a mechanical stop after the matching homing command). */
    public void zeroPivotPosition() {
        pivotEncoder.setPosition(0);
    }

    /** Run pivot slowly toward stow for homing (init / Y+B). */
    public void setPivotTowardStowHoming() {
        pivotMotor.set(IntakeConstants.kPivotHomingSpeed);
    }

    /** Run pivot slowly toward collect (down) for homing until stall (encoder zero = deployed). */
    public void setPivotTowardCollectHoming() {
        pivotMotor.set(IntakeConstants.kPivotCollectHomingSpeed);
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
     * True when relative pivot position is within ±{@link IntakeConstants#kPivotDownPositionToleranceRotations}
     * of {@link IntakeConstants#kPivotDownPositionRotations}.
     */
    public boolean isPivotAtDownPosition() {
        return getPivotRotations() > IntakeConstants.kPivotDownPositionRotations;
        
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
