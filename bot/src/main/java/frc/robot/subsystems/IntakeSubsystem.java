// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IntakeConstants;

/**
 * Intake with 3 SPARK MAXs (REV 2026 Spark API):
 * - Pivot: up/down (position control for stow, collect, intake, outtake).
 * - Roller: in/out (duty cycle for intaking/outtaking).
 * - Hopper: feeds game pieces to the shooter.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor =
            new SparkMax(IntakeConstants.kPivotId, MotorType.kBrushless);
    private final SparkMax rollerMotor =
            new SparkMax(IntakeConstants.kRollerId, MotorType.kBrushless);
    private final SparkMax hopperMotor =
            new SparkMax(IntakeConstants.kHopperId, MotorType.kBrushless);

    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    public IntakeSubsystem() {
        var pivotConfig = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .apply(new ClosedLoopConfig()
                        .pid(IntakeConstants.kPivotKp, IntakeConstants.kPivotKi, IntakeConstants.kPivotKd)
                        .outputRange(-1.0, 1.0));
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var rollerConfig = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var hopperConfig = new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PivotRotations", getPivotRotations());
    }

    // ----- Pivot (up/down) positions -----

    /** Set pivot to stow position (retracted). */
    public void setPivotStow() {
        pivotMotor.getClosedLoopController().setSetpoint(
                IntakeConstants.kPivotStowRotations, ControlType.kPosition);
    }

    /** Set pivot to collect position (deployed to pick from floor). */
    public void setPivotCollect() {
        pivotMotor.getClosedLoopController().setSetpoint(
                IntakeConstants.kPivotCollectRotations, ControlType.kPosition);
    }

    /** Set pivot to intake position. */
    public void setPivotIntake() {
        pivotMotor.getClosedLoopController().setSetpoint(
                IntakeConstants.kPivotIntakeRotations, ControlType.kPosition);
    }

    /** Set pivot to outtake position. */
    public void setPivotOuttake() {
        pivotMotor.getClosedLoopController().setSetpoint(
                IntakeConstants.kPivotOuttakeRotations, ControlType.kPosition);
    }

    /** Get current pivot position in motor rotations. */
    public double getPivotRotations() {
        return pivotEncoder.getPosition();
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

    // ----- Hopper (feed to shooter) -----

    /** Run hopper to feed the shooter. */
    public void runHopper() {
        hopperMotor.set(IntakeConstants.kHopperFeedSpeed);
    }

    /** Stop hopper. */
    public void stopHopper() {
        hopperMotor.set(0);
    }

    /**
     * Run hopper at normalized speed [-1, 1] for motor test (open-loop).
     */
    public void setHopperSpeed(double speed) {
        hopperMotor.set(Math.max(1, Math.min(-1, speed)));
    }

    // ----- Composite actions: intake, outtake, stow, collect -----

    /** Intake: pivot to intake position, run roller in, hopper off (or run if feeding). */
    public void intake() {
        setPivotIntake();
        runRollerIntake();
        stopHopper();
    }

    /** Outtake: pivot to outtake position, run roller out, hopper off. */
    public void outtake() {
        setPivotOuttake();
        runRollerOuttake();
        stopHopper();
    }

    /** Stow: pivot up, stop roller and hopper. */
    public void stow() {
        setPivotStow();
        stopRoller();
        stopHopper();
    }

    /** Collect: pivot to collect position, roller and hopper stopped (ready to run intake). */
    public void collect() {
        setPivotCollect();
        stopRoller();
        stopHopper();
    }

    /** Run hopper to feed shooter (e.g. when shooting). Call after intake has piece. */
    public void feedToShooter() {
        runHopper();
    }

    /** Stop roller, hopper, and pivot (e.g. for motor test). */
    public void stopAll() {
        rollerMotor.set(0);
        hopperMotor.set(0);
        pivotMotor.set(0);
    }
}
