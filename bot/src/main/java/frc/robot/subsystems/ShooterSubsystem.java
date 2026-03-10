// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;
import frc.robot.ShooterConstants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterLeft =
            new TalonFX(ShooterConstants.kShooterLeftId, TunerConstants.kCANBus);
    private final TalonFX shooterRight =
            new TalonFX(ShooterConstants.kShooterRightId, TunerConstants.kCANBus);
    private final TalonFX hoodMotor =
            new TalonFX(ShooterConstants.kHoodMotorId, TunerConstants.kCANBus);

    private final VoltageOut shooterRequest = new VoltageOut(0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    /** True after hood has touched the mechanical stop (real) or sim has "ready" (sim). */
    private boolean shooterReady;

    /** Stored hood angle setpoint in degrees (for dashboard increment/decrement). */
    private double hoodSetpointDeg = ShooterConstants.kHoodMinAngleDeg;
    /** Last hood voltage command sent to the motor controller. */
    private double hoodCommandVolts = 0;

    /** Stored shooter RPM setpoint; 0 means open-loop / stopped. */
    private double shooterRpmSetpoint = 0;

    private final NetworkTable shooterTable =
            NetworkTableInstance.getDefault().getTable("Shooter");

    /** SmartDashboard/Shuffleboard slider entries (robot reads these when dashboard app does not write). */
    private final GenericEntry hoodSliderEntry;
    private final GenericEntry rpmSliderEntry;

    public ShooterSubsystem() {
        // Sliders on SmartDashboard/Shuffleboard for hood and shooter setpoints
        var debugTab = Shuffleboard.getTab("Debug");
        hoodSliderEntry = debugTab
                .add("Hood Setpoint (deg)", ShooterConstants.kHoodMinAngleDeg)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", ShooterConstants.kHoodMinAngleDeg, "max", ShooterConstants.kHoodMaxAngleDeg))
                .getEntry();
        rpmSliderEntry = debugTab
                .add("Shooter RPM Setpoint", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", ShooterConstants.kShooterMinRpm, "max", ShooterConstants.kShooterMaxRpm))
                .getEntry();

        // Invert right shooter so both wheels spin the same direction relative to the robot
        var motorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        shooterRight.getConfigurator().apply(motorOutput);

        var motorOutputLeft = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        shooterLeft.getConfigurator().apply(motorOutputLeft);



        // Hood: current limit and brake when idle. Angle hold is handled in software
        // using a voltage proportional to angle error.
        var hoodCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(ShooterConstants.kHoodStatorCurrentLimitAmps))
                .withStatorCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);
        var hoodOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        hoodMotor.getConfigurator().apply(hoodOutput);

        // Shooter wheels: slot 0 for velocity PID (closed-loop RPM)
        var shooterSlot0 = new Slot0Configs()
                .withKP(ShooterConstants.kShooterKp)
                .withKI(ShooterConstants.kShooterKi)
                .withKD(ShooterConstants.kShooterKd);
        shooterLeft.getConfigurator().apply(shooterSlot0);
        shooterRight.getConfigurator().apply(shooterSlot0);
    }



    /** Set both shooter motors to the same voltage (open-loop). Clears RPM setpoint. */
    public void setShooterVoltage(double volts) {
        shooterRpmSetpoint = 0;
        shooterLeft.setControl(shooterRequest.withOutput(volts));
        shooterRight.setControl(shooterRequest.withOutput(volts));
    }

    /** Set left shooter motor only. Speed in [-1, 1], mapped to voltage. Clears RPM setpoint. */
    public void setShooterLeftSpeed(double speed) {
        shooterRpmSetpoint = 0;
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        shooterLeft.setControl(shooterRequest.withOutput(volts));
    }

    /** Set right shooter motor only. Speed in [-1, 1], mapped to voltage. Clears RPM setpoint. */
    public void setShooterRightSpeed(double speed) {
        shooterRpmSetpoint = 0;
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        shooterRight.setControl(shooterRequest.withOutput(volts));
    }

    /** Stop both shooter motors. */
    public void stopShooter() {
        shooterRpmSetpoint = 0;
        setShooterVoltage(0);
    }

    /**
     * Set shooter velocity setpoint in RPM (closed-loop). Both wheels use the same setpoint.
     * Clamped to [kShooterMinRpm, kShooterMaxRpm]. Applied in periodic().
     */
    public void setShooterRpm(double rpm) {
        shooterRpmSetpoint = Math.max(ShooterConstants.kShooterMinRpm,
                Math.min(ShooterConstants.kShooterMaxRpm, rpm));
    }

    /** Current shooter RPM setpoint (0 when in open-loop). */
    public double getShooterRpmSetpoint() {
        return shooterRpmSetpoint;
    }

    /** Average measured RPM of left and right shooter wheels (closed-loop velocity feedback). */
    public double getShooterRpm() {
        double rpsLeft = shooterLeft.getVelocity().getValueAsDouble();
        double rpsRight = shooterRight.getVelocity().getValueAsDouble();
        return (rpsLeft + rpsRight) * 30.0; // (RPS * 60) / 2 = average RPM
    }

    /** Increment shooter RPM setpoint by {@link ShooterConstants#kShooterRpmIncrement}. */
    public void incrementShooterRpm() {
        setShooterRpm(shooterRpmSetpoint + ShooterConstants.kShooterRpmIncrement);
        rpmSliderEntry.setDouble(shooterRpmSetpoint);
    }

    /** Decrement shooter RPM setpoint by {@link ShooterConstants#kShooterRpmIncrement}. */
    public void decrementShooterRpm() {
        setShooterRpm(shooterRpmSetpoint - ShooterConstants.kShooterRpmIncrement);
        rpmSliderEntry.setDouble(shooterRpmSetpoint);
    }

    /**
     * Set hood setpoint to the given angle in degrees. Subsystem holds that angle using
     * a voltage proportional to angle error. Angle is clamped to the configured range.
     */
    public void setHoodAngle(double degrees) {
        hoodSetpointDeg = Math.max(
                ShooterConstants.kHoodMinAngleDeg,
                Math.min(ShooterConstants.kHoodMaxAngleDeg, degrees));
        if (shooterReady) {
            applyHoodPositionVoltage();
        } else {
            stopHood();
        }
    }

    /** Current hood angle setpoint in degrees. */
    public double getHoodSetpointDegrees() {
        return hoodSetpointDeg;
    }

    /** Increment hood angle setpoint by {@link ShooterConstants#kHoodDegreesIncrement}. */
    public void incrementHoodDegrees() {
        setHoodAngle(hoodSetpointDeg + ShooterConstants.kHoodDegreesIncrement);
        hoodSliderEntry.setDouble(hoodSetpointDeg);
    }

    /** Decrement hood angle setpoint by {@link ShooterConstants#kHoodDegreesIncrement}. */
    public void decrementHoodDegrees() {
        setHoodAngle(hoodSetpointDeg - ShooterConstants.kHoodDegreesIncrement);
        hoodSliderEntry.setDouble(hoodSetpointDeg);
    }

    /** Run hood motor at normalized speed (open-loop). Speed in [-1, 1], mapped to voltage. */
    public void setHoodSpeed(double speed) {
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        hoodCommandVolts = volts;
        hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
    }

    /** Get current hood angle in degrees from integrated encoder. */
    public double getHoodAngleDegrees() {
        double rotations = hoodMotor.getPosition().getValueAsDouble();
        return rotations * ShooterConstants.kHoodDegreesPerMotorRev;
    }

    /** Stop hood motor (voltage 0). */
    public void stopHood() {
        hoodCommandVolts = 0;
        hoodMotor.setControl(hoodVoltageRequest.withOutput(0));
    }

    /** Run hood at a fixed voltage (e.g. for homing). Negative drives toward mechanical stop / hood down. */
    public void setHoodVoltage(double volts) {
        double clamped = Math.max(-ShooterConstants.kMaxVoltageVolts,
                Math.min(ShooterConstants.kMaxVoltageVolts, volts));
        hoodCommandVolts = clamped;
        hoodMotor.setControl(hoodVoltageRequest.withOutput(clamped));
    }

    /** Current through hood motor stator (A). Used for stall detection during homing. */
    public double getHoodStatorCurrentAmps() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getShooterLeftStatorCurrentAmps() {
        return shooterLeft.getStatorCurrent().getValueAsDouble();
    }

    public double getShooterRightStatorCurrentAmps() {
        return shooterRight.getStatorCurrent().getValueAsDouble();
    }

    /** Whether the hood has been homed this enable and shooter is ready to use. */
    public boolean isShooterReady() {
        return shooterReady;
    }

    /** Set by homing command when limit is touched; cleared when disabled. */
    public void setShooterReady(boolean ready) {
        shooterReady = ready;
        if (!ready) {
            stopHood();
        }
    }

    /** Zero the hood position (call when at mechanical stop after homing). */
    public void zeroHoodPosition() {
        hoodMotor.setPosition(0);
        hoodSetpointDeg = 0;
        hoodSliderEntry.setDouble(hoodSetpointDeg);
    }

    private void applyHoodPositionVoltage() {
        if (!shooterReady) {
            stopHood();
            return;
        }

        double errorDeg = getHoodAngleErrorDegrees();

        if (Math.abs(errorDeg) <= ShooterConstants.kHoodAngleToleranceDeg) {
            hoodCommandVolts = 0;
            hoodMotor.setControl(hoodVoltageRequest.withOutput(0));
            return;
        }

        double volts = errorDeg * ShooterConstants.kHoodAngleErrorVoltsPerDeg;
        double minMoveVolts = ShooterConstants.kHoodAngleControlMinVoltageVolts;
        if (Math.abs(volts) < minMoveVolts) {
            volts = Math.copySign(minMoveVolts, errorDeg);
        }
        double clampedVolts = Math.max(
                -ShooterConstants.kHoodAngleControlMaxVoltageVolts,
                Math.min(ShooterConstants.kHoodAngleControlMaxVoltageVolts, volts));
        hoodCommandVolts = clampedVolts;
        hoodMotor.setControl(hoodVoltageRequest.withOutput(clampedVolts));
    }

    private double getHoodAngleErrorDegrees() {
        return hoodSetpointDeg - getHoodAngleDegrees();
    }

    @Override
    public void periodic() {
        // Apply setpoints from slider input: robot-dashboard (NT) takes priority, else SmartDashboard sliders
        double hoodInput = shooterTable.getEntry("hoodSetpointInput").getDouble(Double.NaN);
        double hoodToApply = Double.isNaN(hoodInput) ? hoodSliderEntry.getDouble(hoodSetpointDeg) : hoodInput;
        setHoodAngle(hoodToApply);

        double rpmInput = shooterTable.getEntry("rpmSetpointInput").getDouble(Double.NaN);
        double rpmToApply = Double.isNaN(rpmInput) ? rpmSliderEntry.getDouble(shooterRpmSetpoint) : rpmInput;
        setShooterRpm(rpmToApply);

        if (shooterRpmSetpoint != 0) {
            double rps = shooterRpmSetpoint / 60.0;
            shooterLeft.setControl(shooterVelocityRequest.withVelocity(rps));
            shooterRight.setControl(shooterVelocityRequest.withVelocity(rps));
        }
        SmartDashboard.putBoolean("Shooter Ready", shooterReady);
        SmartDashboard.putNumber("Hood Angle (deg)", getHoodAngleDegrees());
        SmartDashboard.putNumber("Hood Setpoint (deg)", hoodSetpointDeg);
        SmartDashboard.putNumber("Hood Error (deg)", getHoodAngleErrorDegrees());
        SmartDashboard.putNumber("Hood Command Voltage (V)", hoodCommandVolts);
        SmartDashboard.putNumber("Shooter RPM", getShooterRpm());
        SmartDashboard.putNumber("Shooter RPM Setpoint", shooterRpmSetpoint);

        // Debug robot dashboard: /Shooter/* topics
        shooterTable.getEntry("rpm").setDouble(getShooterRpm());
        shooterTable.getEntry("rpmSetpoint").setDouble(shooterRpmSetpoint);
        shooterTable.getEntry("hoodDeg").setDouble(getHoodAngleDegrees());
        shooterTable.getEntry("hoodSetpoint").setDouble(hoodSetpointDeg);
        shooterTable.getEntry("hoodPitchDeg").setDouble(getHoodAngleDegrees());
        shooterTable.getEntry("hoodCurrentAmps").setDouble(getHoodStatorCurrentAmps());
        shooterTable.getEntry("leftCurrentAmps").setDouble(getShooterLeftStatorCurrentAmps());
        shooterTable.getEntry("rightCurrentAmps").setDouble(getShooterRightStatorCurrentAmps());
    }
}
