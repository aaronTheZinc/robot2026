// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0)
            .withSlot(0)
            .withEnableFOC(false)
            // Persistent Talon/Tuner soft limits often default to 0—would freeze position PID.
            .withIgnoreSoftwareLimits(true)
            .withIgnoreHardwareLimits(true);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    /** True after hood has touched the mechanical stop (real) or sim has "ready" (sim). */
    private boolean shooterReady;

    /** Stored hood angle setpoint in degrees (for dashboard increment/decrement). */
    private double hoodSetpointDeg = ShooterConstants.kHoodMinAngleDeg;
    /** Low-pass filtered hood target from slider/NT (when adjustable). */
    private double hoodDashboardSetpointFilteredDeg = ShooterConstants.kHoodMinAngleDeg;

    /**
     * Target shooter RPM (KNN / POV / dashboard). Flywheels only run in {@link #periodic()} while
     * {@link #shootFlywheelVelocityEnabled} is true (shot or explicit spin test).
     */
    private double shooterRpmSetpoint = 0;
    /** True when slider/NetworkTables setpoint inputs should drive hood/RPM setpoints. */
    private boolean dashboardSetpointControlEnabled = true;
    /**
     * When true, closed-loop flywheel velocity tracks {@link #shooterRpmSetpoint}. False while driving so KNN
     * can hold a nonzero RPM target without spinning wheels until a shot sequence enables this flag.
     */
    private boolean shootFlywheelVelocityEnabled;

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



        // Hood: current limit, brake when idle, peak V for PositionVoltage PID, slot 0 gains.
        // Invert must stay consistent with kHoodHomingVoltageVolts (toward mechanical stop).
        var hoodCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(ShooterConstants.kHoodStatorCurrentLimitAmps))
                .withStatorCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);
        var hoodPeakVolts = new VoltageConfigs()
                .withPeakForwardVoltage(ShooterConstants.kHoodAngleControlMaxVoltageVolts)
                .withPeakReverseVoltage(-ShooterConstants.kHoodAngleControlMaxVoltageVolts);
        hoodMotor.getConfigurator().apply(hoodPeakVolts);
        var hoodSlot0 = new Slot0Configs()
                .withKP(ShooterConstants.kHoodKp)
                .withKI(ShooterConstants.kHoodKi)
                .withKD(ShooterConstants.kHoodKd);
        hoodMotor.getConfigurator().apply(hoodSlot0);
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

    /**
     * Stops flywheel output without clearing {@link #shooterRpmSetpoint} so manual / B-shot RPM stays armed.
     */
    public void stopShooter() {
        shooterLeft.setControl(shooterVelocityRequest.withVelocity(0));
        shooterRight.setControl(shooterVelocityRequest.withVelocity(0));
    }

    /**
     * Set shooter velocity setpoint in RPM (closed-loop). Clamped to [kShooterMinRpm, kShooterMaxRpm].
     * Flywheels only track this in {@link #periodic()} while a command requires this subsystem.
     */
    public void setShooterRpm(double rpm) {
        shooterRpmSetpoint = Math.max(ShooterConstants.kShooterMinRpm,
                Math.min(ShooterConstants.kShooterMaxRpm, rpm));
        rpmSliderEntry.setDouble(shooterRpmSetpoint);
    }

    /** Current shooter RPM setpoint (0 when in open-loop). */
    public double getShooterRpmSetpoint() {
        return shooterRpmSetpoint;
    }

    /** Enable/disable dashboard-setpoint ownership of hood and shooter RPM setpoints. */
    public void setDashboardSetpointControlEnabled(boolean enabled) {
        dashboardSetpointControlEnabled = enabled;
    }

    /** When true, flywheels run closed-loop to {@link #shooterRpmSetpoint}; false between shots while KNN still updates the setpoint. */
    public void setShootFlywheelVelocityEnabled(boolean enabled) {
        shootFlywheelVelocityEnabled = enabled;
    }

    public boolean isShootFlywheelVelocityEnabled() {
        return shootFlywheelVelocityEnabled;
    }

    /**
     * Hood to minimum angle and align dashboard/filter/NT hood inputs (e.g. emergency stow / legacy paths).
     */
    public void stowHoodAndSyncDashboardAfterProfile() {
        hoodDashboardSetpointFilteredDeg = ShooterConstants.kHoodMinAngleDeg;
        hoodSliderEntry.setDouble(ShooterConstants.kHoodMinAngleDeg);
        shooterTable.getEntry("hoodSetpointInput").setDouble(ShooterConstants.kHoodMinAngleDeg);
        setHoodAngle(ShooterConstants.kHoodMinAngleDeg);
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
    }

    /** Decrement shooter RPM setpoint by {@link ShooterConstants#kShooterRpmIncrement}. */
    public void decrementShooterRpm() {
        setShooterRpm(shooterRpmSetpoint - ShooterConstants.kShooterRpmIncrement);
    }

    /**
     * Set hood setpoint in degrees. When {@link #isShooterReady()}, {@link #periodic()} applies
     * closed-loop {@link PositionVoltage} toward this angle.
     */
    public void setHoodAngle(double degrees) {
        hoodSetpointDeg = Math.max(
                ShooterConstants.kHoodMinAngleDeg,
                Math.min(ShooterConstants.kHoodMaxAngleDeg, degrees));
        if (!shooterReady) {
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
        hoodDashboardSetpointFilteredDeg = hoodSetpointDeg;
        shooterTable.getEntry("hoodSetpointInput").setDouble(hoodSetpointDeg);
    }

    /** Decrement hood angle setpoint by {@link ShooterConstants#kHoodDegreesIncrement}. */
    public void decrementHoodDegrees() {
        setHoodAngle(hoodSetpointDeg - ShooterConstants.kHoodDegreesIncrement);
        hoodSliderEntry.setDouble(hoodSetpointDeg);
        hoodDashboardSetpointFilteredDeg = hoodSetpointDeg;
        shooterTable.getEntry("hoodSetpointInput").setDouble(hoodSetpointDeg);
    }

    /** Run hood motor at normalized speed (open-loop). Speed in [-1, 1], mapped to voltage. */
    public void setHoodSpeed(double speed) {
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
    }

    /** Get current hood angle in degrees from integrated encoder. */
    public double getHoodAngleDegrees() {
        double rotations = hoodMotor.getPosition().getValueAsDouble();
        return rotations * ShooterConstants.kHoodDegreesPerMotorRev;
    }

    /** Stop hood motor (voltage 0). */
    public void stopHood() {
        hoodMotor.setControl(hoodVoltageRequest.withOutput(0));
    }

    /** Run hood at a fixed voltage (e.g. for homing). Negative drives toward mechanical stop / hood down. */
    public void setHoodVoltage(double volts) {
        double clamped = Math.max(-ShooterConstants.kMaxVoltageVolts,
                Math.min(ShooterConstants.kMaxVoltageVolts, volts));
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
        hoodDashboardSetpointFilteredDeg = 0;
        hoodSliderEntry.setDouble(hoodSetpointDeg);
    }

    /**
     * Applies hood {@link PositionVoltage} toward {@link #hoodSetpointDeg}. Call once per
     * {@code Robot.robotPeriodic} <em>after</em> {@code applyKnnHoodInterpolation()} so KNN updates
     * reach the Talon the same cycle.
     */
    public void applyHoodMotorClosedLoopTick() {
        if (!shooterReady) {
            return;
        }

        double errorDeg = getHoodAngleErrorDegrees();
        if (Math.abs(errorDeg) <= ShooterConstants.kHoodAngleToleranceDeg) {
            hoodMotor.setControl(hoodVoltageRequest.withOutput(0));
            return;
        }

        double setpointRot = hoodSetpointDeg / ShooterConstants.kHoodDegreesPerMotorRev;
        hoodMotor.setControl(hoodPositionRequest.withPosition(setpointRot));
    }

    private double getHoodAngleErrorDegrees() {
        return hoodSetpointDeg - getHoodAngleDegrees();
    }

    @Override
    public void periodic() {
        // Do not overwrite hood/RPM from sliders/NT while a command requires this subsystem (e.g. shot profile).
        // Otherwise a profile's finallyDo can re-enable dashboard for one cycle and fight closed-loop hood/RPM.
        var cs = CommandScheduler.getInstance();
        boolean noShooterCommand = cs.requiring(this) == null;

        if (dashboardSetpointControlEnabled && noShooterCommand) {
            // Hood/RPM from robot-dashboard (NT) or Shuffleboard when no KNN map / interpolation off.
            double hoodInput = shooterTable.getEntry("hoodSetpointInput").getDouble(Double.NaN);
            double hoodToApply =
                    Double.isNaN(hoodInput) ? hoodSliderEntry.getDouble(hoodSetpointDeg) : hoodInput;
            double delta = hoodToApply - hoodDashboardSetpointFilteredDeg;
            if (Math.abs(delta) > ShooterConstants.kHoodDashboardSetpointSnapThresholdDeg) {
                hoodDashboardSetpointFilteredDeg = hoodToApply;
            } else {
                hoodDashboardSetpointFilteredDeg +=
                        ShooterConstants.kHoodDashboardSetpointSmoothingAlpha * delta;
            }
            setHoodAngle(hoodDashboardSetpointFilteredDeg);

            double rpmInput = shooterTable.getEntry("rpmSetpointInput").getDouble(Double.NaN);
            double rpmToApply = Double.isNaN(rpmInput) ? rpmSliderEntry.getDouble(shooterRpmSetpoint) : rpmInput;
            setShooterRpm(rpmToApply);
        }

        if (shootFlywheelVelocityEnabled && shooterRpmSetpoint != 0) {
            double rps = shooterRpmSetpoint / 60.0;
            shooterLeft.setControl(shooterVelocityRequest.withVelocity(rps));
            shooterRight.setControl(shooterVelocityRequest.withVelocity(rps));
        }

        SmartDashboard.putBoolean("Shooter Ready", shooterReady);
        SmartDashboard.putNumber("Hood Angle (deg)", getHoodAngleDegrees());
        SmartDashboard.putNumber("Hood Setpoint (deg)", hoodSetpointDeg);
        SmartDashboard.putNumber("Hood Error (deg)", getHoodAngleErrorDegrees());
        SmartDashboard.putNumber(
                "Hood Command Voltage (V)", hoodMotor.getMotorVoltage().getValueAsDouble());
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
