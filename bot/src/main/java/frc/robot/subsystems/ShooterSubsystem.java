// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);

    /** True after hood has touched the mechanical stop (real) or sim has "ready" (sim). */
    private boolean shooterReady;

    public ShooterSubsystem() {
        // Invert right shooter so both wheels spin the same direction relative to the robot
        var motorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        shooterRight.getConfigurator().apply(motorOutput);

        var motorOutputLeft = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
        shooterLeft.getConfigurator().apply(motorOutputLeft);



        // Hood: PID slot for position control, current limit, brake when idle
        var hoodSlot0 = new Slot0Configs()
                .withKP(ShooterConstants.kHoodKp)
                .withKI(ShooterConstants.kHoodKi)
                .withKD(ShooterConstants.kHoodKd);
        hoodMotor.getConfigurator().apply(hoodSlot0);
        var hoodCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(ShooterConstants.kHoodStatorCurrentLimitAmps))
                .withStatorCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);
        var hoodOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        hoodMotor.getConfigurator().apply(hoodOutput);

        if(ShooterConstants.kShooterLeftTestDefaultInvert) {

        }
    }

    /** Set both shooter motors to the same voltage. */
    public void setShooterVoltage(double volts) {
        shooterLeft.setControl(shooterRequest.withOutput(volts));
        shooterRight.setControl(shooterRequest.withOutput(volts));
    }

    /** Set left shooter motor only. Speed in [-1, 1], mapped to voltage. */
    public void setShooterLeftSpeed(double speed) {
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        shooterLeft.setControl(shooterRequest.withOutput(volts));
    }

    /** Set right shooter motor only. Speed in [-1, 1], mapped to voltage. */
    public void setShooterRightSpeed(double speed) {
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        shooterRight.setControl(shooterRequest.withOutput(volts));
    }

    /** Stop both shooter motors. */
    public void stopShooter() {
        setShooterVoltage(0);
    }

    /**
     * Set hood setpoint to the given angle in degrees. Subsystem holds that angle using
     * position closed-loop. Angle is clamped to [kHoodMinAngleDeg, kHoodMaxAngleDeg].
     */
    public void setHoodAngle(double degrees) {
        double clamped =
                Math.max(
                        ShooterConstants.kHoodMinAngleDeg,
                        Math.min(ShooterConstants.kHoodMaxAngleDeg, degrees));
        double rotations = clamped / ShooterConstants.kHoodDegreesPerMotorRev;
        hoodMotor.setControl(hoodPositionRequest.withPosition(rotations));
    }

    /** Run hood motor at normalized speed (open-loop). Speed in [-1, 1], mapped to voltage. */
    public void setHoodSpeed(double speed) {
        double volts = Math.max(-1, Math.min(1, speed)) * ShooterConstants.kMaxVoltageVolts;
        hoodMotor.setControl(shooterRequest.withOutput(volts));
    }

    /** Get current hood angle in degrees from integrated encoder. */
    public double getHoodAngleDegrees() {
        double rotations = hoodMotor.getPosition().getValueAsDouble();
        return rotations * ShooterConstants.kHoodDegreesPerMotorRev;
    }

    /** Stop hood motor (voltage 0). */
    public void stopHood() {
        hoodMotor.setControl(shooterRequest.withOutput(0));
    }

    /** Run hood at a fixed voltage (e.g. for homing). Use negative to run toward mechanical stop. */
    public void setHoodVoltage(double volts) {
        double clamped = Math.max(-ShooterConstants.kMaxVoltageVolts,
                Math.min(ShooterConstants.kMaxVoltageVolts, volts));
        hoodMotor.setControl(shooterRequest.withOutput(clamped));
    }

    /** Current through hood motor stator (A). Used for stall detection during homing. */
    public double getHoodStatorCurrentAmps() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    /** Whether the hood has been homed this enable and shooter is ready to use. */
    public boolean isShooterReady() {
        return shooterReady;
    }

    /** Set by homing command when limit is touched; cleared when disabled. */
    public void setShooterReady(boolean ready) {
        shooterReady = ready;
    }

    /** Zero the hood position (call when at mechanical stop after homing). */
    public void zeroHoodPosition() {
        hoodMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Ready", shooterReady);
        SmartDashboard.putNumber("Hood Angle (deg)", getHoodAngleDegrees());
    }
}
