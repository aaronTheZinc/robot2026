// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads MotorTest/Enable, MotorTest/Motor, MotorTest/Speed from NetworkTables and
 * applies open-loop output to the selected motor for individual testing. Only runs when disabled.
 */
public class MotorTestSubsystem extends SubsystemBase {

    private static final double kDefaultSpeed = 0.15;
    /** Max voltage for swerve drive/steer during test (super slow). */
    private static final double kSwerveTestMaxVolts = 3.0;

    private final NetworkTable motorTestTable;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final CommandSwerveDrivetrain drivetrain;

    private final VoltageOut voltageOut = new VoltageOut(0);

    public MotorTestSubsystem(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.intake = intake;
        this.drivetrain = drivetrain;
        motorTestTable = NetworkTableInstance.getDefault().getTable("MotorTest");
    }

    /** Stop all subsystems' motors used by motor test. */
    public void stopAll() {
        shooter.stopShooter();
        shooter.stopHood();
        intake.stopAll();
        for (int i = 0; i < 4; i++) {
            var mod = drivetrain.getModule(i);
            mod.getDriveMotor().setControl(voltageOut.withOutput(0));
            mod.getSteerMotor().setControl(voltageOut.withOutput(0));
        }
    }

    @Override
    public void periodic() {
        if (!DriverStation.isDisabled()) {
            stopAll();
            return;
        }

        boolean enable = motorTestTable.getEntry("Enable").getBoolean(false);
        String motor = motorTestTable.getEntry("Motor").getString("");
        double speed = motorTestTable.getEntry("Speed").getDouble(kDefaultSpeed);
        double clampedSpeed = Math.max(-1, Math.min(1, speed));

        if (!enable || motor.isEmpty()) {
            stopAll();
            return;
        }

        switch (motor) {
            case "shooterLeft" -> {
                stopAll();
                shooter.setShooterLeftSpeed(clampedSpeed);
            }
            case "shooterRight" -> {
                stopAll();
                shooter.setShooterRightSpeed(clampedSpeed);
            }
            case "hood" -> {
                stopAll();
                shooter.setHoodSpeed(clampedSpeed);
            }
            case "intakeRoller" -> {
                stopAll();
                intake.setRollerSpeed(clampedSpeed);
            }
            case "intakePivot" -> {
                stopAll();
                intake.setPivotSpeed(clampedSpeed);
            }
            case "drive0", "drive1", "drive2", "drive3" -> {
                stopAll();
                int idx = motor.charAt(5) - '0';
                double volts = clampedSpeed * kSwerveTestMaxVolts;
                drivetrain.getModule(idx).getDriveMotor().setControl(voltageOut.withOutput(volts));
            }
            case "steer0", "steer1", "steer2", "steer3" -> {
                stopAll();
                int idx = motor.charAt(5) - '0';
                double volts = clampedSpeed * kSwerveTestMaxVolts;
                drivetrain.getModule(idx).getSteerMotor().setControl(voltageOut.withOutput(volts));
            }
            default -> stopAll();
        }
    }
}
