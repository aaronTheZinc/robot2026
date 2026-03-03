// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * Reads MotorTest/Enable, MotorTest/Motor, MotorTest/Speed from NetworkTables and
 * applies open-loop output to the selected motor for individual testing.
 * Runs when disabled or in Test mode. Uses MotorTestConstants for min/max and applies invert from NT.
 */
public class MotorTestSubsystem extends SubsystemBase {

    private final VoltageOut voltageOut = new VoltageOut(0);

    private final NetworkTable motorTestTable;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final CommandSwerveDrivetrain drivetrain;

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
        if (!DriverStation.isDisabled() && !Robot.inTestMode()) {
            stopAll();
            return;
        }

        boolean enable = motorTestTable.getEntry("Enable").getBoolean(false);
        String motor = motorTestTable.getEntry("Motor").getString("");
        double speed = motorTestTable.getEntry("Speed").getDouble(MotorTestConstants.kDefaultSpeed);
        double minSpeed = MotorTestConstants.getMinSpeed(motor);
        double maxSpeed = MotorTestConstants.getMaxSpeed(motor);
        double clampedSpeed = Math.max(minSpeed, Math.min(maxSpeed, Math.max(-1, Math.min(1, speed))));

        boolean invert = motorTestTable.getEntry("Invert_" + motor).getBoolean(MotorTestConstants.getDefaultInvert(motor));
        if (invert) {
            clampedSpeed = -clampedSpeed;
        }

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
            case "hopper" -> {
                stopAll();
                intake.setHopperSpeed(clampedSpeed);
            }
            case "drive0", "drive1", "drive2", "drive3" -> {
                stopAll();
                int idx = motor.charAt(5) - '0';
                double volts = clampedSpeed * MotorTestConstants.getSwerveTestMaxVolts();
                drivetrain.getModule(idx).getDriveMotor().setControl(voltageOut.withOutput(volts));
            }
            case "steer0", "steer1", "steer2", "steer3" -> {
                stopAll();
                int idx = motor.charAt(5) - '0';
                double volts = clampedSpeed * MotorTestConstants.getSwerveTestMaxVolts();
                drivetrain.getModule(idx).getSteerMotor().setControl(voltageOut.withOutput(volts));
            }
            default -> stopAll();
        }
    }
}
