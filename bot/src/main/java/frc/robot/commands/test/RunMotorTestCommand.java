// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorTestSubsystem;

/**
 * Writes MotorTest/Enable, MotorTest/Motor, MotorTest/Speed to NetworkTables every execute()
 * so MotorTestSubsystem.periodic() drives the selected motor. Clears Enable on end.
 * Used by controller bindings (with axis as speed supplier) and SmartDashboard (with fixed speed).
 */
public class RunMotorTestCommand extends Command {
    private final String motorId;
    private final DoubleSupplier speedSupplier;
    private final MotorTestSubsystem motorTest;

    public RunMotorTestCommand(String motorId, DoubleSupplier speedSupplier, MotorTestSubsystem motorTest) {
        this.motorId = motorId;
        this.speedSupplier = speedSupplier;
        this.motorTest = motorTest;
        addRequirements(motorTest);
    }

    /** Command that runs the given motor at a fixed speed (e.g. for SmartDashboard). */
    public static RunMotorTestCommand atFixedSpeed(String motorId, double speed, MotorTestSubsystem motorTest) {
        return new RunMotorTestCommand(motorId, () -> speed, motorTest);
    }

    @Override
    public void execute() {
        var table = NetworkTableInstance.getDefault().getTable("MotorTest");
        table.getEntry("Motor").setString(motorId);
        table.getEntry("Speed").setDouble(speedSupplier.getAsDouble());
        table.getEntry("Enable").setBoolean(true);
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableInstance.getDefault().getTable("MotorTest").getEntry("Enable").setBoolean(false);
        motorTest.stopAll();
    }
}
