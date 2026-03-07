// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot command: increment hood angle setpoint by the configured step. For use on SmartDashboard. */
public class IncrementHoodDegreesCommand extends Command {
    private final ShooterSubsystem shooter;

    public IncrementHoodDegreesCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.incrementHoodDegrees();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
