// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Test command: run the left shooter motor at test speed while held. */
public class RunShooterLeftCommand extends Command {
    private final ShooterSubsystem shooter;

    public RunShooterLeftCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterLeftSpeed(ShooterConstants.kTestSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterLeftSpeed(0);
    }
}
