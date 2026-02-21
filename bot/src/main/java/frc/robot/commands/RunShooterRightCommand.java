// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Test command: run the right shooter motor at test speed while held. */
public class RunShooterRightCommand extends Command {
    private final ShooterSubsystem shooter;

    public RunShooterRightCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterRightSpeed(ShooterConstants.kTestSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRightSpeed(0);
    }
}
