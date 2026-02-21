// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Test command: run the hood motor at a normalized speed while held (open-loop). */
public class RunHoodCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double speed;

    /** Run hood at default test speed (positive = one direction). */
    public RunHoodCommand(ShooterSubsystem shooter) {
        this(shooter, ShooterConstants.kTestSpeed);
    }

    /** Run hood at the given speed in [-1, 1] (negative = opposite direction). */
    public RunHoodCommand(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodSpeed(0);
    }
}
