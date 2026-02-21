// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * On the real robot: run the hood motor backward until stator current spikes (mechanical stop),
 * then zero the position and set shooter ready. Do not run in simulation.
 */
public class HoodHomingCommand extends Command {
    private final ShooterSubsystem shooter;
    private int stallCount;

    public HoodHomingCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        stallCount = 0;
    }

    @Override
    public void execute() {
        shooter.setHoodVoltage(ShooterConstants.kHoodHomingVoltageVolts);
        double current = Math.abs(shooter.getHoodStatorCurrentAmps());
        if (current >= ShooterConstants.kHoodStallCurrentAmps) {
            stallCount++;
        } else {
            stallCount = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopHood();
        if (!interrupted && stallCount >= ShooterConstants.kHoodStallConfirmCycles) {
            shooter.zeroHoodPosition();
            shooter.setShooterReady(true);
        }
    }

    @Override
    public boolean isFinished() {
        return stallCount >= ShooterConstants.kHoodStallConfirmCycles;
    }
}
