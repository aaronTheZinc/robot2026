// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Run pivot downward until stator current indicates mechanical stop (deploy), then stop and mark state.
 */
public class PivotToDeployCommand extends Command {
    private final IntakeSubsystem intake;
    private int stallCount;

    public PivotToDeployCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        stallCount = 0;
        intake.setPivotTowardDeploy();
    }

    @Override
    public void execute() {
        double current = Math.abs(intake.getPivotOutputCurrentAmps());
        if (current >= IntakeConstants.kPivotStallCurrentAmps) {
            stallCount++;
        } else {
            stallCount = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
        if (!interrupted && stallCount >= IntakeConstants.kPivotStallConfirmCycles) {
            intake.setPivotStowed(false);
        }
    }

    @Override
    public boolean isFinished() {
        return stallCount >= IntakeConstants.kPivotStallConfirmCycles;
    }
}
