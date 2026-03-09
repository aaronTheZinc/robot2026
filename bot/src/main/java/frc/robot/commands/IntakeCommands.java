package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {
    private final IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command getIntakeCommand() {
        return Commands.run(intake::intake, intake)
                .finallyDo(() -> {
                    intake.stopRoller();
                    intake.stopHopper();
                });
    }

    public Command getFeedToShooterCommand() {
        return Commands.run(intake::feedToShooter, intake)
                .finallyDo(intake::stopHopper);
    }

    public Command getStopHopperCommand() {
        return Commands.runOnce(intake::stopHopper, intake);
    }

    public Command getPivotToStowCommand() {
        int[] stallCount = {0};

        return Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardStow();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles),
                Commands.runOnce(() -> intake.setPivotStowed(true), intake))
                .finallyDo(intake::stopPivot);
    }

    public Command getPivotToDeployCommand() {
        int[] stallCount = {0};

        return Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardDeploy();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles),
                Commands.runOnce(() -> intake.setPivotStowed(false), intake))
                .finallyDo(intake::stopPivot);
    }

    public Command getSpitOutCommand() {
        return Commands.run(intake::spitOut, intake)
                .finallyDo(() -> {
                    intake.stopRoller();
                    intake.stopHopper();
                });
    }
}
