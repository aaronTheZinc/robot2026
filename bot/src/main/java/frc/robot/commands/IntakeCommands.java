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
        return Commands.run(intake::intake, intake).finallyDo(intake::stow);
    }

    public Command getFeedToShooterCommand() {
        return Commands.run(intake::feedToShooter, intake)
                .finallyDo(intake::stopHopper);
    }

    public Command getStopHopperCommand() {
        return Commands.runOnce(intake::stopHopper, intake);
    }

    /**
     * Run intake roller inward until interrupted (PathPlanner event end); stops roller on end.
     */
    public Command getRollerIntakeHoldCommand() {
        return Commands.run(intake::runRollerIntake, intake).finallyDo(intake::stopRoller);
    }

    public Command getPivotTowardStowManualCommand() {
        return Commands.run(intake::setPivotTowardStow, intake)
                .finallyDo(intake::stopPivot);
    }

    public Command getPivotTowardCollectManualCommand() {
        return Commands.run(intake::setPivotTowardCollect, intake)
                .finallyDo(intake::stopPivot);
    }

    /**
     * Home pivot slowly toward stow until mechanical stop (current stall), then zero encoder and
     * mark ready. Used on enable and when operator holds Y+B. Has safety timeout.
     */
    public Command getPivotHomingCommand() {
        int[] stallCount = {0};

        Command runToStall = Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardStowHoming();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles),
                Commands.runOnce(() -> {
                    intake.zeroPivotPosition();
                    intake.setPivotReady(true);
                    intake.setPivotStowed(true);
                }, intake));

        return Commands.race(
                runToStall,
                Commands.waitSeconds(IntakeConstants.kPivotStallTimeoutSeconds))
                .finallyDo(() -> intake.stopPivot());
    }

    /**
     * Home pivot toward collect (down) until mechanical stop (stall current), then zero encoder and mark ready.
     * Same pattern as {@link frc.robot.commands.ShooterCommands#getHoodHomingCommand()}: relative position is 0 at
     * the down stop; {@link frc.robot.subsystems.IntakeSubsystem#isPivotStowed()} is false. Has safety timeout.
     */
    public Command getPivotDownHomingCommand() {
        int[] stallCount = {0};

        Command runToStall = Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardCollectHoming();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles),
                Commands.runOnce(() -> {
                    intake.zeroPivotPosition();
                    intake.setPivotReady(true);
                    intake.setPivotStowed(false);
                }, intake));

        return Commands.race(
                runToStall,
                Commands.waitSeconds(IntakeConstants.kPivotStallTimeoutSeconds))
                .finallyDo(intake::stopPivot);
    }

    /**
     * Run pivot toward stow until mechanical stop. No-op if not homed yet. Has safety timeout.
     */
    public Command getPivotToStowCommand() {
        if (!intake.isPivotReady()) {
            return Commands.none();
        }
        int[] stallCount = {0};

        Command runToStall = Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardStow();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles),
                Commands.runOnce(() -> intake.setPivotStowed(true), intake));

        return Commands.race(
                runToStall,
                Commands.waitSeconds(IntakeConstants.kPivotStallTimeoutSeconds))
                .finallyDo(intake::stopPivot);
    }

    /**
     * Run pivot toward collect until {@link IntakeSubsystem#isPivotAtDownPosition()} is true. No-op if not homed
     * yet. Has safety timeout if the encoder never reaches the down band.
     */
    public Command getPivotToCollectCommand() {
        if (!intake.isPivotReady()) {
            return Commands.none();
        }

        Command runToDown = Commands.sequence(
                Commands.run(intake::setPivotTowardCollect, intake).until(intake::isPivotAtDownPosition),
                Commands.runOnce(intake::stopPivot, intake),
                Commands.runOnce(() -> intake.setPivotStowed(false), intake));

        return Commands.race(
                runToDown,
                Commands.waitSeconds(IntakeConstants.kPivotStallTimeoutSeconds))
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
