package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {
    private final IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    /**
     * Collect homing: drive down, then require sustained high current (after an ignore window) before stall
     * counts toward the stop.
     */
    private void updateCollectHomingStall(int[] stallCount, int[] phaseCycles) {
        intake.setPivotTowardCollectHoming();
        double current = Math.abs(intake.getPivotOutputCurrentAmps());
        phaseCycles[0]++;
        if (phaseCycles[0] < IntakeConstants.kPivotCollectHomingIgnoreStallCycles) {
            stallCount[0] = 0;
            return;
        }
        stallCount[0] = current >= IntakeConstants.kPivotCollectHomingStallCurrentAmps
                ? stallCount[0] + 1
                : 0;
    }

    private void runPivotSeekCollectWithStallEscape(int[] seekStallCount) {
        intake.runPivotSeekCollectSetpoint();
        double r = intake.getPivotRotations();
        double target = intake.getPivotDownRotationsLearned();
        double tol = IntakeConstants.kPivotDownPositionToleranceRotations;
        if (r < target - tol
                && Math.abs(intake.getPivotOutputCurrentAmps())
                        >= IntakeConstants.kPivotCollectSeekStallStopAmps) {
            seekStallCount[0]++;
        } else {
            seekStallCount[0] = 0;
        }
    }

    private boolean isCollectSeekFinished(int[] seekStallCount) {
        return intake.isPivotAtDownPosition()
                || intake.isPivotDownLimitPressed()
                || seekStallCount[0] >= IntakeConstants.kPivotCollectSeekStallConfirmCycles;
    }

    public Command getIntakeCommand() {
        return Commands.run(intake::intake, intake).finallyDo(intake::stow);
    }

    public Command getFeedToShooterCommand() {
        return Commands.run(intake::feedToShooter, intake)
                .finallyDo(intake::stopHopper);
    }

    /**
     * Same timing as {@link #getFeedToShooterCommand()} for shot sequences: hopper feed plus roller inward
     * (indexing). Stops roller and hopper when the command ends; pivot unchanged.
     */
    public Command getFeedToShooterWithRollerCommand() {
        return Commands.run(intake::feedToShooterWithRollerIntake, intake)
                .finallyDo(intake::stow);
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
     * Run the pivot open-loop toward collect (down) for {@link IntakeConstants#kPivotTowardCollectPulseSeconds},
     * then stop the pivot.
     */
    public Command getPivotTowardCollectPulseCommand() {
        return Commands.sequence(
                        Commands.run(intake::setPivotTowardCollect, intake)
                                .withTimeout(IntakeConstants.kPivotTowardCollectPulseSeconds),
                        Commands.runOnce(intake::stopPivot, intake))
                .finallyDo(intake::stopPivot)
                .withName("IntakePivotTowardCollectPulse");
    }

    /**
     * Full pivot homing: (1) mechanical stow — stall or up limit, zero encoder; (2) seek stow encoder setpoint;
     * (3) mechanical collect — high sustained current + ignore window ({@link IntakeConstants}); (4) seek collect;
     * then {@link IntakeSubsystem#setPivotReady(boolean) true}. Later phases run only if earlier ones succeed.
     */
    public Command getPivotHomingCommand() {
        boolean[] stowMechanicalOk = {false};
        boolean[] stowSeekOk = {false};
        boolean[] downMechanicalOk = {false};
        int[] stallCount = {0};
        int[] collectHomingPhaseCycles = {0};

        Command towardStowUntilStop = Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, intake),
                Commands.run(() -> {
                    intake.setPivotTowardStowHoming();
                    double current = Math.abs(intake.getPivotOutputCurrentAmps());
                    stallCount[0] = current >= IntakeConstants.kPivotStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, intake).until(() -> stallCount[0] >= IntakeConstants.kPivotStallConfirmCycles
                        || intake.isPivotUpLimitPressed()),
                Commands.runOnce(() -> {
                    intake.zeroPivotPosition();
                    intake.setPivotStowed(true);
                    stowMechanicalOk[0] = true;
                }, intake));

        Command stowMechanicalRace = Commands.race(
                towardStowUntilStop,
                Commands.waitSeconds(IntakeConstants.kPivotStallTimeoutSeconds));

        Command seekStowSetpoint = Commands.sequence(
                Commands.run(intake::runPivotSeekStowSetpoint, intake).until(intake::isPivotAtStowPosition),
                Commands.runOnce(() -> stowSeekOk[0] = true, intake));

        Command seekStowRace = Commands.race(
                seekStowSetpoint,
                Commands.waitSeconds(IntakeConstants.kPivotSetpointSeekTimeoutSeconds));

        Command towardDownUntilStop = Commands.sequence(
                Commands.runOnce(
                        () -> {
                            stallCount[0] = 0;
                            collectHomingPhaseCycles[0] = 0;
                            intake.applyPivotCurrentLimitForCollectHoming();
                        },
                        intake),
                Commands.run(() -> updateCollectHomingStall(stallCount, collectHomingPhaseCycles), intake)
                        .until(
                                () -> stallCount[0] >= IntakeConstants.kPivotCollectHomingStallConfirmCycles
                                        || intake.isPivotDownLimitPressed()),
                Commands.runOnce(
                        () -> {
                            intake.setPivotDownRotationsLearnedFromHoming();
                            intake.setPivotStowed(false);
                            intake.markCollectMechanicalHomingSucceeded();
                            intake.applyPivotDefaultCurrentLimit();
                            downMechanicalOk[0] = true;
                        },
                        intake));

        Command downMechanicalRace = Commands.race(
                towardDownUntilStop,
                Commands.waitSeconds(IntakeConstants.kPivotCollectHomingStallTimeoutSeconds));

        int[] collectSeekStall = {0};
        Command seekCollectSetpoint = Commands.sequence(
                Commands.runOnce(() -> collectSeekStall[0] = 0, intake),
                Commands.run(() -> runPivotSeekCollectWithStallEscape(collectSeekStall), intake)
                        .until(() -> isCollectSeekFinished(collectSeekStall)),
                Commands.runOnce(() -> intake.setPivotReady(true), intake));

        Command seekCollectRace = Commands.race(
                seekCollectSetpoint,
                Commands.waitSeconds(IntakeConstants.kPivotSetpointSeekTimeoutSeconds));

        Command afterStowMechanical = Commands.sequence(
                seekStowRace,
                new ConditionalCommand(
                        Commands.sequence(
                                downMechanicalRace,
                                new ConditionalCommand(
                                        seekCollectRace,
                                        Commands.none(),
                                        () -> downMechanicalOk[0])),
                        Commands.none(),
                        () -> stowSeekOk[0]));

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    stowMechanicalOk[0] = false;
                                    stowSeekOk[0] = false;
                                    downMechanicalOk[0] = false;
                                },
                                intake),
                        stowMechanicalRace,
                        new ConditionalCommand(afterStowMechanical, Commands.none(), () -> stowMechanicalOk[0]))
                .finallyDo(
                        () -> {
                            intake.applyPivotDefaultCurrentLimit();
                            intake.stopPivot();
                        });
    }

    /**
     * From stow: mechanical collect (stall), learn encoder, seek collect setpoint, then ready. For PathPlanner
     * when the arm is already stowed.
     */
    public Command getPivotDownHomingCommand() {
        boolean[] downMechanicalOk = {false};
        int[] stallCount = {0};
        int[] collectHomingPhaseCycles = {0};

        Command towardDownUntilStop = Commands.sequence(
                Commands.runOnce(
                        () -> {
                            stallCount[0] = 0;
                            collectHomingPhaseCycles[0] = 0;
                            intake.applyPivotCurrentLimitForCollectHoming();
                        },
                        intake),
                Commands.run(() -> updateCollectHomingStall(stallCount, collectHomingPhaseCycles), intake)
                        .until(
                                () -> stallCount[0] >= IntakeConstants.kPivotCollectHomingStallConfirmCycles
                                        || intake.isPivotDownLimitPressed()),
                Commands.runOnce(
                        () -> {
                            intake.setPivotDownRotationsLearnedFromHoming();
                            intake.setPivotStowed(false);
                            intake.markCollectMechanicalHomingSucceeded();
                            intake.applyPivotDefaultCurrentLimit();
                            downMechanicalOk[0] = true;
                        },
                        intake));

        Command downMechanicalRace = Commands.race(
                towardDownUntilStop,
                Commands.waitSeconds(IntakeConstants.kPivotCollectHomingStallTimeoutSeconds));

        int[] collectSeekStall = {0};
        Command seekCollectSetpoint = Commands.sequence(
                Commands.runOnce(() -> collectSeekStall[0] = 0, intake),
                Commands.run(() -> runPivotSeekCollectWithStallEscape(collectSeekStall), intake)
                        .until(() -> isCollectSeekFinished(collectSeekStall)),
                Commands.runOnce(() -> intake.setPivotReady(true), intake));

        Command seekCollectRace = Commands.race(
                seekCollectSetpoint,
                Commands.waitSeconds(IntakeConstants.kPivotSetpointSeekTimeoutSeconds));

        return Commands.sequence(
                        downMechanicalRace,
                        new ConditionalCommand(seekCollectRace, Commands.none(), () -> downMechanicalOk[0]))
                .finallyDo(
                        () -> {
                            intake.applyPivotDefaultCurrentLimit();
                            intake.stopPivot();
                        });
    }

    /**
     * Move to stow encoder setpoint (0) after homing — no stall; uses learned range from homing only.
     */
    public Command getPivotToStowCommand() {
        if (!intake.isPivotReady()) {
            return Commands.none();
        }

        Command seek =
                Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    intake.applyPivotPidSnapCurrentLimit();
                                    intake.resetPivotPositionPid();
                                },
                                intake),
                        Commands.run(() -> intake.runPivotPidToSetpoint(0.0), intake)
                                .until(() -> intake.isPivotPidAtSetpoint(0.0)),
                        Commands.runOnce(() -> intake.setPivotStowed(true), intake));

        return Commands.race(
                        seek,
                        Commands.waitSeconds(IntakeConstants.kPivotSetpointSeekTimeoutSeconds))
                .finallyDo(
                        () -> {
                            intake.applyPivotDefaultCurrentLimit();
                            intake.resetPivotPositionPid();
                            intake.stopPivot();
                        });
    }

    /**
     * Move to collect encoder setpoint ({@link frc.robot.subsystems.IntakeSubsystem#getPivotDownRotationsLearned()})
     * after homing — no stall; range was stored when stall hit the down stop during homing.
     */
    public Command getPivotToCollectCommand() {
        if (!intake.isPivotReady()) {
            return Commands.none();
        }

        Command seek =
                Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    intake.applyPivotPidSnapCurrentLimit();
                                    intake.resetPivotPositionPid();
                                },
                                intake),
                        Commands.run(
                                        () ->
                                                intake.runPivotPidToSetpoint(
                                                        intake.getPivotDownRotationsLearned()),
                                        intake)
                                .until(
                                        () ->
                                                intake.isPivotPidAtSetpoint(
                                                        intake.getPivotDownRotationsLearned())),
                        Commands.runOnce(() -> intake.setPivotStowed(false), intake));

        return Commands.race(
                        seek,
                        Commands.waitSeconds(IntakeConstants.kPivotSetpointSeekTimeoutSeconds))
                .finallyDo(
                        () -> {
                            intake.applyPivotDefaultCurrentLimit();
                            intake.resetPivotPositionPid();
                            intake.stopPivot();
                        });
    }

    public Command getSpitOutCommand() {
        return Commands.run(intake::spitOut, intake)
                .finallyDo(() -> {
                    intake.stopRoller();
                    intake.stopHopper();
                });
    }

}
