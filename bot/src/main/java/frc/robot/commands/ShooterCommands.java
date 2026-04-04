package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private final ShooterSubsystem shooter;

    public ShooterCommands(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    public Command getIncrementHoodDegreesCommand() {
        return Commands.runOnce(shooter::incrementHoodDegrees, shooter);
    }

    public Command getDecrementHoodDegreesCommand() {
        return Commands.runOnce(shooter::decrementHoodDegrees, shooter);
    }

    /** Subsystem controller POV up: step hood up every {@link ShooterConstants#kHoodPovRepeatDelaySeconds} while held. */
    public Command getAdjustHoodUpWhileHeldCommand() {
        return Commands.repeatingSequence(
                Commands.runOnce(shooter::incrementHoodDegrees, shooter),
                Commands.waitSeconds(ShooterConstants.kHoodPovRepeatDelaySeconds));
    }

    /** Subsystem controller POV down: step hood down every {@link ShooterConstants#kHoodPovRepeatDelaySeconds} while held. */
    public Command getAdjustHoodDownWhileHeldCommand() {
        return Commands.repeatingSequence(
                Commands.runOnce(shooter::decrementHoodDegrees, shooter),
                Commands.waitSeconds(ShooterConstants.kHoodPovRepeatDelaySeconds));
    }

    public Command getIncrementShooterRpmCommand() {
        return Commands.runOnce(shooter::incrementShooterRpm, shooter);
    }

    public Command getDecrementShooterRpmCommand() {
        return Commands.runOnce(shooter::decrementShooterRpm, shooter);
    }

    public Command getRunShooterCommand() {
        return Commands.run(
                () -> shooter.setShooterRpm(shooter.getShooterRpmSetpoint()),
                shooter)
                .finallyDo(shooter::stopShooter);
    }

    /**
     * Hold hood + RPM with no {@code finallyDo} cleanup — use when composing ramp + feed sequences so cleanup
     * runs once at the end (deadline cancelling an inner command must not stow between phases).
     */
    public Command getRunShotProfileHoldCommand(double hoodAngleDeg, double rpm) {
        return Commands.run(
                        () -> {
                            shooter.setDashboardSetpointControlEnabled(false);
                            shooter.setHoodAngle(hoodAngleDeg);
                            shooter.setShooterRpm(rpm);
                        },
                        shooter)
                .beforeStarting(shooter::clearKnnHoodManualTuneBlock);
    }

    /** Runs hood + closed-loop shooter at the given profile until interrupted; stows on end. */
    public Command getRunShotProfileCommand(double hoodAngleDeg, double rpm) {
        return getRunShotProfileHoldCommand(hoodAngleDeg, rpm)
                .finallyDo(
                        () -> {
                            shooter.stopShooter();
                            shooter.stowHoodAndSyncDashboardAfterProfile();
                            shooter.setDashboardSetpointControlEnabled(true);
                        });
    }

    public Command getRunWingShotCommand() {
        return getRunShotProfileCommand(
                ShooterConstants.kWingShotHoodAngleDeg,
                ShooterConstants.kWingShotRpm);
    }

    public Command getRunWingShotHoldCommand() {
        return getRunShotProfileHoldCommand(
                ShooterConstants.kWingShotHoodAngleDeg,
                ShooterConstants.kWingShotRpm);
    }

    /**
     * Teleop B ramp/feed segments: manual hood + RPM only — does not clear the KNN hood block (inferred hood must not
     * win during scheduler gaps). Sequence {@code finallyDo} clears after the shot.
     */
    public Command getRunWingShotWithManualHoldCommand() {
        return Commands.run(
                () -> {
                    shooter.setDashboardSetpointControlEnabled(false);
                    shooter.setHoodAngle(shooter.getHoodSetpointDegrees());
                    shooter.setShooterRpm(shooter.getShooterRpmSetpoint());
                },
                shooter);
    }

    /**
     * Standalone teleop B profile with stow on interrupt/end. For sequences use {@link #getRunWingShotWithManualHoldCommand()}.
     */
    public Command getRunWingShotWithManualRpmCommand() {
        return getRunWingShotWithManualHoldCommand()
                .finallyDo(
                        () -> {
                            shooter.stopShooter();
                            shooter.stowHoodAndSyncDashboardAfterProfile();
                            shooter.setDashboardSetpointControlEnabled(true);
                            shooter.clearKnnHoodManualTuneBlock();
                        });
    }

    public Command getRunCenterShotCommand() {
        return getRunShotProfileCommand(
                ShooterConstants.kLeftBumperShotHoodAngleDeg,
                ShooterConstants.kLeftBumperShotRpm);
    }

    public Command getRunCenterShotHoldCommand() {
        return getRunShotProfileHoldCommand(
                ShooterConstants.kLeftBumperShotHoodAngleDeg,
                ShooterConstants.kLeftBumperShotRpm);
    }

    public Command getRunShooterCommandTest() {
        return getRunShooterVoltageCommand(ShooterConstants.kTestSpeed);
    }

    private Command getRunShooterVoltageCommand(double speed) {
        double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed));
        double shooterVolts = clampedSpeed * ShooterConstants.kMaxVoltageVolts;

        return Commands.run(() -> shooter.setShooterVoltage(shooterVolts), shooter)
                .finallyDo(shooter::stopShooter);
    }

    public Command getReverseShooterCommand() {
        return getRunShooterVoltageCommand(-ShooterConstants.kTestSpeed);
    }

    public Command getHoodHomingCommand() {
        int[] stallCount = {0};

        return Commands.sequence(
                Commands.runOnce(() -> stallCount[0] = 0, shooter),
                Commands.run(() -> {
                    shooter.setHoodVoltage(ShooterConstants.kHoodHomingVoltageVolts);
                    double current = Math.abs(shooter.getHoodStatorCurrentAmps());
                    stallCount[0] = current >= ShooterConstants.kHoodStallCurrentAmps
                            ? stallCount[0] + 1
                            : 0;
                }, shooter).until(() -> stallCount[0] >= ShooterConstants.kHoodStallConfirmCycles),
                Commands.runOnce(() -> {
                    shooter.zeroHoodPosition();
                    shooter.setShooterReady(true);
                }, shooter))
                .finallyDo(shooter::stopHood);
    }
}
