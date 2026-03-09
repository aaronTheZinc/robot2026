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

    public Command getIncrementShooterRpmCommand() {
        return Commands.runOnce(shooter::incrementShooterRpm, shooter);
    }

    public Command getDecrementShooterRpmCommand() {
        return Commands.runOnce(shooter::decrementShooterRpm, shooter);
    }

    public Command getRunShooterCommand() {
        return getRunShooterCommand(ShooterConstants.kTestSpeed);
    }

    public Command getRunShooterCommand(double speed) {
        double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed));
        double shooterVolts = clampedSpeed * ShooterConstants.kMaxVoltageVolts;

        return Commands.run(() -> shooter.setShooterVoltage(shooterVolts), shooter)
                .finallyDo(shooter::stopShooter);
    }

    public Command getReverseShooterCommand() {
        return getRunShooterCommand(-ShooterConstants.kTestSpeed);
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
