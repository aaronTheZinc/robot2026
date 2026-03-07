package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Run both shooter motors together at the same speed while held. */
public class ShootComamnd extends Command {
    private final ShooterSubsystem shooter;
    private final double speed;

    /** Run both shooter motors at the default test speed. */
    public ShootComamnd(ShooterSubsystem shooter) {
        this(shooter, ShooterConstants.kTestSpeed);
    }

    /** Run both shooter motors together at the given normalized speed in [-1, 1]. */
    public ShootComamnd(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = Math.max(-1.0, Math.min(1.0, speed));
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterVoltage(speed * ShooterConstants.kMaxVoltageVolts);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
