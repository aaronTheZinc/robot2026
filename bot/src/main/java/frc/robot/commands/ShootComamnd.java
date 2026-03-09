package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Run shooter and hopper while held. Shooter ramps up immediately; hopper turns on after 1.0 s.
 * Release stops both shooter and hopper.
 */
public class ShootComamnd extends Command {
    private static final double kHopperDelaySeconds = 1.0;

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final double speed;
    private final Timer timer = new Timer();

    /** Run shooter at default test speed; hopper after 1.0 s. */
    public ShootComamnd(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this(shooter, intake, ShooterConstants.kTestSpeed);
    }

    /** Run shooter at the given normalized speed in [-1, 1]; hopper after 1.0 s. */
    public ShootComamnd(ShooterSubsystem shooter, IntakeSubsystem intake, double speed) {
        this.shooter = shooter;
        this.intake = intake;
        this.speed = Math.max(-1.0, Math.min(1.0, speed));
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.stopHopper();
    }

    @Override
    public void execute() {
        shooter.setShooterVoltage(speed * ShooterConstants.kMaxVoltageVolts);
        if (timer.hasElapsed(kHopperDelaySeconds)) {
            intake.feedToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        intake.stopHopper();
    }
}
