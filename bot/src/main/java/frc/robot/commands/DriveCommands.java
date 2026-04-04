package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Swerve drive helpers (robot-relative moves, etc.). */
public class DriveCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final double backSpeedMps;
    private final SwerveRequest.RobotCentric backTranslation =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * @param drivetrain drivetrain subsystem
     * @param backTranslationSpeedMps positive magnitude for reverse travel (m/s)
     */
    public DriveCommands(CommandSwerveDrivetrain drivetrain, double backTranslationSpeedMps) {
        this.drivetrain = drivetrain;
        this.backSpeedMps = backTranslationSpeedMps;
    }

    /**
     * Robot-relative pure translation backward (toward the rear of the robot), no rotation.
     * Typically bound {@code whileTrue}.
     */
    public Command getBackTranslationCommand() {
        return drivetrain.applyRequest(
                () -> backTranslation
                        .withVelocityX(-backSpeedMps)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    /** Same motion for a fixed duration (useful for PathPlanner named commands). */
    public Command getBackTranslationForSeconds(double seconds) {
        return Commands.deadline(Commands.waitSeconds(seconds), getBackTranslationCommand());
    }

    /**
     * Robot-relative reverse translation until odometry shows at least {@code meters} straight-line travel from
     * the pose when this command starts (matches a straight PathPlanner chord when heading is unchanged).
     */
    public Command getBackTranslationForMeters(double meters) {
        final Pose2d[] start = new Pose2d[1];
        return Commands.sequence(
                drivetrain.runOnce(() -> start[0] = drivetrain.getState().Pose),
                getBackTranslationCommand()
                        .until(
                                () ->
                                        drivetrain
                                                .getState()
                                                .Pose
                                                .getTranslation()
                                                .getDistance(start[0].getTranslation())
                                                >= meters));
    }
}
