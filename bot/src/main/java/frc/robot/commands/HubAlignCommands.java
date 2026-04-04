package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.DriveConstants;
import frc.robot.HubAlignCalibration;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * In-place rotation toward the hub using the same heading law as driver right bumper (shot-map hub heading
 * plus {@link HubAlignCalibration} distance scaling).
 */
public final class HubAlignCommands {
    private static final double kAlignTimeoutSeconds = 4.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final HubAlignCalibration hubAlignCalibration;

    private final SwerveRequest.FieldCentric hubAlignRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public HubAlignCommands(CommandSwerveDrivetrain drivetrain, HubAlignCalibration hubAlignCalibration) {
        this.drivetrain = drivetrain;
        this.hubAlignCalibration = hubAlignCalibration;
    }

    /** Zero translation; rotate until within hub-align tolerance or timeout. */
    public Command getRotateToHubInPlaceCommand() {
        return getRotateToHubInPlaceCommand(0.0);
    }

    /**
     * Same as {@link #getRotateToHubInPlaceCommand()} but adds {@code extraFieldHeadingOffsetDeg} to the bearing
     * toward the hub (after calibration). Use e.g. {@code -28} for depo shot angle only.
     */
    public Command getRotateToHubInPlaceCommand(double extraFieldHeadingOffsetDeg) {
        return drivetrain
                .applyRequest(
                        () -> {
                            var pose = drivetrain.getState().Pose;
                            double offsetDeg =
                                    hubAlignCalibration.getScaledOffsetDeg(pose)
                                            + extraFieldHeadingOffsetDeg;
                            return hubAlignRequest
                                    .withVelocityX(0)
                                    .withVelocityY(0)
                                    .withRotationalRate(
                                            DriveConstants.teleopOmegaTowardHub(pose, offsetDeg));
                        })
                .until(() -> isAlignedToHub(extraFieldHeadingOffsetDeg))
                .withTimeout(kAlignTimeoutSeconds)
                .withName("Rotate to hub");
    }

    private boolean isAlignedToHub(double extraFieldHeadingOffsetDeg) {
        var pose = drivetrain.getState().Pose;
        Rotation2d target =
                DriveConstants.rotationToFaceHubFromShotMap(pose)
                        .plus(
                                Rotation2d.fromDegrees(
                                        hubAlignCalibration.getScaledOffsetDeg(pose)
                                                + extraFieldHeadingOffsetDeg));
        double errRad =
                MathUtil.angleModulus(
                        target.getRadians() - pose.getRotation().getRadians());
        return Math.abs(errRad) <= Math.toRadians(DriveConstants.kTeleopHubAlignToleranceDeg);
    }
}
