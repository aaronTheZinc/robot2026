// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Subsystem that reads Limelight pose estimates and fuses them with drivetrain odometry.
 * Runs periodically: sends robot orientation to the Limelight, gets the current pose
 * estimate, computes vision measurement standard deviations from tag count and distance,
 * and adds the measurement to the drivetrain's pose estimator.
 */
public class VisionMeasurement extends SubsystemBase {
    private static final String DEFAULT_LIMELIGHT_NAME = "limelight";

    /** Max rotational rate (rotations/sec) above which vision updates are rejected (blur). */
    private static final double kMaxOmegaRotationsPerSecond = 2.0;

    /** Base standard deviation for x/y (meters) when 1 tag at 0 m. */
    private static final double kBaseStdDevXY = 0.5;
    /** Base standard deviation for theta (radians) when 1 tag at 0 m. */
    private static final double kBaseStdDevTheta = 0.5;
    /** Scale factor for std dev increase with average tag distance (1/m). */
    private static final double kDistanceScalePerMeter = 0.2;
    /** Minimum tag count to apply a vision measurement. */
    private static final int kMinTagCount = 1;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final String m_limelightName;
    private final boolean m_useMegaTag2;
    private final DoubleArrayPublisher m_dashboardPosePublisher = NetworkTableInstance.getDefault()
        .getTable("Pose")
        .getDoubleArrayTopic("robotPose")
        .publish();
    private final DoubleArrayPublisher m_limelightEstimatedPosePublisher = NetworkTableInstance.getDefault()
        .getTable("Pose")
        .getDoubleArrayTopic("limelightEstimatedPose")
        .publish();
    private final BooleanPublisher m_limelightEstimatedPoseValidPublisher = NetworkTableInstance.getDefault()
        .getTable("Pose")
        .getBooleanTopic("limelightEstimatedPoseValid")
        .publish();
    private final double[] m_dashboardPose = new double[3];
    private final double[] m_limelightEstimatedPose = new double[3];

    /**
     * Creates a vision measurement subsystem that fuses the given Limelight with the drivetrain.
     *
     * @param drivetrain    The swerve drivetrain to apply vision measurements to
     * @param limelightName NetworkTables name of the Limelight (e.g. "limelight")
     * @param useMegaTag2    Whether to use MegaTag2 pose estimate (botpose_orb_wpi*)
     */
    public VisionMeasurement(
        CommandSwerveDrivetrain drivetrain,
        String limelightName,
        boolean useMegaTag2
    ) {
        m_drivetrain = drivetrain;
        m_limelightName = limelightName != null && !limelightName.isEmpty() ? limelightName : DEFAULT_LIMELIGHT_NAME;
        m_useMegaTag2 = useMegaTag2;
    }

    /**
     * Creates a vision measurement subsystem with default Limelight name "limelight" and MegaTag2.
     */
    public VisionMeasurement(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, DEFAULT_LIMELIGHT_NAME, true);
    }

    /**
     * Computes vision measurement standard deviations [x, y, theta] from tag count and
     * average tag distance. More tags and closer tags yield tighter (smaller) std devs.
     */
    private Matrix<N3, N1> getVisionStdDevs(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount < kMinTagCount) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        double distFactor = 1.0 + kDistanceScalePerMeter * estimate.avgTagDist;
        double tagFactor = 1.0 / Math.max(1, estimate.tagCount);
        double xy = kBaseStdDevXY * distFactor * tagFactor;
        double theta = kBaseStdDevTheta * distFactor * tagFactor;
        return VecBuilder.fill(xy, xy, theta);
    }

    @Override
    public void periodic() {
        publishLimelightToSmartDashboard();

        // Keep the dashboard pose live even when vision fusion is disabled or rejected.
        publishDashboardPose(m_drivetrain.getState().Pose);

        var driveState = m_drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
            m_limelightName,
            headingDeg, 0, 0,
            0, 0, 0
        );

        PoseEstimate estimate = getPoseEstimateForAlliance();
        if (estimate != null
                && LimelightHelpers.validPoseEstimate(estimate)
                && estimate.tagCount >= kMinTagCount) {
            publishLimelightEstimatedPose(estimate.pose);
        } else {
            publishLimelightEstimatedPoseInvalid();
        }

        // No vision pose fusion in teleop; chassis runs on encoder odometry only and is uninterrupted.
        if (DriverStation.isTeleop()) {
            return;
        }

        if (estimate == null || !LimelightHelpers.validPoseEstimate(estimate)) {
            return;
        }
        if (estimate.tagCount < kMinTagCount) {
            return;
        }
        if (Math.abs(omegaRps) >= kMaxOmegaRotationsPerSecond) {
            return;
        }

        Matrix<N3, N1> stdDevs = getVisionStdDevs(estimate);
        m_drivetrain.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            stdDevs
        );

        publishDashboardPose(m_drivetrain.getState().Pose);
    }

    private static final int kLogThrottlePeriods = 100;

    private int m_periodsWithoutData;

    /** Forwards Limelight NetworkTables values to SmartDashboard under the "Limelight/" prefix. */
    private void publishLimelightToSmartDashboard() {
        String p = "Limelight/";

        double tv = LimelightHelpers.getLimelightNTDouble(m_limelightName, "tv");
        double getpipe = LimelightHelpers.getLimelightNTDouble(m_limelightName, "getpipe");
        String getpipetype = LimelightHelpers.getLimelightNTString(m_limelightName, "getpipetype");

        // Detect if we're getting any data from the Limelight (getpipe is set by camera; default 0)
        boolean hasConnection = getpipetype != null && !getpipetype.isEmpty();
        String status;
        if (!hasConnection) {
            status = "No data (Limelight not connected to this NT server)";
            m_periodsWithoutData++;
            if (Utils.isSimulation() && m_periodsWithoutData == 1) {
                SmartDashboard.putString(p + "help",
                    "In sim use http://localhost:5801 for Limelight config; set NT server to this PC's IP.");
            }
            if (m_periodsWithoutData % kLogThrottlePeriods == 1) {
                DriverStation.reportWarning(
                    "Limelight: No NetworkTables data from \"" + m_limelightName + "\". "
                        + (Utils.isSimulation()
                            ? "In simulation set Limelight NT server to this computer's IP (http://localhost:5801)."
                            : "Check Limelight power and team number / NT server setting."),
                    false);
            }
        } else {
            m_periodsWithoutData = 0;
            status = tv >= 1.0 ? "Target" : "No target";
            SmartDashboard.putString(p + "help", ""); // clear help when connected
        }
        SmartDashboard.putString(p + "status", status);

        SmartDashboard.putNumber(p + "tv", tv);
        SmartDashboard.putNumber(p + "tx", LimelightHelpers.getLimelightNTDouble(m_limelightName, "tx"));
        SmartDashboard.putNumber(p + "ty", LimelightHelpers.getLimelightNTDouble(m_limelightName, "ty"));
        SmartDashboard.putNumber(p + "ta", LimelightHelpers.getLimelightNTDouble(m_limelightName, "ta"));
        SmartDashboard.putNumber(p + "tid", LimelightHelpers.getLimelightNTDouble(m_limelightName, "tid"));
        SmartDashboard.putNumber(p + "tl", LimelightHelpers.getLimelightNTDouble(m_limelightName, "tl"));
        SmartDashboard.putNumber(p + "cl", LimelightHelpers.getLimelightNTDouble(m_limelightName, "cl"));
        SmartDashboard.putNumber(p + "getpipe", getpipe);
        SmartDashboard.putString(p + "getpipetype", getpipetype != null ? getpipetype : "");

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        String poseKey = isRed ? "botpose_wpired" : "botpose_wpiblue";
        double[] botpose = LimelightHelpers.getLimelightNTDoubleArray(m_limelightName, poseKey);
        if (botpose != null && botpose.length >= 6) {
            SmartDashboard.putNumber(p + "pose_x", botpose[0]);
            SmartDashboard.putNumber(p + "pose_y", botpose[1]);
            SmartDashboard.putNumber(p + "pose_z", botpose[2]);
            SmartDashboard.putNumber(p + "pose_roll", botpose[3]);
            SmartDashboard.putNumber(p + "pose_pitch", botpose[4]);
            SmartDashboard.putNumber(p + "pose_yaw", botpose[5]);
            if (botpose.length >= 11) {
                SmartDashboard.putNumber(p + "latency_ms", botpose[6]);
                SmartDashboard.putNumber(p + "tag_count", botpose[7]);
                SmartDashboard.putNumber(p + "tag_span", botpose[8]);
                SmartDashboard.putNumber(p + "tag_dist", botpose[9]);
                SmartDashboard.putNumber(p + "tag_area", botpose[10]);
            }
        }
    }

    private PoseEstimate getPoseEstimateForAlliance() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (m_useMegaTag2) {
            return isRed
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
        } else {
            return isRed
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(m_limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
        }
    }

    private void publishDashboardPose(Pose2d pose) {
        m_dashboardPose[0] = pose.getX();
        m_dashboardPose[1] = pose.getY();
        m_dashboardPose[2] = pose.getRotation().getDegrees();
        m_dashboardPosePublisher.set(m_dashboardPose);
    }

    private void publishLimelightEstimatedPose(Pose2d pose) {
        m_limelightEstimatedPose[0] = pose.getX();
        m_limelightEstimatedPose[1] = pose.getY();
        m_limelightEstimatedPose[2] = pose.getRotation().getDegrees();
        m_limelightEstimatedPosePublisher.set(m_limelightEstimatedPose);
        m_limelightEstimatedPoseValidPublisher.set(true);
    }

    private void publishLimelightEstimatedPoseInvalid() {
        m_limelightEstimatedPoseValidPublisher.set(false);
    }
}
