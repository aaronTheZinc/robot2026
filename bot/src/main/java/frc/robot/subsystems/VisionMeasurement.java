// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Optional;

import frc.robot.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.BotPoseNetworkTableSample;
import frc.robot.VisionConstants;

/**
 * Reads one or two Limelight {@code botpose*} NT arrays, fuses into the Phoenix pose estimator when the
 * robot is enabled. Publishes {@code Pose/robotPose} after fusion so dashboards match {@code getState().Pose}.
 * <p>
 * Fused {@link Pose2d} struct on NetworkTables: {@code /SmartDashboard/Odometry/FusedPose}.
 * <p>
 * Debug: {@code Pose/idealShooterPose} — {@code [x, y]} copied from fused odometry; {@code headingDeg} is
 * the shot-map hub-facing heading (same model as the robot dashboard / {@code hubField.ts}).
 */
public class VisionMeasurement extends SubsystemBase {
    private static final String[] kCommonSecondaryLimelightAliases = {"limelight2", "limelight-front"};

    private final CommandSwerveDrivetrain m_drivetrain;
    private final String m_primaryLimelightName;
    private final String m_secondaryLimelightName;
    private final String[] m_secondaryLimelightCandidates;
    private final boolean m_useMegaTag2;

    private final DoubleArrayPublisher m_dashboardPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("robotPose")
            .publish();
    /** Same format as {@code robotPose}: [x, y, headingDeg] — hub-facing heading at current XY (see class javadoc). */
    private final DoubleArrayPublisher m_idealShooterPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("idealShooterPose")
            .publish();
    private final StringPublisher m_poseFieldTypePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getStringTopic(".type")
            .publish();

    private final DoubleArrayPublisher m_limelightEstimatedPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("limelightEstimatedPose")
            .publish();
    private final BooleanPublisher m_limelightEstimatedPoseValidPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelightEstimatedPoseValid")
            .publish();
    private final BooleanPublisher m_limelightHasLockPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelightHasLock")
            .publish();

    private final DoubleArrayPublisher m_limelight2EstimatedPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("limelight2EstimatedPose")
            .publish();
    private final BooleanPublisher m_limelight2EstimatedPoseValidPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelight2EstimatedPoseValid")
            .publish();
    private final DoubleArrayPublisher m_limelightFrontEstimatedPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("limelightFrontEstimatedPose")
            .publish();
    private final BooleanPublisher m_limelightFrontEstimatedPoseValidPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelightFrontEstimatedPoseValid")
            .publish();
    private final BooleanPublisher m_limelightFrontHasLockPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelightFrontHasLock")
            .publish();

    private final double[] m_dashboardPose = new double[3];
    private final double[] m_idealShooterPose = new double[3];
    private final double[] m_limelightEstimatedPose = new double[3];
    private final double[] m_limelight2EstimatedPose = new double[3];

    /** Cached each {@link #periodic()} for lightweight readers (e.g. CSV session log). */
    private boolean m_primaryTvLock;
    private boolean m_secondaryTvLock;
    private String m_resolvedSecondaryLimelightName;
    private double m_primaryTxDeg;
    private double m_primaryTyDeg;
    private double m_secondaryTxDeg;
    private double m_secondaryTyDeg;

    /** Limelight MegaTag field pose from NT (WPIBlue or WPIRed per alliance), cached for logging. */
    private boolean m_primaryReportedPoseValid;
    private double m_primaryReportedPoseXMeters;
    private double m_primaryReportedPoseYMeters;
    private double m_primaryReportedPoseDeg;

    private boolean m_secondaryReportedPoseValid;
    private double m_secondaryReportedPoseXMeters;
    private double m_secondaryReportedPoseYMeters;
    private double m_secondaryReportedPoseDeg;

    /** Connection throttle counters for {@link #publishLimelightToSmartDashboard}. */
    private final int[] m_periodsWithoutDataPrimary = new int[1];
    private final int[] m_periodsWithoutDataSecondary = new int[1];

    private final StructPublisher<Pose2d> m_fusedPose2dSmartDashboard = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("Odometry")
            .getStructTopic("FusedPose", Pose2d.struct)
            .publish();

    /**
     * @param drivetrain Drivetrain pose for NT and vision fusion into the Phoenix pose estimator.
     * @param primaryName NetworkTables name of the primary Limelight (e.g. {@code limelight})
     * @param secondaryName Second Limelight table name, or null/empty to disable
     * @param useMegaTag2 Whether to use MegaTag2 pose estimate ({@code botpose_orb_wpi*})
     */
    public VisionMeasurement(
            CommandSwerveDrivetrain drivetrain,
            String primaryName,
            String secondaryName,
            boolean useMegaTag2) {
        m_drivetrain = drivetrain;
        m_primaryLimelightName =
                primaryName != null && !primaryName.isBlank() ? primaryName : VisionConstants.kPrimaryLimelightName;
        m_secondaryLimelightName = secondaryName != null && !secondaryName.isBlank() ? secondaryName : null;
        m_secondaryLimelightCandidates =
                buildSecondaryLimelightCandidates(m_primaryLimelightName, m_secondaryLimelightName);
        m_useMegaTag2 = useMegaTag2;
    }

    /** Primary + secondary names from {@link VisionConstants}; MegaTag mode from {@link VisionConstants#kUseMegaTag2}. */
    public VisionMeasurement(CommandSwerveDrivetrain drivetrain) {
        this(
                drivetrain,
                VisionConstants.kPrimaryLimelightName,
                VisionConstants.kSecondaryLimelightName,
                VisionConstants.kUseMegaTag2);
    }

    /** Primary Limelight {@code tv} ≥ 1 or secondary (if configured): pipeline has a valid target. */
    public boolean limelightHasTagLock() {
        return m_primaryTvLock || (m_secondaryLimelightName != null && m_secondaryTvLock);
    }

    /** {@code tv} ≥ 1 on the primary Limelight (updated each vision periodic). */
    public boolean primaryLimelightHasTagLock() {
        return m_primaryTvLock;
    }

    /** {@code tv} ≥ 1 on the secondary Limelight, or false if no secondary camera. */
    public boolean secondaryLimelightHasTagLock() {
        return m_secondaryLimelightName != null && m_secondaryTvLock;
    }

    public double getPrimaryTxDeg() {
        return m_primaryTxDeg;
    }

    public double getPrimaryTyDeg() {
        return m_primaryTyDeg;
    }

    public double getSecondaryTxDeg() {
        return m_secondaryLimelightName != null ? m_secondaryTxDeg : 0.0;
    }

    public double getSecondaryTyDeg() {
        return m_secondaryLimelightName != null ? m_secondaryTyDeg : 0.0;
    }

    /** Primary Limelight reported pose (valid when {@code tv} and botpose sample present). */
    public boolean primaryReportedPoseValid() {
        return m_primaryReportedPoseValid;
    }

    public double getPrimaryReportedPoseXMeters() {
        return m_primaryReportedPoseXMeters;
    }

    public double getPrimaryReportedPoseYMeters() {
        return m_primaryReportedPoseYMeters;
    }

    public double getPrimaryReportedPoseDeg() {
        return m_primaryReportedPoseDeg;
    }

    /** Secondary Limelight reported pose; invalid if no secondary camera or no target. */
    public boolean secondaryReportedPoseValid() {
        return m_secondaryLimelightName != null && m_secondaryReportedPoseValid;
    }

    public double getSecondaryReportedPoseXMeters() {
        return m_secondaryReportedPoseXMeters;
    }

    public double getSecondaryReportedPoseYMeters() {
        return m_secondaryReportedPoseYMeters;
    }

    public double getSecondaryReportedPoseDeg() {
        return m_secondaryReportedPoseDeg;
    }

    @Override
    public void periodic() {
        var driveState = m_drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        m_resolvedSecondaryLimelightName = resolveSecondaryLimelightName();

        LimelightHelpers.SetRobotOrientation(m_primaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        for (String candidate : m_secondaryLimelightCandidates) {
            LimelightHelpers.SetRobotOrientation(candidate, headingDeg, 0, 0, 0, 0, 0);
        }

        Optional<BotPoseNetworkTableSample> ntPosePrimary =
                LimelightHelpers.getBotPoseNetworkTableSample(m_primaryLimelightName, botPoseNtEntryName());
        publishLimelightToSmartDashboard(
                "Limelight/", m_primaryLimelightName, ntPosePrimary, m_periodsWithoutDataPrimary);

        double tvPrimary = LimelightHelpers.getLimelightNTDouble(m_primaryLimelightName, "tv");
        m_primaryTvLock = tvPrimary >= 1.0;
        m_limelightHasLockPublisher.set(m_primaryTvLock);
        m_primaryTxDeg = LimelightHelpers.getLimelightNTDouble(m_primaryLimelightName, "tx");
        m_primaryTyDeg = LimelightHelpers.getLimelightNTDouble(m_primaryLimelightName, "ty");
        boolean validPrimary = tvPrimary >= 1.0 && ntPosePrimary.isPresent();
        if (validPrimary) {
            var reported = ntPosePrimary.get().pose;
            m_primaryReportedPoseValid = true;
            m_primaryReportedPoseXMeters = reported.getX();
            m_primaryReportedPoseYMeters = reported.getY();
            m_primaryReportedPoseDeg = reported.getRotation().getDegrees();
            publishLimelightEstimatedPose(
                    m_limelightEstimatedPose,
                    m_limelightEstimatedPosePublisher,
                    m_limelightEstimatedPoseValidPublisher,
                    reported);
            if (DriverStation.isEnabled()) {
                fuseBotPoseFromNetworkTables(ntPosePrimary.get());
            }
        } else {
            m_primaryReportedPoseValid = false;
            m_primaryReportedPoseXMeters = 0.0;
            m_primaryReportedPoseYMeters = 0.0;
            m_primaryReportedPoseDeg = 0.0;
            m_limelightEstimatedPoseValidPublisher.set(false);
        }

        SmartDashboard.putString(
                "Limelight2/configured_name", m_secondaryLimelightName != null ? m_secondaryLimelightName : "");
        SmartDashboard.putString(
                "Limelight2/resolved_name",
                m_resolvedSecondaryLimelightName != null ? m_resolvedSecondaryLimelightName : "");

        if (m_secondaryLimelightName != null) {
            String secondaryReadName = getSecondaryReadTableName();
            String secondaryName = secondaryReadName != null ? secondaryReadName : m_secondaryLimelightName;
            Optional<BotPoseNetworkTableSample> ntPoseSecondary =
                    secondaryReadName != null
                            ? LimelightHelpers.getBotPoseNetworkTableSample(
                                    secondaryReadName, botPoseNtEntryName())
                            : Optional.empty();
            publishLimelightToSmartDashboard(
                    "Limelight2/", secondaryName, ntPoseSecondary, m_periodsWithoutDataSecondary);

            m_secondaryTvLock = anySecondaryCandidateHasTagLock();
            m_limelightFrontHasLockPublisher.set(m_secondaryTvLock);
            double tvSecondary =
                    secondaryReadName != null
                            ? LimelightHelpers.getLimelightNTDouble(secondaryReadName, "tv")
                            : 0.0;
            m_secondaryTxDeg =
                    secondaryReadName != null
                            ? LimelightHelpers.getLimelightNTDouble(secondaryReadName, "tx")
                            : 0.0;
            m_secondaryTyDeg =
                    secondaryReadName != null
                            ? LimelightHelpers.getLimelightNTDouble(secondaryReadName, "ty")
                            : 0.0;
            boolean validSecondary = tvSecondary >= 1.0 && ntPoseSecondary.isPresent();
            if (validSecondary) {
                var reported2 = ntPoseSecondary.get().pose;
                m_secondaryReportedPoseValid = true;
                m_secondaryReportedPoseXMeters = reported2.getX();
                m_secondaryReportedPoseYMeters = reported2.getY();
                m_secondaryReportedPoseDeg = reported2.getRotation().getDegrees();
                publishLimelightEstimatedPose(
                        m_limelight2EstimatedPose,
                        m_limelight2EstimatedPosePublisher,
                        m_limelight2EstimatedPoseValidPublisher,
                        reported2);
                publishLimelightEstimatedPose(
                        m_limelight2EstimatedPose,
                        m_limelightFrontEstimatedPosePublisher,
                        m_limelightFrontEstimatedPoseValidPublisher,
                        reported2);
                if (DriverStation.isEnabled()) {
                    fuseBotPoseFromNetworkTables(ntPoseSecondary.get());
                }
            } else {
                m_secondaryReportedPoseValid = false;
                m_secondaryReportedPoseXMeters = 0.0;
                m_secondaryReportedPoseYMeters = 0.0;
                m_secondaryReportedPoseDeg = 0.0;
                m_limelight2EstimatedPoseValidPublisher.set(false);
                m_limelightFrontEstimatedPoseValidPublisher.set(false);
            }
        } else {
            m_secondaryTvLock = false;
            m_limelightFrontHasLockPublisher.set(false);
            m_secondaryTxDeg = 0.0;
            m_secondaryTyDeg = 0.0;
            m_secondaryReportedPoseValid = false;
            m_secondaryReportedPoseXMeters = 0.0;
            m_secondaryReportedPoseYMeters = 0.0;
            m_secondaryReportedPoseDeg = 0.0;
            m_limelight2EstimatedPoseValidPublisher.set(false);
            m_limelightFrontEstimatedPoseValidPublisher.set(false);
        }

        publishFusedRobotPoseToDashboard(m_drivetrain.getState().Pose);
    }

    /**
     * Always WPIBlue: PathPlanner / {@code getState().Pose} use the blue-origin WPILib field frame.
     * {@code *_wpired} is a different origin; alliance path flip does not convert fused pose — fuse WPIBlue only.
     */
    private String botPoseNtEntryName() {
        return m_useMegaTag2 ? "botpose_orb_wpiblue" : "botpose_wpiblue";
    }

    private void fuseBotPoseFromNetworkTables(BotPoseNetworkTableSample sample) {
        Matrix<N3, N1> stdDevs = m_useMegaTag2
                ? VecBuilder.fill(
                        DriveConstants.kMegaTag2VisionStdDevXMeters,
                        DriveConstants.kMegaTag2VisionStdDevYMeters,
                        DriveConstants.kMegaTag2VisionStdDevThetaRadians)
                : VecBuilder.fill(
                        DriveConstants.kMegaTag1VisionStdDevXMeters,
                        DriveConstants.kMegaTag1VisionStdDevYMeters,
                        DriveConstants.kMegaTag1VisionStdDevThetaRadians);
        m_drivetrain.addVisionMeasurement(sample.pose, sample.timestampSeconds, stdDevs);
    }

    private static final int kLogThrottlePeriods = 100;

    private static String[] buildSecondaryLimelightCandidates(String primaryName, String secondaryName) {
        if (secondaryName == null || secondaryName.isBlank()) {
            return new String[0];
        }

        LinkedHashSet<String> names = new LinkedHashSet<>();
        names.add(secondaryName);
        for (String alias : kCommonSecondaryLimelightAliases) {
            if (alias != null && !alias.isBlank()) {
                names.add(alias);
            }
        }
        names.remove(primaryName);
        return new ArrayList<>(names).toArray(String[]::new);
    }

    /**
     * Picks which NT table to use for secondary botpose / dashboard when multiple aliases are possible.
     * Uses the first candidate that looks connected (pipeline type string, or live latency / target data).
     */
    private String resolveSecondaryLimelightName() {
        for (String candidate : m_secondaryLimelightCandidates) {
            String pipelineType = LimelightHelpers.getLimelightNTString(candidate, "getpipetype");
            if (pipelineType != null && !pipelineType.isEmpty()) {
                return candidate;
            }
            double tl = LimelightHelpers.getLimelightNTDouble(candidate, "tl");
            double tv = LimelightHelpers.getLimelightNTDouble(candidate, "tv");
            if (tl > 0.0 || tv >= 1.0) {
                return candidate;
            }
        }
        return null;
    }

    /**
     * NT table name for reading secondary pose / tv when a secondary camera is configured. Prefer resolved alias;
     * otherwise use the configured name so we still read {@code tv} if {@link #resolveSecondaryLimelightName} missed
     * (e.g. empty {@code getpipetype} on some firmware).
     */
    private String getSecondaryReadTableName() {
        if (m_secondaryLimelightName == null) {
            return null;
        }
        return m_resolvedSecondaryLimelightName != null
                ? m_resolvedSecondaryLimelightName
                : m_secondaryLimelightName;
    }

    /** True if any secondary alias reports a valid target ({@code tv} ≥ 1). */
    private boolean anySecondaryCandidateHasTagLock() {
        for (String candidate : m_secondaryLimelightCandidates) {
            if (LimelightHelpers.getLimelightNTDouble(candidate, "tv") >= 1.0) {
                return true;
            }
        }
        return false;
    }

    /** Forwards Limelight NetworkTables values to SmartDashboard under the given prefix. */
    private void publishLimelightToSmartDashboard(
            String sdPrefix,
            String llName,
            Optional<BotPoseNetworkTableSample> ntPose,
            int[] periodsWithoutDataCounter) {
        double tv = LimelightHelpers.getLimelightNTDouble(llName, "tv");
        double getpipe = LimelightHelpers.getLimelightNTDouble(llName, "getpipe");
        String getpipetype = LimelightHelpers.getLimelightNTString(llName, "getpipetype");

        boolean hasConnection = getpipetype != null && !getpipetype.isEmpty();
        String status;
        if (!hasConnection) {
            status = "No data (Limelight not connected to this NT server)";
            periodsWithoutDataCounter[0]++;
            if (Utils.isSimulation() && periodsWithoutDataCounter[0] == 1) {
                SmartDashboard.putString(
                        sdPrefix + "help",
                        "In sim use http://localhost:5801 for Limelight config; set NT server to this PC's IP.");
            }
            if (periodsWithoutDataCounter[0] % kLogThrottlePeriods == 1) {
                DriverStation.reportWarning(
                        "Limelight: No NetworkTables data from \""
                                + llName
                                + "\". "
                                + (Utils.isSimulation()
                                        ? "In simulation set Limelight NT server to this computer's IP (http://localhost:5801)."
                                        : "Check Limelight power and team number / NT server setting."),
                        false);
            }
        } else {
            periodsWithoutDataCounter[0] = 0;
            status = tv >= 1.0 ? "Target" : "No target";
            SmartDashboard.putString(sdPrefix + "help", "");
        }
        SmartDashboard.putString(sdPrefix + "status", status);

        SmartDashboard.putNumber(sdPrefix + "tv", tv);
        SmartDashboard.putNumber(sdPrefix + "tx", LimelightHelpers.getLimelightNTDouble(llName, "tx"));
        SmartDashboard.putNumber(sdPrefix + "ty", LimelightHelpers.getLimelightNTDouble(llName, "ty"));
        SmartDashboard.putNumber(sdPrefix + "ta", LimelightHelpers.getLimelightNTDouble(llName, "ta"));
        SmartDashboard.putNumber(sdPrefix + "tid", LimelightHelpers.getLimelightNTDouble(llName, "tid"));
        SmartDashboard.putNumber(sdPrefix + "tl", LimelightHelpers.getLimelightNTDouble(llName, "tl"));
        SmartDashboard.putNumber(sdPrefix + "cl", LimelightHelpers.getLimelightNTDouble(llName, "cl"));
        SmartDashboard.putNumber(sdPrefix + "getpipe", getpipe);
        SmartDashboard.putString(sdPrefix + "getpipetype", getpipetype != null ? getpipetype : "");

        SmartDashboard.putString(sdPrefix + "pose_nt_key", botPoseNtEntryName());
        if (ntPose.isPresent()) {
            double[] botpose = ntPose.get().poseArray;
            SmartDashboard.putNumber(sdPrefix + "pose_x", botpose[0]);
            SmartDashboard.putNumber(sdPrefix + "pose_y", botpose[1]);
            SmartDashboard.putNumber(sdPrefix + "pose_z", botpose[2]);
            SmartDashboard.putNumber(sdPrefix + "pose_roll", botpose[3]);
            SmartDashboard.putNumber(sdPrefix + "pose_pitch", botpose[4]);
            SmartDashboard.putNumber(sdPrefix + "pose_yaw", botpose[5]);
            if (botpose.length >= 11) {
                SmartDashboard.putNumber(sdPrefix + "latency_ms", botpose[6]);
                SmartDashboard.putNumber(sdPrefix + "tag_count", botpose[7]);
                SmartDashboard.putNumber(sdPrefix + "tag_span", botpose[8]);
                SmartDashboard.putNumber(sdPrefix + "tag_dist", botpose[9]);
                SmartDashboard.putNumber(sdPrefix + "tag_area", botpose[10]);
            }
        }
    }

    private void publishFusedRobotPoseToDashboard(Pose2d pose) {
        m_poseFieldTypePublisher.set("Field2d");
        m_dashboardPose[0] = pose.getX();
        m_dashboardPose[1] = pose.getY();
        m_dashboardPose[2] = pose.getRotation().getDegrees();
        m_dashboardPosePublisher.set(m_dashboardPose);

        double odometryX = pose.getX();
        double odometryY = pose.getY();
        var hubFacingRotation = DriveConstants.rotationToFaceHubFromShotMap(pose);
        m_idealShooterPose[0] = odometryX;
        m_idealShooterPose[1] = odometryY;
        m_idealShooterPose[2] = hubFacingRotation.getDegrees();
        m_idealShooterPosePublisher.set(m_idealShooterPose);

        m_fusedPose2dSmartDashboard.set(pose);

        SmartDashboard.putNumber("Odometry/pose_x", pose.getX());
        SmartDashboard.putNumber("Odometry/pose_y", pose.getY());
        SmartDashboard.putNumber("Odometry/pose_z", 0.0);
        SmartDashboard.putNumber("Odometry/pose_roll", 0.0);
        SmartDashboard.putNumber("Odometry/pose_pitch", 0.0);
        SmartDashboard.putNumber("Odometry/pose_yaw", pose.getRotation().getDegrees());
    }

    private static void publishLimelightEstimatedPose(
            double[] buffer,
            DoubleArrayPublisher posePub,
            BooleanPublisher validPub,
            Pose2d pose) {
        buffer[0] = pose.getX();
        buffer[1] = pose.getY();
        buffer[2] = pose.getRotation().getDegrees();
        posePub.set(buffer);
        validPub.set(true);
    }
}
