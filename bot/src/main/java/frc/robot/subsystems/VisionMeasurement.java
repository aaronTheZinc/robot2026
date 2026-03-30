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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
 * freshly computed so the robot would face the hub from that position ({@link DriveConstants#rotationToFaceHub(Pose2d)}).
 */
public class VisionMeasurement extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final String m_primaryLimelightName;
    private final String m_secondaryLimelightName;
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

    private final DoubleArrayPublisher m_limelight2EstimatedPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("limelight2EstimatedPose")
            .publish();
    private final BooleanPublisher m_limelight2EstimatedPoseValidPublisher = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getBooleanTopic("limelight2EstimatedPoseValid")
            .publish();

    private final double[] m_dashboardPose = new double[3];
    private final double[] m_idealShooterPose = new double[3];
    private final double[] m_limelightEstimatedPose = new double[3];
    private final double[] m_limelight2EstimatedPose = new double[3];

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
        m_useMegaTag2 = useMegaTag2;
    }

    /** Primary + secondary names from {@link VisionConstants}, MegaTag2. */
    public VisionMeasurement(CommandSwerveDrivetrain drivetrain) {
        this(
                drivetrain,
                VisionConstants.kPrimaryLimelightName,
                VisionConstants.kSecondaryLimelightName,
                true);
    }

    /** Primary Limelight {@code tv} ≥ 1 or secondary (if configured): pipeline has a valid target. */
    public boolean limelightHasTagLock() {
        if (LimelightHelpers.getLimelightNTDouble(m_primaryLimelightName, "tv") >= 1.0) {
            return true;
        }
        if (m_secondaryLimelightName != null
                && LimelightHelpers.getLimelightNTDouble(m_secondaryLimelightName, "tv") >= 1.0) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        var driveState = m_drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(m_primaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        if (m_secondaryLimelightName != null) {
            LimelightHelpers.SetRobotOrientation(m_secondaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        }

        Optional<BotPoseNetworkTableSample> ntPosePrimary =
                LimelightHelpers.getBotPoseNetworkTableSample(m_primaryLimelightName, botPoseNtEntryName());
        publishLimelightToSmartDashboard(
                "Limelight/", m_primaryLimelightName, ntPosePrimary, m_periodsWithoutDataPrimary);

        double tvPrimary = LimelightHelpers.getLimelightNTDouble(m_primaryLimelightName, "tv");
        boolean validPrimary = tvPrimary >= 1.0 && ntPosePrimary.isPresent();
        if (validPrimary) {
            publishLimelightEstimatedPose(
                    m_limelightEstimatedPose,
                    m_limelightEstimatedPosePublisher,
                    m_limelightEstimatedPoseValidPublisher,
                    ntPosePrimary.get().pose);
            if (DriverStation.isEnabled()) {
                fuseBotPoseFromNetworkTables(ntPosePrimary.get());
            }
        } else {
            m_limelightEstimatedPoseValidPublisher.set(false);
        }

        if (m_secondaryLimelightName != null) {
            Optional<BotPoseNetworkTableSample> ntPoseSecondary =
                    LimelightHelpers.getBotPoseNetworkTableSample(m_secondaryLimelightName, botPoseNtEntryName());
            publishLimelightToSmartDashboard(
                    "Limelight2/", m_secondaryLimelightName, ntPoseSecondary, m_periodsWithoutDataSecondary);

            double tvSecondary = LimelightHelpers.getLimelightNTDouble(m_secondaryLimelightName, "tv");
            boolean validSecondary = tvSecondary >= 1.0 && ntPoseSecondary.isPresent();
            if (validSecondary) {
                publishLimelightEstimatedPose(
                        m_limelight2EstimatedPose,
                        m_limelight2EstimatedPosePublisher,
                        m_limelight2EstimatedPoseValidPublisher,
                        ntPoseSecondary.get().pose);
                if (DriverStation.isEnabled()) {
                    fuseBotPoseFromNetworkTables(ntPoseSecondary.get());
                }
            } else {
                m_limelight2EstimatedPoseValidPublisher.set(false);
            }
        }

        publishFusedRobotPoseToDashboard(m_drivetrain.getState().Pose);
    }

    /** {@code botpose_orb_wpi*} when MegaTag2, else {@code botpose_wpi*}, matching alliance. */
    private String botPoseNtEntryName() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (m_useMegaTag2) {
            return isRed ? "botpose_orb_wpired" : "botpose_orb_wpiblue";
        }
        return isRed ? "botpose_wpired" : "botpose_wpiblue";
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
        var hubFacingRotation = DriveConstants.rotationToFaceHub(pose);
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
