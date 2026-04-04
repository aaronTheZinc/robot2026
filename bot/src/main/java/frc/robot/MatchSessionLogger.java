// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionMeasurement;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;

/**
 * Lightweight CSV session log (fused pose, Limelight-reported field poses, alliance, locks/offsets, chassis
 * motion). Writes under
 * {@code /home/lvuser/logs/} on the roboRIO (or the working directory in simulation). Call {@link #periodic}
 * after the scheduler runs so vision caches are current.
 */
public final class MatchSessionLogger {
    private static final double kLogPeriodSeconds = 0.1;
    private static final int kFlushEveryLines = 5;
    private static final String kEnableKey = "Debug/Match session CSV log";
    private static final String kLogPathKey = "Debug/Match session log file";

    private BufferedWriter m_writer;
    private double m_lastLogTime;
    private Pose2d m_lastPose;
    private boolean m_hasLastPose;
    private int m_linesSinceFlush;
    private boolean m_reportedIoError;

    public MatchSessionLogger() {
        SmartDashboard.setDefaultBoolean(kEnableKey, true);
    }

    /** Open a new timestamped file for this robot boot. */
    public void robotInit() {
        try {
            Path dir = Filesystem.getOperatingDirectory().toPath().resolve("logs");
            Files.createDirectories(dir);
            String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
            Path path = dir.resolve("session_" + stamp + ".csv");
            m_writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8);
            SmartDashboard.putString(kLogPathKey, path.toString());
            m_writer.write(
                    "fpga_s,match_time_s,mode,alliance,ds_attached,"
                            + "pose_x_m,pose_y_m,pose_deg,vx_mps,vy_mps,omega_radps,"
                            + "dx_m,dy_m,dtheta_deg,ll_primary_lock,ll_secondary_lock,"
                            + "primary_tx_deg,primary_ty_deg,secondary_tx_deg,secondary_ty_deg,"
                            + "report_primary_valid,report_primary_x_m,report_primary_y_m,report_primary_deg,"
                            + "report_secondary_valid,report_secondary_x_m,report_secondary_y_m,report_secondary_deg\n");
            m_writer.flush();
            m_linesSinceFlush = 0;
        } catch (IOException e) {
            DriverStation.reportError("[MatchSessionLogger] Failed to open log: " + e.getMessage(), false);
            m_writer = null;
            SmartDashboard.putString(kLogPathKey, "(failed — see console)");
        }
    }

    /** Flush when disabled so data is on disk between matches. */
    public void flush() {
        if (m_writer == null) {
            return;
        }
        try {
            m_writer.flush();
        } catch (IOException e) {
            handleIoError(e);
        }
    }

    public void periodic(CommandSwerveDrivetrain drivetrain, VisionMeasurement vision) {
        if (m_writer == null || m_reportedIoError) {
            return;
        }
        if (!SmartDashboard.getBoolean(kEnableKey, true)) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (m_lastLogTime != 0.0 && now - m_lastLogTime < kLogPeriodSeconds) {
            return;
        }
        m_lastLogTime = now;

        var state = drivetrain.getState();
        Pose2d pose = state.Pose;
        var speeds = state.Speeds;

        double dx = 0.0;
        double dy = 0.0;
        double dthetaDeg = 0.0;
        if (m_hasLastPose) {
            dx = pose.getX() - m_lastPose.getX();
            dy = pose.getY() - m_lastPose.getY();
            dthetaDeg = pose.getRotation().minus(m_lastPose.getRotation()).getDegrees();
        }
        m_lastPose = pose;
        m_hasLastPose = true;

        String mode;
        if (DriverStation.isDisabled()) {
            mode = "DISABLED";
        } else if (DriverStation.isAutonomous()) {
            mode = "AUTO";
        } else if (Robot.inTestMode()) {
            mode = "TEST";
        } else {
            mode = "TELEOP";
        }

        String alliance = DriverStation.getAlliance().map(a -> a.name()).orElse("UNKNOWN");

        String line =
                String.format(
                        Locale.US,
                        "%.3f,%.3f,%s,%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f,%.4f,%.4f,%.4f,"
                                + "%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f\n",
                        now,
                        DriverStation.getMatchTime(),
                        mode,
                        alliance,
                        DriverStation.isDSAttached() ? 1 : 0,
                        pose.getX(),
                        pose.getY(),
                        pose.getRotation().getDegrees(),
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        dx,
                        dy,
                        dthetaDeg,
                        vision.primaryLimelightHasTagLock() ? 1 : 0,
                        vision.secondaryLimelightHasTagLock() ? 1 : 0,
                        vision.getPrimaryTxDeg(),
                        vision.getPrimaryTyDeg(),
                        vision.getSecondaryTxDeg(),
                        vision.getSecondaryTyDeg(),
                        vision.primaryReportedPoseValid() ? 1 : 0,
                        vision.getPrimaryReportedPoseXMeters(),
                        vision.getPrimaryReportedPoseYMeters(),
                        vision.getPrimaryReportedPoseDeg(),
                        vision.secondaryReportedPoseValid() ? 1 : 0,
                        vision.getSecondaryReportedPoseXMeters(),
                        vision.getSecondaryReportedPoseYMeters(),
                        vision.getSecondaryReportedPoseDeg());

        try {
            m_writer.write(line);
            m_linesSinceFlush++;
            if (m_linesSinceFlush >= kFlushEveryLines) {
                m_writer.flush();
                m_linesSinceFlush = 0;
            }
        } catch (IOException e) {
            handleIoError(e);
        }
    }

    private void handleIoError(IOException e) {
        if (!m_reportedIoError) {
            DriverStation.reportError("[MatchSessionLogger] I/O error: " + e.getMessage(), false);
            m_reportedIoError = true;
        }
        try {
            if (m_writer != null) {
                m_writer.close();
            }
        } catch (IOException ignored) {
        }
        m_writer = null;
    }
}
