// Copyright (c) FIRST and other WPILib contributors. Open Source Software; see license.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Teleop hub-align heading correction: driver aims manually, then presses L3 (left stick) at a known
 * distance from the hub. We store the heading error vs {@link DriveConstants#rotationToFaceHubFromShotMap}
 * (KNN map linear fit) and scale it linearly with distance so farther shots pick up proportionally more
 * offset.
 */
public final class HubAlignCalibration {
    private static final String kPrefix = "Hub Align/";

    /** Ignore calibration if closer than this (m) — bearing is ill-conditioned. */
    private static final double kMinCalibrationDistanceM = 0.25;

    private double calDistanceM;
    private double calOffsetDeg;

    /** Straight-line distance from robot to field hub (m). */
    public static double distanceToHubMeters(Pose2d pose) {
        double dx = DriveConstants.kHubFieldXMeters - pose.getX();
        double dy = DriveConstants.kHubFieldYMeters - pose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Call when the driver is satisfied with aim: records distance and
     * {@code wrap180(currentHeading − rotationToFaceHubFromShotMap)} as the offset at that distance.
     */
    public void recordAtCurrentPose(Pose2d pose) {
        double d = distanceToHubMeters(pose);
        if (d < kMinCalibrationDistanceM) {
            return;
        }
        Rotation2d nominal = DriveConstants.rotationToFaceHubFromShotMap(pose);
        double deltaDeg =
                Math.toDegrees(
                        MathUtil.angleModulus(
                                pose.getRotation().getRadians() - nominal.getRadians()));
        calDistanceM = d;
        calOffsetDeg = deltaDeg;
    }

    public void clear() {
        calDistanceM = 0.0;
        calOffsetDeg = 0.0;
    }

    /**
     * Linear model: {@code offset(d) = calOffsetDeg × (d / calDistanceM)} after calibration; {@code 0}
     * if never calibrated.
     */
    public double getScaledOffsetDeg(Pose2d pose) {
        if (calDistanceM < 1e-6) {
            return 0.0;
        }
        double d = distanceToHubMeters(pose);
        return calOffsetDeg * (d / calDistanceM);
    }

    public boolean isCalibrated() {
        return calDistanceM > 1e-6;
    }

    /** Publishes offset, distance, calibration snapshot, scale (deg/m), and heading error for tuning. */
    public void publishTelemetry(Pose2d pose) {
        double d = distanceToHubMeters(pose);
        double offsetApplied = getScaledOffsetDeg(pose);
        Rotation2d target =
                DriveConstants.rotationToFaceHubFromShotMap(pose).plus(Rotation2d.fromDegrees(offsetApplied));
        double errRad =
                MathUtil.angleModulus(
                        target.getRadians() - pose.getRotation().getRadians());

        SmartDashboard.putNumber(kPrefix + "Distance to hub (m)", d);
        SmartDashboard.putNumber(kPrefix + "Offset applied (deg)", offsetApplied);
        SmartDashboard.putNumber(kPrefix + "Cal distance (m)", calDistanceM);
        SmartDashboard.putNumber(kPrefix + "Cal offset at cal (deg)", calOffsetDeg);
        double degPerM = calDistanceM > 1e-6 ? calOffsetDeg / calDistanceM : 0.0;
        SmartDashboard.putNumber(kPrefix + "Scale (deg per m)", degPerM);
        SmartDashboard.putNumber(kPrefix + "Heading error (deg)", Math.toDegrees(errRad));
        SmartDashboard.putBoolean(kPrefix + "Calibrated", isCalibrated());
    }
}
