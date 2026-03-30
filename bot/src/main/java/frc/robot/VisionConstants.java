// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Limelight NetworkTables table names (Settings → hostname on each device) and vision options.
 * Each camera publishes to {@code /limelight/...} and {@code /limelight2/...} etc.
 */
public final class VisionConstants {
    private VisionConstants() {}

    /** Primary Limelight (MegaTag2 fusion in {@link frc.robot.subsystems.VisionMeasurement}). */
    public static final String kPrimaryLimelightName = "limelight";

    /**
     * Second Limelight — also fuses into Phoenix odometry via {@code addVisionMeasurement}.
     * Must match the NT table name for that camera (e.g. rename in Limelight web UI to {@code limelight2}).
     * Set to {@code null} or empty to disable.
     */
    public static final String kSecondaryLimelightName = "limelight-front";
}
