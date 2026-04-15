// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Limelight NetworkTables table names (Settings → name on each device) and vision options.
 * Each camera publishes to {@code /limelight/...}, {@code /limelight-front/...}, etc.
 * <p>
 * <b>Two Limelights on the same robot:</b> each unit must use a <em>different</em> name
 * (matching these constants) <em>and</em> a <em>different</em> static IP on the robot network
 * (e.g. {@code 10.TE.AM.11} vs {@code 10.TE.AM.12}). If both still use the factory default
 * {@code limelight} hostname or the same IP, mDNS ({@code limelight.local}) and the web feed
 * ({@code :5801}) will collide — one feed appears to “die” when the other is plugged in even
 * though the robot code is fine. Use the per-device IP in the browser if hostnames conflict.
 * <p>
 * <b>After changing static IPs, “nothing works” on any Limelight:</b> every device on the robot
 * Ethernet must share one subnet (typically {@code 10.TE.AM.x} with mask {@code 255.255.255.0},
 * where {@code TE.AM} is the four-digit team number, e.g. team 190 → {@code 10.1.90.x}). Use
 * <em>different</em> last octets for roboRIO, radio, and each Limelight (no duplicate IPs). Set each
 * Limelight’s <b>NetworkTables server address</b> to the roboRIO address (usually {@code 10.TE.AM.2})
 * or your PC’s IP when testing over USB—wrong NT host makes tables empty on the robot. Reboot each
 * Limelight after saving network settings; verify with ping from the Driver Station PC before relying
 * on {@code *.local} names.
 */
public final class VisionConstants {
    private VisionConstants() {}

    /** Primary Limelight (MegaTag2 fusion in {@link frc.robot.subsystems.VisionMeasurement}). */
    public static final String kPrimaryLimelightName = "limelight-back";

    /**
     * Second Limelight — also fuses into Phoenix odometry via {@code addVisionMeasurement}.
     * Must match the NT table name for that camera (e.g. rename in Limelight web UI to {@code limelight2}).
     * Set to {@code null} or empty to disable.
     */
    public static final String kSecondaryLimelightName = "limelight-front";
}
