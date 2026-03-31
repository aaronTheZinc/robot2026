// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Motor test constants for the drivetrain (drive and steer modules).
 * All test speeds and inversion for drive/steer in this file.
 * Do not edit generated TunerConstants for drivetrain config.
 */
public final class DriveConstants {
    private DriveConstants() {}

    /** Max voltage (V) for swerve drive/steer during motor test (super slow). */
    public static final double kSwerveTestMaxVolts = 3.0;

    /** Motor test: normalized min/max speed for drive modules (drive0–drive3). */
    public static final double kDriveTestMinSpeed = -1.0;
    public static final double kDriveTestMaxSpeed = 1.0;

    /** Motor test: normalized min/max speed for steer modules (steer0–steer3). */
    public static final double kSteerTestMinSpeed = -1.0;
    public static final double kSteerTestMaxSpeed = 1.0;

    /** Motor test: default invert for drive modules. */
    public static final boolean kDriveTestDefaultInvert = false;

    /** Motor test: default invert for steer modules. */
    public static final boolean kSteerTestDefaultInvert = false;

    // ----- PathPlanner (deploy/pathplanner) — keep JSON globalConstraints in sync with these -----

    /** Default max chassis translation speed (m/s) for autonomous paths. */
    public static final double kAutoPathMaxVelocityMps = 1.25;
    /** Default max chassis translation acceleration (m/s²) for autonomous paths. */
    public static final double kAutoPathMaxAccelerationMps2 = 2.5;
    /** Default max angular velocity (deg/s) for autonomous paths. */
    public static final double kAutoPathMaxAngularVelocityDps = 360.0;
    /** Default max angular acceleration (deg/s²) for autonomous paths. */
    public static final double kAutoPathMaxAngularAccelerationDps2 = 480.0;

    // ----- PathPlanner holonomic follower (PPHolonomicDriveController in CommandSwerveDrivetrain) -----

    /**
     * Translation PID for path following. Moderate D can reduce weave on carpet; keep I at 0 unless tuned.
     */
    public static final double kHolonomicTranslationP = 0.5;
    public static final double kHolonomicTranslationI = 0.0;
    public static final double kHolonomicTranslationD = 0.02;

    /**
     * Rotation PID — high P (e.g. 7) often over-corrects when wheels slip, feeding runaway rotation.
     * Start around 2 with small D; tune on field.
     */
    public static final double kHolonomicRotationP = 0.07;
    public static final double kHolonomicRotationI = 0.0;
    public static final double kHolonomicRotationD = 0.03;

    /**
     * Scales omega from PathPlanner before {@code ApplyRobotSpeeds}. Use +1.0 normally; set to -1.0
     * only if holonomic heading correction clearly drives the wrong way (curved paths will invert too).
     */
    public static final double kPathFollowerOmegaMultiplier = 1.0;

    // ----- Slow on-the-fly pathfind (driver waypoint button) -----

    /** Max translation speed (m/s) for slow driver pathfind. */
    public static final double kSlowDriverPathfindMaxVelocityMps = 0.5;
    /** Max translation acceleration (m/s²) for slow driver pathfind. */
    public static final double kSlowDriverPathfindMaxAccelerationMps2 = 0.85;
    /** Max angular velocity (deg/s) for slow driver pathfind. */
    public static final double kSlowDriverPathfindMaxAngularVelocityDps = 75.0;
    /** Max angular acceleration (deg/s²) for slow driver pathfind. */
    public static final double kSlowDriverPathfindMaxAngularAccelerationDps2 = 100.0;

    // ----- Teleop field-centric rotation -----

    /**
     * Right-stick X scaling for rotation (+1.0 = stick right yields positive omega from controller).
     * Set to -1.0 if field-centric rotation feels inverted.
     */
    public static final double kTeleopRotationStickSign = 1.0;

    // ----- Hub / goal (field pose, same frame as PathPlanner / fused odometry — blue alliance WPILib) -----

    /** Hub center X (m). */
    public static final double kHubFieldXMeters = 4.62;
    /** Hub center Y (m). */
    public static final double kHubFieldYMeters = 4.0;

    /**
     * Extra rotation added to the bearing toward the hub. Use {@code 0} when the scorer aligns with robot +X;
     * use {@code 180} if the shooter is on the back and should face the hub.
     */
    public static final double kHubFacingBearingOffsetDegrees = 0.0;

    /**
     * Heading to hold so the scoring side faces the hub (field frame), from fused pose.
     * Bearing is toward hub plus {@link #kHubFacingBearingOffsetDegrees}.
     */
    public static Rotation2d rotationToFaceHub(Pose2d robotPose) {
        return rotationToFaceFieldPoint(robotPose, kHubFieldXMeters, kHubFieldYMeters);
    }

    /**
     * Heading so the scoring side faces a field point (same convention as {@link #rotationToFaceHub} —
     * bearing toward the target plus {@link #kHubFacingBearingOffsetDegrees}).
     */
    public static Rotation2d rotationToFaceFieldPoint(
            Pose2d robotPose, double targetXMeters, double targetYMeters) {
        double dx = targetXMeters - robotPose.getX();
        double dy = targetYMeters - robotPose.getY();
        if (dx * dx + dy * dy < 1e-8) {
            return robotPose.getRotation();
        }
        Rotation2d toward = Rotation2d.fromRadians(Math.atan2(dy, dx));
        return toward.plus(Rotation2d.fromDegrees(kHubFacingBearingOffsetDegrees));
    }

    /** Heading PID for {@link com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle} while shooting. */
    public static final double kHubFacingHeadingKp = 2.0;
    public static final double kHubFacingHeadingKi = 0.0;
    public static final double kHubFacingHeadingKd = 3.0;

    /** Translation deadband (m/s) for hub lock — keep small; motion is zero. */
    public static final double kHubFacingDeadbandMps = 0.05;
    /** Rotational deadband (rad/s) — larger values reduce hunting / oscillation near heading setpoint. */
    public static final double kHubFacingRotationalDeadbandRad = 0.22;

    /**
     * Max magnitude of angular rate (rad/s) while hub locking — lower than teleop spin cap to limit
     * overshoot and back-and-forth. Tune on field (try 2.0–3.5).
     */
    public static final double kHubFacingMaxAngularRateRadPerSec = 2.5;

    // ----- Limelight MegaTag2 → Phoenix pose estimator (addVisionMeasurement) -----

    /**
     * Standard deviations for MegaTag2 vision fusion. MegaTag2 uses injected gyro heading, so vision
     * yaw is not applied (large theta std dev). Tune xy on field vs tag distance / multipath.
     */
    public static final double kMegaTag2VisionStdDevXMeters = 0.7;
    public static final double kMegaTag2VisionStdDevYMeters = 0.7;
    public static final double kMegaTag2VisionStdDevThetaRadians = 999_999.0;

    /** Standard deviations when using MegaTag1 ({@code botpose_wpiblue} / {@code botpose_wpired}). */
    public static final double kMegaTag1VisionStdDevXMeters = 0.75;
    public static final double kMegaTag1VisionStdDevYMeters = 0.75;
    public static final double kMegaTag1VisionStdDevThetaRadians = 0.75;
}
