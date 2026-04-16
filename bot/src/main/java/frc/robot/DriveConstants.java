// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public static final double kAutoPathMaxVelocityMps = 0.7;
    /** Default max chassis translation acceleration (m/s²) for autonomous paths. */
    public static final double kAutoPathMaxAccelerationMps2 = 2.5;
    /** Default max angular velocity (deg/s) for autonomous paths. */
    public static final double kAutoPathMaxAngularVelocityDps = 360.0;
    /** Default max angular acceleration (deg/s²) for autonomous paths. */
    public static final double kAutoPathMaxAngularAccelerationDps2 = 480.0;

    /** Max time (s) for PathPlanner named {@code Heading 0} in-place rotation before timeout. */
    public static final double kAutoFieldHeadingZeroTimeoutSeconds = 4.0;

    // ----- PathPlanner holonomic follower (PPHolonomicDriveController in CommandSwerveDrivetrain) -----

    /**
     * Translation PID for path following. Moderate D can reduce weave on carpet; keep I at 0 unless tuned.
     */
    public static final double kHolonomicTranslationP = 0.2;
    public static final double kHolonomicTranslationI = 0.0;
    public static final double kHolonomicTranslationD = 0.02;

    /**
     * Rotation PID — high P (e.g. 7) often over-corrects when wheels slip, feeding runaway rotation.
     * Tune on field if heading hunts or overshoots toward path goal heading.
     */
    public static final double kHolonomicRotationP = 2.0;
    public static final double kHolonomicRotationI = 0.0;
    public static final double kHolonomicRotationD = 0.06;

    /**
     * Scales omega from PathPlanner before {@code ApplyRobotSpeeds}. CTRE swerve often needs {@code -1.0} so
     * holonomic rotation tracks path heading (if the robot spins against the path, flip this). Translation
     * is unchanged.
     */
    public static final double kPathFollowerOmegaMultiplier = -1.0;

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

    /**
     * Driver right bumper (hub align): stop correcting within this many degrees of the bearing toward the hub.
     */
    public static final double kTeleopHubAlignToleranceDeg = 3.0;
    /**
     * Proportional gain: omega (rad/s) += {@code kTeleopHubAlignKp} × heading error (rad), then clamped.
     */
    public static final double kTeleopHubAlignKp = 4.5;
    /** Max angular velocity (rad/s) while hub aligning (right bumper). */
    public static final double kTeleopHubAlignMaxOmegaRadPerSec = 2.75;

    /**
     * Extra sign on hub-align omega relative to WPILib-style heading error. Use {@code -1.0} if the robot
     * turns away from the hub and never settles inside {@link #kTeleopHubAlignToleranceDeg}; use {@code +1.0}
     * if hub assist already matches the shortest turn toward the hub.
     */
    public static final double kTeleopHubAlignOmegaSign = -1.0;

    /**
     * Field-centric rotational rate (rad/s) to turn the scoring side toward the hub. Zero within
     * {@link #kTeleopHubAlignToleranceDeg}. Uses {@link #kTeleopRotationStickSign} so hub assist matches
     * teleop rotation direction, and {@link #kTeleopHubAlignOmegaSign} if chassis omega is inverted vs error.
     *
     * @param hubHeadingOffsetDeg additive field heading (deg) on top of the shot-map hub bearing (e.g. driver
     *     calibration)
     */
    public static double teleopOmegaTowardHub(Pose2d robotPose, double hubHeadingOffsetDeg) {
        // Matches dashboard {@code Pose/idealShooterPose} third element and {@code hubShotMapHeadingTowardHubDeg}.
        Rotation2d target = rotationToFaceHubForShooting(robotPose, hubHeadingOffsetDeg);
        double errorRad =
                MathUtil.angleModulus(target.getRadians() - robotPose.getRotation().getRadians());
        if (Math.abs(errorRad) <= Math.toRadians(kTeleopHubAlignToleranceDeg)) {
            return 0.0;
        }
        double omega = kTeleopHubAlignKp * errorRad;
        double cap = kTeleopHubAlignMaxOmegaRadPerSec;
        if (omega > cap) {
            omega = cap;
        } else if (omega < -cap) {
            omega = -cap;
        }
        return kTeleopHubAlignOmegaSign * kTeleopRotationStickSign * omega;
    }

    /**
     * Field-centric omega (rad/s) to rotate toward field heading 0 rad (+X forward). Same P/clamp/sign as
     * {@link #teleopOmegaTowardHub}; reapplies every control frame so it is not a one-shot setpoint.
     */
    public static double teleopOmegaTowardFieldHeadingZero(Pose2d robotPose) {
        double errorRad = MathUtil.angleModulus(-robotPose.getRotation().getRadians());
        if (Math.abs(errorRad) <= Math.toRadians(kTeleopHubAlignToleranceDeg)) {
            return 0.0;
        }
        double omega = kTeleopHubAlignKp * errorRad;
        double cap = kTeleopHubAlignMaxOmegaRadPerSec;
        if (omega > cap) {
            omega = cap;
        } else if (omega < -cap) {
            omega = -cap;
        }
        return kTeleopHubAlignOmegaSign * kTeleopRotationStickSign * omega;
    }

    /** True when field heading is within {@link #kTeleopHubAlignToleranceDeg} of 0 rad. */
    public static boolean isFieldHeadingNearZero(Pose2d robotPose) {
        double errRad = MathUtil.angleModulus(-robotPose.getRotation().getRadians());
        return Math.abs(errRad) <= Math.toRadians(kTeleopHubAlignToleranceDeg);
    }

    // ----- Hub / goal (field pose, same frame as PathPlanner / fused odometry — blue alliance WPILib) -----

    /** Full field length X (m), blue-origin WPILib field frame. */
    public static final double kFieldLengthXMeters = 16.46;
    /** Full field width Y (m), blue-origin WPILib field frame. */
    public static final double kFieldWidthYMeters = 8.23;
    /** Blue-side hub center X (m). */
    public static final double kHubFieldXMeters = 4.62;
    /** Blue-side hub center Y (m). */
    public static final double kHubFieldYMeters = 4.0;

    /**
     * Extra rotation added to the bearing toward the hub. Use {@code 0} when the scorer aligns with robot +X;
     * use {@code 180} if the shooter is on the back and should face the hub.
     */
    public static final double kHubFacingBearingOffsetDegrees = 0.0;

    /**
     * Legacy shot-map LSQ coefficients (dashboard / log fit). On-robot hub align does not apply this offset —
     * {@link #rotationToFaceHubFromShotMap} matches {@link #rotationToFaceHub} (geometric bearing).
     */
    public static final double kHubShotMapHeadingOffsetA = 0.0;
    public static final double kHubShotMapHeadingOffsetPerMDx = 0.0;
    public static final double kHubShotMapHeadingOffsetPerMDy = 0.0;

    /**
     * Heading to hold so the scoring side faces the hub (field frame), from fused pose.
     * Bearing is toward hub plus {@link #kHubFacingBearingOffsetDegrees}.
     */
    public static Rotation2d rotationToFaceHub(Pose2d robotPose) {
        return rotationToFaceFieldPoint(robotPose, hubTargetXMeters(), hubTargetYMeters());
    }

    /**
     * Same hub-facing heading as {@link #rotationToFaceHub} (no extra shot-map bearing offset).
     */
    public static Rotation2d rotationToFaceHubFromShotMap(Pose2d robotPose) {
        return rotationToFaceHub(robotPose);
    }

    /**
     * Full shooting / hub-align aim: {@link #rotationToFaceHubFromShotMap} plus optional trim (deg), e.g. driver
     * calibration from {@link frc.robot.HubAlignCalibration}.
     */
    public static Rotation2d rotationToFaceHubForShooting(Pose2d robotPose, double extraHeadingOffsetDeg) {
        return rotationToFaceHubFromShotMap(robotPose).plus(Rotation2d.fromDegrees(extraHeadingOffsetDeg));
    }

    /** Unused on-robot (always 0); kept for API parity with dashboard helpers. */
    public static double hubShotMapHeadingOffsetRad(double hubMinusRobotDxM, double hubMinusRobotDyM) {
        return 0.0;
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

    /** Active alliance hub X (m) in the WPILib blue-origin field frame. */
    public static double hubTargetXMeters() {
        return isRedAlliance() ? kFieldLengthXMeters - kHubFieldXMeters : kHubFieldXMeters;
    }

    /** Active alliance hub Y (m) in the WPILib blue-origin field frame. */
    public static double hubTargetYMeters() {
        return isRedAlliance() ? kFieldWidthYMeters - kHubFieldYMeters : kHubFieldYMeters;
    }

    private static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

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
