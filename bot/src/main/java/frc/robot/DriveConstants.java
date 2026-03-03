// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}
