// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import frc.robot.DriveConstants;
import frc.robot.IntakeConstants;
import frc.robot.ShooterConstants;

/**
 * Motor test binding table and default speed. Per-motor min/max and default invert
 * are defined in ShooterConstants, IntakeConstants, and DriveConstants; this class
 * looks them up by motorId.
 */
public final class MotorTestConstants {
    private MotorTestConstants() {}

    /** Buttons used for motor test bindings (controller 0 and 1). */
    public enum TestButton {
        A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, POV_UP, POV_DOWN
    }

    /** Default test speed when no axis is provided (e.g. SmartDashboard). */
    public static final double kDefaultSpeed = 0.15;

    /** Ordered motor IDs for NT seeding and SmartDashboard. */
    private static final String[] MOTOR_IDS = {
        "shooterLeft", "shooterRight", "hood",
        "intakeRoller", "intakePivot", "hopper",
        "drive0", "drive1", "drive2", "drive3",
        "steer0", "steer1", "steer2", "steer3"
    };

    /** Display names for SmartDashboard (same order as MOTOR_IDS). */
    private static final String[] DISPLAY_NAMES = {
        "Shooter Left", "Shooter Right", "Hood",
        "Intake Roller", "Intake Pivot", "Hopper",
        "Drive 0", "Drive 1", "Drive 2", "Drive 3",
        "Steer 0", "Steer 1", "Steer 2", "Steer 3"
    };

    /** Controller 0 (joystick): button → motorId. Drive and steer. */
    private static final Map<TestButton, String> CONTROLLER_0_BINDINGS = Map.ofEntries(
        Map.entry(TestButton.A, "drive0"),
        Map.entry(TestButton.B, "drive1"),
        Map.entry(TestButton.X, "drive2"),
        Map.entry(TestButton.Y, "drive3"),
        Map.entry(TestButton.LEFT_BUMPER, "steer0"),
        Map.entry(TestButton.RIGHT_BUMPER, "steer1"),
        Map.entry(TestButton.POV_UP, "steer2"),
        Map.entry(TestButton.POV_DOWN, "steer3")
    );

    /** Controller 1 (subsystems): button → motorId. Shooter and intake. */
    private static final Map<TestButton, String> CONTROLLER_1_BINDINGS = Map.ofEntries(
        Map.entry(TestButton.A, "shooterLeft"),
        Map.entry(TestButton.B, "shooterRight"),
        Map.entry(TestButton.X, "hood"),
        Map.entry(TestButton.Y, "intakeRoller"),
        Map.entry(TestButton.LEFT_BUMPER, "intakePivot"),
        Map.entry(TestButton.RIGHT_BUMPER, "hopper")
    );

    public static double getDefaultSpeed() {
        return kDefaultSpeed;
    }

    public static double getSwerveTestMaxVolts() {
        return DriveConstants.kSwerveTestMaxVolts;
    }

    public static double getMinSpeed(String motorId) {
        return switch (motorId) {
            case "shooterLeft" -> ShooterConstants.kShooterLeftTestMinSpeed;
            case "shooterRight" -> ShooterConstants.kShooterRightTestMinSpeed;
            case "hood" -> ShooterConstants.kHoodTestMinSpeed;
            case "intakeRoller" -> IntakeConstants.kRollerTestMinSpeed;
            case "intakePivot" -> IntakeConstants.kPivotTestMinSpeed;
            case "hopper" -> IntakeConstants.kHopperTestMinSpeed;
            case "drive0", "drive1", "drive2", "drive3" -> DriveConstants.kDriveTestMinSpeed;
            case "steer0", "steer1", "steer2", "steer3" -> DriveConstants.kSteerTestMinSpeed;
            default -> -1.0;
        };
    }

    public static double getMaxSpeed(String motorId) {
        return switch (motorId) {
            case "shooterLeft" -> ShooterConstants.kShooterLeftTestMaxSpeed;
            case "shooterRight" -> ShooterConstants.kShooterRightTestMaxSpeed;
            case "hood" -> ShooterConstants.kHoodTestMaxSpeed;
            case "intakeRoller" -> IntakeConstants.kRollerTestMaxSpeed;
            case "intakePivot" -> IntakeConstants.kPivotTestMaxSpeed;
            case "hopper" -> IntakeConstants.kHopperTestMaxSpeed;
            case "drive0", "drive1", "drive2", "drive3" -> DriveConstants.kDriveTestMaxSpeed;
            case "steer0", "steer1", "steer2", "steer3" -> DriveConstants.kSteerTestMaxSpeed;
            default -> 1.0;
        };
    }

    public static boolean getDefaultInvert(String motorId) {
        return switch (motorId) {
            case "shooterLeft" -> ShooterConstants.kShooterLeftTestDefaultInvert;
            case "shooterRight" -> ShooterConstants.kShooterRightTestDefaultInvert;
            case "hood" -> ShooterConstants.kHoodTestDefaultInvert;
            case "intakeRoller" -> IntakeConstants.kRollerTestDefaultInvert;
            case "intakePivot" -> IntakeConstants.kPivotTestDefaultInvert;
            case "hopper" -> IntakeConstants.kHopperTestDefaultInvert;
            case "drive0", "drive1", "drive2", "drive3" -> DriveConstants.kDriveTestDefaultInvert;
            case "steer0", "steer1", "steer2", "steer3" -> DriveConstants.kSteerTestDefaultInvert;
            default -> false;
        };
    }

    public static List<String> getMotorIds() {
        return List.of(MOTOR_IDS);
    }

    public static String getDisplayName(String motorId) {
        for (int i = 0; i < MOTOR_IDS.length; i++) {
            if (MOTOR_IDS[i].equals(motorId)) {
                return DISPLAY_NAMES[i];
            }
        }
        return motorId;
    }

    public static Map<TestButton, String> getController0Bindings() {
        return CONTROLLER_0_BINDINGS;
    }

    public static Map<TestButton, String> getController1Bindings() {
        return CONTROLLER_1_BINDINGS;
    }
}
