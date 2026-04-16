// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.Set;

import frc.robot.commands.DriveCommands;
import frc.robot.commands.HubAlignCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.knn.KnnInterpreter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionMeasurement;

public class RobotContainer {
    private static final double kAutoIntakeSequenceSeconds = 1.5;
    /** PathPlanner {@code Intake 6s} named command duration. */
    private static final double kAutoIntakeSixSeconds = 6.0;
    private static final String kDriveAssistTargetXKey = "Drive Assist/Target X (m)";
    private static final String kDriveAssistTargetYKey = "Drive Assist/Target Y (m)";
    private static final String kDriveAssistTargetHeadingKey = "Drive Assist/Target Heading (deg)";
    /** Robot-relative reverse translation speed (m/s) for named auto / driver assist. */
    private static final double kBackTranslationMps =
            0.45 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kNamedBackTranslationSeconds = 2.0;
    private static final double kShotRampSeconds = 1.0;
    private static final double kAutoShotFeedSeconds = 2;
    /** Feed / hopper window for {@code straight-back.auto} (longer than default autos). */
    private static final double kStraightBackAutoShotFeedSeconds = 3.5;
    /** Driver Xbox rumble (0–1) while Limelight reports a tag lock ({@code tv}). */
    private static final double kDriverTagLockRumble = 0.4;
    /** Same as Phoenix SwerveWithPathPlanner example: full kSpeedAt12Volts top speed. */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    /** Same as {@link #drive} but no rotational deadband so small hub-align omega is not zeroed. */
    private final SwerveRequest.FieldCentric driveHubAlign = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final KnnInterpreter knnInterpreter = new KnnInterpreter();
    /** Tracks {@link frc.robot.knn.KnnConstants#kInterpolateHoodWhileDriving} so we re-enable dashboard sliders when it is false. */
    private boolean knnInterpolationWasEnabled = false;
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subsystems = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final HubAlignCommands hubAlignCommands =
            new HubAlignCommands(drivetrain);
    private final DriveCommands driveCommands = new DriveCommands(drivetrain, kBackTranslationMps);
    /**
     * Declared after {@code drivetrain} so periodic runs after swerve odometry; publishes {@code Pose/robotPose}
     * from fused {@code getState().Pose} when tags are valid (teleop + auto).
     */
    public final VisionMeasurement visionMeasurement = new VisionMeasurement(drivetrain);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterCommands shooterCommands = new ShooterCommands(shooter);
    private final IntakeCommands intakeCommands = new IntakeCommands(intake);

    /**
     * Once true, {@link #getIntakeDownPulseOncePerSessionCommand()} never runs the homing pulse again for this JVM
     * session (first auto or teleop enable wins). Never reset.
     */
    private boolean intakeDownHomingPulseConsumed;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Center Shoot", getHubAlignThenKnnInferredAutoShotCommand(false));
        NamedCommands.registerCommand("Base Shoot", getHubAlignThenKnnInferredAutoShotCommand(false));
        NamedCommands.registerCommand("Wing Shot", getHubAlignThenKnnInferredAutoShotCommand(true));
        NamedCommands.registerCommand(
                "Straight Back Wing Shot",
                getHubAlignThenKnnInferredAutoShotCommand(true, kStraightBackAutoShotFeedSeconds));
        NamedCommands.registerCommand(
            "Intake Sequence",
            Commands.deadline(
                Commands.waitSeconds(kAutoIntakeSequenceSeconds),
                intakeCommands.getIntakeCommand()
            )
        );
        NamedCommands.registerCommand(
            "Intake 6s",
            Commands.deadline(
                    Commands.waitSeconds(kAutoIntakeSixSeconds), intakeCommands.getIntakeCommand())
                    .withName("Intake 6s"));
        NamedCommands.registerCommand(
            "Back Translation",
            driveCommands
                    .getBackTranslationForSeconds(kNamedBackTranslationSeconds)
                    .finallyDo(() -> drivetrain.setControl(brake)));
        NamedCommands.registerCommand("Point and Shoot", getPointAndShootCommand());
        NamedCommands.registerCommand("Point and Shoot Depo", getDepoPointAndShootCommand());
        NamedCommands.registerCommand("Heading 0", getFaceFieldHeadingZeroCommand());

        NamedCommands.registerCommand("roller-in", intakeCommands.getRollerIntakeHoldCommand());
        NamedCommands.registerCommand("Pivot Home Stow", intakeCommands.getPivotHomingCommand());
        NamedCommands.registerCommand("Pivot Home Down", intakeCommands.getPivotDownHomingCommand());
        NamedCommands.registerCommand("Pivot To Stow", intakeCommands.getPivotToStowCommand());
        NamedCommands.registerCommand("Pivot To Collect", intakeCommands.getPivotToCollectCommand());
        NamedCommands.registerCommand("Intake Down", intakeCommands.getPivotTowardCollectPulseCommand());

        AutoDiagnostics.publishRegisteredNamedCommands(
                "Center Shoot, Base Shoot, Wing Shot, Straight Back Wing Shot, Intake Sequence, Intake 6s, Back Translation, Point and Shoot, "
                        + "Point and Shoot Depo, Heading 0, roller-in, Pivot Home Stow, Pivot Home Down, Pivot To Stow, "
                        + "Pivot To Collect, Intake Down");

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        autoChooser.addOption("Basic Shoot Auto", getBasicShootAutoCommand());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Debug: adjustable hood and shooter setpoints (run from SmartDashboard / robot dashboard)
        SmartDashboard.putData("Debug/Increment Hood (deg)", shooterCommands.getIncrementHoodDegreesCommand());
        SmartDashboard.putData("Debug/Decrement Hood (deg)", shooterCommands.getDecrementHoodDegreesCommand());
        SmartDashboard.putData("Debug/Increment Shooter RPM", shooterCommands.getIncrementShooterRpmCommand());
        SmartDashboard.putData("Debug/Decrement Shooter RPM", shooterCommands.getDecrementShooterRpmCommand());
        SmartDashboard.putString(
                "Drive Assist/Source",
                "KNN: hood + RPM setpoints track smoothed IDW while idle; wheels only during shot | A = inferred shot");
        SmartDashboard.putNumber(kDriveAssistTargetXKey, 0.0);
        SmartDashboard.putNumber(kDriveAssistTargetYKey, 0.0);
        SmartDashboard.putNumber(kDriveAssistTargetHeadingKey, 0.0);

        // High-rate NT / SignalLogger debug streams (DriveState, PathFollower errors, etc.); default off
        SmartDashboard.putBoolean("Debug/Verbose NT Telemetry", false);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses (matches SwerveWithPathPlanner example).
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private Command getBasicShootAutoCommand() {
        return getHubAlignThenKnnInferredAutoShotCommand(false);
    }

    /**
     * Rotate in place to hub-facing heading (same law as teleop hub align), then KNN-inferred ramp/feed.
     * Used by PathPlanner {@code Center Shoot}, {@code Base Shoot}, {@code Wing Shot}, {@code Point and Shoot},
     * and {@code Point and Shoot Depo}.
     */
    private Command getHubAlignThenKnnInferredAutoShotCommand(boolean wingFallbackWhenNoMap) {
        return getHubAlignThenKnnInferredAutoShotCommand(wingFallbackWhenNoMap, kAutoShotFeedSeconds);
    }

    private Command getHubAlignThenKnnInferredAutoShotCommand(
            boolean wingFallbackWhenNoMap, double feedSeconds) {
        return Commands.sequence(
                hubAlignCommands.getRotateToHubInPlaceCommand(),
                getKnnInferredAutoShotSequenceCommand(wingFallbackWhenNoMap, feedSeconds));
    }

    /** PathPlanner named command: hub align + same shot as {@link #getHubAlignThenKnnInferredAutoShotCommand(boolean)}. */
    private Command getPointAndShootCommand() {
        return getHubAlignThenKnnInferredAutoShotCommand(false).withName("Point and Shoot");
    }

    /** Depo auto: same hub alignment angle as teleop / {@code Point and Shoot} (no extra heading offset). */
    private Command getDepoPointAndShootCommand() {
        return getHubAlignThenKnnInferredAutoShotCommand(false).withName("Point and Shoot Depo");
    }

    /**
     * PathPlanner named command: rotate in place until fused field heading is ~0 rad (+X forward), then brake.
     * Uses per-frame {@link SwerveRequest.FieldCentric#withRotationalRate} (same pattern as hub align) so control
     * is not a single locked facing-angle setpoint.
     */
    private Command getFaceFieldHeadingZeroCommand() {
        return drivetrain
                .applyRequest(
                        () ->
                                driveHubAlign
                                        .withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(
                                                DriveConstants.teleopOmegaTowardFieldHeadingZero(
                                                        drivetrain.getState().Pose)))
                .until(() -> DriveConstants.isFieldHeadingNearZero(drivetrain.getState().Pose))
                .withTimeout(DriveConstants.kAutoFieldHeadingZeroTimeoutSeconds)
                .finallyDo(() -> drivetrain.setControl(brake))
                .withName("Heading 0");
    }

    /** Updates shooter from map: smoothed IDW hood + IDW RPM (pose from drivetrain). */
    private void applyLiveKnnShootingSetpoints() {
        knnInterpreter.update(drivetrain.getState().Pose);
        shooter.setDashboardSetpointControlEnabled(false);
        shooter.setHoodAngle(knnInterpreter.getSmoothedInterpolatedHoodDeg());
        shooter.setShooterRpm(knnInterpreter.getInterpolatedRpm());
    }

    /** After any shot sequence: stop wheels, disable flywheel gate, hand KNN back setpoint ownership. */
    private void endShotResumeKnnTracking() {
        shooter.stopShooter();
        shooter.setShootFlywheelVelocityEnabled(false);
        shooter.setDashboardSetpointControlEnabled(false);
    }

    /**
     * PathPlanner timed shot using IDW hood/RPM from pose when {@code knn_map.json} has points; otherwise
     * fixed wing or center profile ({@code wingFallback}).
     */
    private Command getKnnInferredAutoShotSequenceCommand(boolean wingFallbackWhenNoMap, double feedSeconds) {
        return Commands.defer(
                () -> {
                    Pose2d here = drivetrain.getState().Pose;
                    knnInterpreter.update(here);
                    if (knnInterpreter.getMapSize() == 0) {
                        DriverStation.reportWarning(
                                "KNN: knn_map.json missing or empty — auto using fixed "
                                        + (wingFallbackWhenNoMap ? "wing" : "center")
                                        + " shot",
                                false);
                        return getAutoShotSequenceCommand(wingFallbackWhenNoMap, feedSeconds);
                    }
                    return getKnnMapAutoShotLiveSequenceCommand(feedSeconds);
                },
                Set.of(shooter, intake));
    }

    /**
     * Auto shot ramp + feed with hood/RPM updated every cycle from IDW (smoothed hood). Same timing as
     * fixed-profile autos; {@link ShooterCommands#getRunShotProfileCommand} cleanup runs on shot end.
     */
    private Command getKnnMapAutoShotLiveSequenceCommand(double feedSeconds) {
        Command rampShot = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
        Command feedShot = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
        return Commands.sequence(
                        Commands.runOnce(() -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds),
                                Commands.parallel(
                                        intakeCommands.getStopHopperCommand(),
                                        Commands.sequence(
                                                Commands.runOnce(
                                                        () -> {
                                                            knnInterpreter.update(
                                                                    drivetrain.getState().Pose);
                                                            knnInterpreter
                                                                    .snapSmoothedHoodToInterpolated();
                                                            knnInterpreter
                                                                    .snapSmoothedRpmToInterpolated();
                                                        }),
                                                rampShot))),
                        Commands.deadline(
                                Commands.waitSeconds(feedSeconds),
                                Commands.parallel(
                                        feedShot,
                                        intakeCommands.getFeedToShooterWithRollerCommand())))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            endShotResumeKnnTracking();
                        });
    }

    private Command getAutoShotSequenceCommand(boolean wingShot, double feedSeconds) {
        Command rampShot =
                wingShot
                        ? shooterCommands.getRunWingShotHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        Command feedShot =
                wingShot
                        ? shooterCommands.getRunWingShotHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        return Commands.sequence(
                        Commands.runOnce(() -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds),
                                Commands.parallel(
                                        intakeCommands.getStopHopperCommand(), rampShot)),
                        Commands.deadline(
                                Commands.waitSeconds(feedSeconds),
                                Commands.parallel(
                                        feedShot,
                                        intakeCommands.getFeedToShooterWithRollerCommand())))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            endShotResumeKnnTracking();
                        });
    }

    private Command getHeldShotSequenceCommand(boolean wingShot) {
        Command rampShot =
                wingShot
                        ? shooterCommands.getRunWingShotWithManualHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        Command feedShot =
                wingShot
                        ? shooterCommands.getRunWingShotWithManualHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        return Commands.sequence(
                        Commands.runOnce(() -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds),
                                Commands.parallel(
                                        intakeCommands.getStopHopperCommand(),
                                        rampShot)),
                        Commands.parallel(
                                feedShot,
                                intakeCommands.getFeedToShooterWithRollerCommand()))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            endShotResumeKnnTracking();
                        });
    }

    private Command getCenterHeldShotSequenceCommand() {
        return getHeldShotSequenceCommand(false);
    }

    private Command getWingHeldShotSequenceCommand() {
        return getHeldShotSequenceCommand(true);
    }

    /**
     * Held shot using IDW-inferred hood/RPM from the current pose (same inference as dashboard / R3).
     * Refreshes {@link KnnInterpreter} for the live pose when the command starts.
     */
    private Command getKnnHeldShotFromInferredCommand() {
        return Commands.defer(
                () -> {
                    if (knnInterpreter.getMapSize() == 0) {
                        DriverStation.reportWarning("KNN: knn_map.json missing or has no points", false);
                        return Commands.none();
                    }
                    return Commands.sequence(
                                    Commands.runOnce(() -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                                    Commands.deadline(
                                            Commands.waitSeconds(kShotRampSeconds),
                                            Commands.parallel(
                                                    intakeCommands.getStopHopperCommand(),
                                                    Commands.sequence(
                                                            Commands.runOnce(
                                                                    () -> {
                                                                        knnInterpreter.update(
                                                                                drivetrain
                                                                                        .getState()
                                                                                        .Pose);
                                                                        knnInterpreter
                                                                                .snapSmoothedHoodToInterpolated();
                                                                        knnInterpreter
                                                                                .snapSmoothedRpmToInterpolated();
                                                                    }),
                                                            Commands.run(
                                                                    this::applyLiveKnnShootingSetpoints,
                                                                    shooter)))),
                                    Commands.parallel(
                                            Commands.run(this::applyLiveKnnShootingSetpoints, shooter),
                                            intakeCommands.getFeedToShooterWithRollerCommand()))
                            .finallyDo(
                                    () -> {
                                        intake.stopHopper();
                                        endShotResumeKnnTracking();
                                    })
                            .withName("KNN held shot (inferred)");
                },
                Set.of(shooter, intake));
    }

    /**
     * Like {@link #getKnnHeldShotFromInferredCommand()} but rotates toward the hub in place during the ramp window,
     * then feeds (same as inferred held shot).
     */
    private Command getKnnHeldShotAlignHubDuringRampCommand() {
        return Commands.defer(
                () -> {
                    if (knnInterpreter.getMapSize() == 0) {
                        DriverStation.reportWarning("KNN: knn_map.json missing or has no points", false);
                        return Commands.none();
                    }
                    Command rampKnn =
                            Commands.sequence(
                                    Commands.runOnce(
                                            () -> {
                                                knnInterpreter.update(drivetrain.getState().Pose);
                                                knnInterpreter.snapSmoothedHoodToInterpolated();
                                                knnInterpreter.snapSmoothedRpmToInterpolated();
                                            }),
                                    Commands.run(this::applyLiveKnnShootingSetpoints, shooter));
                    return Commands.sequence(
                                    Commands.runOnce(
                                            () -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                                    Commands.deadline(
                                            Commands.waitSeconds(kShotRampSeconds),
                                            Commands.parallel(
                                                    intakeCommands.getStopHopperCommand(),
                                                    hubAlignCommands.getRotateToHubInPlaceCommand(),
                                                    rampKnn)),
                                    Commands.parallel(
                                            Commands.run(this::applyLiveKnnShootingSetpoints, shooter),
                                            intakeCommands.getFeedToShooterWithRollerCommand()))
                            .finallyDo(
                                    () -> {
                                        intake.stopHopper();
                                        endShotResumeKnnTracking();
                                    })
                                    .withName("KNN held shot (align hub during ramp)");
                },
                Set.of(shooter, intake, drivetrain));
    }

    /**
     * Same field-centric translation + hub omega as right bumper; each call returns a new command (safe to compose
     * in sequence/deadline).
     */
    private Command getTeleopHubAlignDriveCommand() {
        return drivetrain.applyRequest(
                () -> {
                    var pose = drivetrain.getState().Pose;
                    return driveHubAlign
                            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                            .withRotationalRate(
                                    DriveConstants.teleopOmegaTowardHub(pose, 0.0));
                });
    }

    /**
     * KNN held shot with hub alignment during ramp and feed: left stick translates, robot rotates toward hub
     * (second phase of operator X after {@link HubAlignCommands#getRotateToHubInPlaceCommand()}).
     */
    private Command getKnnHeldShotWithHubAlignCommand() {
        return Commands.defer(
                () -> {
                    if (knnInterpreter.getMapSize() == 0) {
                        DriverStation.reportWarning("KNN: knn_map.json missing or has no points", false);
                        return Commands.none();
                    }
                    Command rampKnn = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
                    Command feedKnn = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
                    return Commands.sequence(
                                    Commands.runOnce(() -> shooter.setShootFlywheelVelocityEnabled(true), shooter),
                                    Commands.deadline(
                                            Commands.waitSeconds(kShotRampSeconds),
                                            Commands.parallel(
                                                    intakeCommands.getStopHopperCommand(),
                                                    Commands.sequence(
                                                            Commands.runOnce(
                                                                    () -> {
                                                                        knnInterpreter.update(
                                                                                drivetrain
                                                                                        .getState()
                                                                                        .Pose);
                                                                        knnInterpreter
                                                                                .snapSmoothedHoodToInterpolated();
                                                                        knnInterpreter
                                                                                .snapSmoothedRpmToInterpolated();
                                                                    }),
                                                            Commands.parallel(
                                                                    getTeleopHubAlignDriveCommand(),
                                                                    rampKnn)))),
                                    Commands.parallel(
                                            getTeleopHubAlignDriveCommand(),
                                            feedKnn,
                                            intakeCommands.getFeedToShooterWithRollerCommand()))
                            .finallyDo(
                                    () -> {
                                        intake.stopHopper();
                                        endShotResumeKnnTracking();
                                    })
                                    .withName("KNN held shot (hub align)");
                },
                Set.of(shooter, intake, drivetrain));
    }

    /**
     * Operator X: same in-place hub facing as PathPlanner {@code Point and Shoot} ({@link
     * HubAlignCommands#getRotateToHubInPlaceCommand()}), then KNN ramp/feed with driver-stick hub assist.
     */
    private Command getPointToHubThenKnnHeldShotWithHubAlignCommand() {
        return Commands.sequence(
                        hubAlignCommands.getRotateToHubInPlaceCommand(),
                        getKnnHeldShotWithHubAlignCommand())
                .withName("Point to hub then KNN shot (hub align)");
    }

    private Pose2d getDriveAssistTargetPose() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = knnInterpreter.getNearestPose(currentPose).orElse(currentPose);
        SmartDashboard.putNumber(kDriveAssistTargetXKey, targetPose.getX());
        SmartDashboard.putNumber(kDriveAssistTargetYKey, targetPose.getY());
        SmartDashboard.putNumber(kDriveAssistTargetHeadingKey, targetPose.getRotation().getDegrees());
        return targetPose;
    }

    private Command getDriveAssistCommand() {
        return drivetrain.pathfindToPose(this::getDriveAssistTargetPose)
            .andThen(drivetrain.applyRequest(() -> brake));
    }

    public void flipDriveDirection() {
            Pose2d currentPose = drivetrain.getState().Pose;
            drivetrain.resetPose(new Pose2d(
                currentPose.getTranslation(),
                currentPose.getRotation().plus(Rotation2d.k180deg)
            ));
    }

    private void configureBindings() {
        // Chassis: match Phoenix SwerveWithPathPlanner (field-centric stick signs + bindings).
        // Note that X is forward and Y is left per WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // forward
                                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // left
                                        .withRotationalRate(
                                                DriveConstants.kTeleopRotationStickSign
                                                        * joystick.getRightX()
                                                        * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(
                                                        -joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.povUp()
                .whileTrue(
                        drivetrain.applyRequest(
                                () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.povDown()
                .whileTrue(
                        drivetrain.applyRequest(
                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading (matches example: left bumper only).
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Hold: translate with left stick; rotate toward the true hub center.
        joystick.rightBumper()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .whileTrue(getTeleopHubAlignDriveCommand().withName("Hub align (hold)"));

        // R3 (driver) hold: same hub-facing assist as right bumper. (onTrue below = one-shot KNN snap on press.)
        joystick.rightStick()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .whileTrue(getTeleopHubAlignDriveCommand().withName("Hub align (R3 hold)"));

        // R3 (driver) press: snap hood smoothing and apply smoothed IDW hood + RPM once.
        joystick.rightStick()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    if (knnInterpreter.getMapSize() == 0) {
                                        return;
                                    }
                                    knnInterpreter.update(drivetrain.getState().Pose);
                                    knnInterpreter.snapSmoothedHoodToInterpolated();
                                    knnInterpreter.snapSmoothedRpmToInterpolated();
                                    shooter.setDashboardSetpointControlEnabled(false);
                                    shooter.setHoodAngle(knnInterpreter.getSmoothedInterpolatedHoodDeg());
                                    shooter.setShooterRpm(knnInterpreter.getInterpolatedRpm());
                                },
                                shooter));

        // Subsystems controller (1): triggers, bumpers, buttons, D-pad
        // R2: reverse shooter while held
        subsystems.rightTrigger().whileTrue(shooterCommands.getReverseShooterCommand());
        // L1: collect with intake while held
        subsystems.leftBumper().whileTrue(intakeCommands.getIntakeCommand());
        // L2: reverse intake while held
        subsystems.leftTrigger().whileTrue(intakeCommands.getSpitOutCommand());
        // R3 (right stick): manual hopper feed while held
        subsystems.rightStick().whileTrue(intakeCommands.getFeedToShooterCommand());

        var teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
        // A (hold): face hub while KNN ramp, then feed. L3: KNN inferred only (no in-place hub rotate).
        subsystems.a().and(teleopEnabled).whileTrue(getKnnHeldShotAlignHubDuringRampCommand());
        subsystems.leftStick().and(teleopEnabled).whileTrue(getKnnHeldShotFromInferredCommand());
        // Back: center held shot (moved from A).
        subsystems.back().whileTrue(getCenterHeldShotSequenceCommand());
        subsystems.b().whileTrue(getWingHeldShotSequenceCommand());

        // Intake pivot: X/Y = manual open-loop toward collect / stow ({@link IntakeConstants#kPivotCollectSpeed} /
        // {@link IntakeConstants#kPivotStowSpeed}). Encoder PID snap (pivotReady) is unused while homing is off.
        // Previous: getPivotToCollectCommand / getPivotToStowCommand.
        subsystems.x()
                .and(teleopEnabled)
                .whileTrue(intakeCommands.getPivotTowardCollectManualCommand());
        subsystems.y()
                .and(teleopEnabled)
                .whileTrue(intakeCommands.getPivotTowardStowManualCommand());
        // R1 without Start: aim + KNN held shot. (Start+R1 reserved for pivot homing when enabled.)
        subsystems.rightBumper()
                .and(teleopEnabled)
                .and(subsystems.start().negate())
                .whileTrue(getPointToHubThenKnnHeldShotWithHubAlignCommand());
        if (IntakeConstants.kEnableIntakePivotHomingAtEnable) {
            subsystems.start().and(subsystems.rightBumper()).onTrue(intakeCommands.getPivotHomingCommand());
        }

        // D-pad: hood angle (while held = repeat); latch keeps setpoint after release vs stow-at-min.
        subsystems.povUp().whileTrue(shooterCommands.getAdjustHoodUpWhileHeldCommand());
        subsystems.povDown().whileTrue(shooterCommands.getAdjustHoodDownWhileHeldCommand());
        subsystems.povLeft().onTrue(shooterCommands.getDecrementShooterRpmCommand());
        subsystems.povRight().onTrue(shooterCommands.getIncrementShooterRpmCommand());

        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            knnInterpreter.update(state.Pose);
        });
    }

    /**
     * When {@link frc.robot.knn.KnnConstants#kInterpolateHoodWhileDriving} is true, sets hood and RPM setpoints
     * from smoothed IDW each loop after commands. Skips while another command requires the shooter (shot
     * profiles own setpoints). Flywheels only spin when {@link ShooterSubsystem#setShootFlywheelVelocityEnabled}
     * is true during a shot sequence.
     * Re-enable dashboard slider ownership when interpolation turns off.
     * Call from {@code Robot.robotPeriodic} after {@code CommandScheduler.run()}; then call
     * {@link ShooterSubsystem#applyHoodMotorClosedLoopTick()}.
     */
    public void applyKnnHoodInterpolation() {
        boolean interp = knnInterpreter.isInterpolateEnabled() && knnInterpreter.getMapSize() > 0;
        if (interp) {
            if (CommandScheduler.getInstance().requiring(shooter) == null) {
                shooter.setDashboardSetpointControlEnabled(false);
                shooter.setHoodAngle(knnInterpreter.getSmoothedInterpolatedHoodDeg());
                shooter.setShooterRpm(knnInterpreter.getInterpolatedRpm());
            }
        } else if (knnInterpolationWasEnabled) {
            shooter.setDashboardSetpointControlEnabled(true);
        }
        knnInterpolationWasEnabled = interp;
    }

    /** Call from {@code Robot.robotPeriodic} after {@code CommandScheduler.run()} for driver feedback. */
    public void updateDriverRumble() {
        double v = 0.0;
        if (DriverStation.isTeleopEnabled() && visionMeasurement.limelightHasTagLock()) {
            v = kDriverTagLockRumble;
        }
        var hid = joystick.getHID();
        hid.setRumble(GenericHID.RumbleType.kLeftRumble, v);
        hid.setRumble(GenericHID.RumbleType.kRightRumble, v);
    }

    public void clearDriverRumble() {
        var hid = joystick.getHID();
        hid.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        hid.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }

    /** SmartDashboard {@code Hub Align/*} — distance and heading error vs shot-map hub aim (dashboard model). */
    public void publishHubAlignTelemetry() {
        var pose = drivetrain.getState().Pose;
        SmartDashboard.putNumber(
                "Hub Align/Distance to hub (m)",
                HubAlignCalibration.distanceToHubMeters(pose));
        double errDeg =
                Math.toDegrees(
                        edu.wpi.first.math.MathUtil.angleModulus(
                                DriveConstants.rotationToFaceHubFromShotMap(pose).getRadians()
                                        - pose.getRotation().getRadians()));
        SmartDashboard.putNumber("Hub Align/Heading error (deg)", errDeg);
        double geometricDeg =
                Math.toDegrees(
                        edu.wpi.first.math.MathUtil.angleModulus(
                                DriveConstants.rotationToFaceHub(pose).getRadians()
                                        - DriveConstants.rotationToFaceHubFromShotMap(pose).getRadians()));
        SmartDashboard.putNumber("Hub Align/Shot map offset vs geometric (deg)", geometricDeg);
        SmartDashboard.putNumber("Hub Align/Offset applied (deg)", 0.0);
        SmartDashboard.putBoolean("Hub Align/Calibrated", false);
    }

    /**
     * Before autonomous runs: reset fused odometry to the PathPlanner routine’s start pose when the selected
     * command is a {@link PathPlannerAuto} with a defined start. Does not depend on Limelight or teleop so
     * auto can run with wheel odometry only.
     */
    public void seedOdometryForAuto(Command selectedAuto) {
        SmartDashboard.putBoolean("Auto/Pose seeded this enable", false);
        Pose2d start = null;
        if (selectedAuto instanceof PathPlannerAuto ppa) {
            start = ppa.getStartingPose();
        }
        if (start == null) {
            return;
        }
        drivetrain.resetPose(start);
        AutoDiagnostics.logPoseReset(start);
        SmartDashboard.putBoolean("Auto/Pose seeded this enable", true);
        SmartDashboard.putNumber("Auto/Seed pose X (m)", start.getX());
        SmartDashboard.putNumber("Auto/Seed pose Y (m)", start.getY());
        SmartDashboard.putNumber("Auto/Seed pose heading (deg)", start.getRotation().getDegrees());
    }

    /** Same as SwerveWithPathPlanner: chooser selection only. */
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        AutoDiagnostics.publishChooserAutoSelection(selected);
        AutoDiagnostics.publishResolvedAutonomous(selected, false);
        return selected;
    }

    public ShooterSubsystem getShooter() {
        return shooter;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    /** Hood homing for real robot; schedule when enabling (auto or teleop). */
    public Command getHoodHomingCommand() {
        return shooterCommands.getHoodHomingCommand();
    }

    /**
     * Hood stall homing with {@link IntakeCommands#getPivotTowardCollectPulseCommand()} in parallel; the pulse uses
     * {@link #getIntakeDownPulseOncePerSessionCommand()} so it runs at most once for the whole run (auto or teleop,
     * whichever schedules homing first). When {@link IntakeConstants#kEnableIntakePivotHomingAtEnable} is true, full
     * pivot homing runs after hood finishes.
     */
    public Command getEnableHomingSequenceCommand() {
        Command hoodHoming = shooterCommands.getHoodHomingCommand();
        Command intakeDownOnce = getIntakeDownPulseOncePerSessionCommand();
        Command hoodWithIntake =
                Commands.parallel(hoodHoming, intakeDownOnce).withName("HoodHomingWithIntakeDown");
        if (IntakeConstants.kEnableIntakePivotHomingAtEnable) {
            return hoodWithIntake.andThen(getPivotHomingCommand()).withName("HoodThenPivotHoming");
        }
        return hoodWithIntake;
    }

    /**
     * One-shot for the robot session: same pivot pulse as PathPlanner {@code Intake Down}, but after the first run
     * this becomes a no-op until code redeploy / power cycle. Path markers should use {@code Intake Down} for
     * repeatable pulses.
     */
    public Command getIntakeDownPulseOncePerSessionCommand() {
        return new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> intakeDownHomingPulseConsumed = true),
                                intakeCommands.getPivotTowardCollectPulseCommand()),
                        Commands.none(),
                        () -> !intakeDownHomingPulseConsumed)
                .withName("IntakeDownOncePerSession");
    }

    /** Intake pivot homing for real robot; schedule when enabling (auto or teleop). */
    public Command getPivotHomingCommand() {
        return intakeCommands.getPivotHomingCommand();
    }

    /** Intake pivot homing to collect (down) stop; encoder zero at deployed, like hood homing. */
    public Command getPivotDownHomingCommand() {
        return intakeCommands.getPivotDownHomingCommand();
    }
}
