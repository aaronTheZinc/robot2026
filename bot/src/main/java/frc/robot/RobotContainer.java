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
    /**
     * Extra field heading (deg) on top of hub bearing + calibration for PathPlanner {@code Point and Shoot Depo}
     * only (e.g. depo-grab auto).
     */
    private static final double kDepoGrabPointAndShootExtraHeadingDeg = -28.0;
    private static final double kShotRampSeconds = 1.0;
    private static final double kAutoShotFeedSeconds = 2;
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
    /** L3: sample hub heading offset at current distance; scaled offset = cal × (d / d_cal). */
    private final HubAlignCalibration hubAlignCalibration = new HubAlignCalibration();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subsystems = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final HubAlignCommands hubAlignCommands =
            new HubAlignCommands(drivetrain, hubAlignCalibration);
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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Center Shoot", getKnnInferredAutoShotSequenceCommand(false));
        NamedCommands.registerCommand("Base Shoot", getKnnInferredAutoShotSequenceCommand(false));
        NamedCommands.registerCommand("Wing Shot", getKnnInferredAutoShotSequenceCommand(true));
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

        AutoDiagnostics.publishRegisteredNamedCommands(
                "Center Shoot, Base Shoot, Wing Shot, Intake Sequence, Intake 6s, Back Translation, Point and Shoot, "
                        + "Point and Shoot Depo, Heading 0, roller-in, Pivot Home Stow, Pivot Home Down, Pivot To Stow, Pivot To Collect");

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
                "KNN: A hold = inferred shot (hood up only during shot) | R3 = RPM | hood stowed when idle");
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
        return getKnnInferredAutoShotSequenceCommand(false);
    }

    /** PathPlanner named command: rotate in place toward hub, then same KNN-inferred ramp/feed as Center Shoot. */
    private Command getPointAndShootCommand() {
        return Commands.sequence(
                        hubAlignCommands.getRotateToHubInPlaceCommand(),
                        getKnnInferredAutoShotSequenceCommand(false))
                .withName("Point and Shoot");
    }

    /**
     * PathPlanner named command for depo auto only: aim {@link #kDepoGrabPointAndShootExtraHeadingDeg}° off default
     * hub bearing, then same KNN shot sequence as {@link #getPointAndShootCommand()}.
     */
    private Command getDepoPointAndShootCommand() {
        return Commands.sequence(
                        hubAlignCommands.getRotateToHubInPlaceCommand(kDepoGrabPointAndShootExtraHeadingDeg),
                        getKnnInferredAutoShotSequenceCommand(false))
                .withName("Point and Shoot Depo");
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
        shooter.clearKnnHoodManualTuneBlock();
        shooter.setDashboardSetpointControlEnabled(false);
        shooter.setHoodAngle(knnInterpreter.getSmoothedInterpolatedHoodDeg());
        shooter.setShooterRpm(knnInterpreter.getInterpolatedRpm());
    }

    /**
     * PathPlanner timed shot using IDW hood/RPM from pose when {@code knn_map.json} has points; otherwise
     * fixed wing or center profile ({@code wingFallback}).
     */
    private Command getKnnInferredAutoShotSequenceCommand(boolean wingFallbackWhenNoMap) {
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
                        return getAutoShotSequenceCommand(wingFallbackWhenNoMap);
                    }
                    return getKnnMapAutoShotLiveSequenceCommand();
                },
                Set.of(shooter, intake));
    }

    /**
     * Auto shot ramp + feed with hood/RPM updated every cycle from IDW (smoothed hood). Same timing as
     * fixed-profile autos; {@link ShooterCommands#getRunShotProfileCommand} cleanup runs on shot end.
     */
    private Command getKnnMapAutoShotLiveSequenceCommand() {
        Command rampShot = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
        Command feedShot = Commands.run(this::applyLiveKnnShootingSetpoints, shooter);
        return Commands.sequence(
                        intakeCommands.getStopHopperCommand(),
                        Commands.runOnce(
                                () -> {
                                    knnInterpreter.update(drivetrain.getState().Pose);
                                    knnInterpreter.snapSmoothedHoodToInterpolated();
                                    knnInterpreter.snapSmoothedRpmToInterpolated();
                                }),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds), rampShot),
                        Commands.deadline(
                                Commands.waitSeconds(kAutoShotFeedSeconds),
                                Commands.parallel(
                                        feedShot, intakeCommands.getFeedToShooterCommand())))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            shooter.stopShooter();
                            shooter.stowHoodAndSyncDashboardAfterProfile();
                            shooter.setDashboardSetpointControlEnabled(true);
                        });
    }

    private Command getAutoShotSequenceCommand(boolean wingShot) {
        Command rampShot =
                wingShot
                        ? shooterCommands.getRunWingShotHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        Command feedShot =
                wingShot
                        ? shooterCommands.getRunWingShotHoldCommand()
                        : shooterCommands.getRunCenterShotHoldCommand();
        return Commands.sequence(
                        intakeCommands.getStopHopperCommand(),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds),
                                rampShot),
                        Commands.deadline(
                                Commands.waitSeconds(kAutoShotFeedSeconds),
                                Commands.parallel(
                                        feedShot,
                                        intakeCommands.getFeedToShooterCommand())))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            shooter.stopShooter();
                            shooter.stowHoodAndSyncDashboardAfterProfile();
                            shooter.setDashboardSetpointControlEnabled(true);
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
                        wingShot
                                ? Commands.runOnce(
                                        () -> shooter.setKnnHoodBlockedAfterManualTune(true), shooter)
                                : Commands.none(),
                        intakeCommands.getStopHopperCommand(),
                        Commands.deadline(
                                Commands.waitSeconds(kShotRampSeconds),
                                rampShot),
                        Commands.parallel(
                                feedShot,
                                intakeCommands.getFeedToShooterCommand()))
                .finallyDo(
                        () -> {
                            intake.stopHopper();
                            shooter.stopShooter();
                            shooter.stowHoodAndSyncDashboardAfterProfile();
                            shooter.setDashboardSetpointControlEnabled(true);
                            if (wingShot) {
                                shooter.clearKnnHoodManualTuneBlock();
                            }
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
                                    intakeCommands.getStopHopperCommand(),
                                    Commands.runOnce(
                                            () -> {
                                                knnInterpreter.update(drivetrain.getState().Pose);
                                                knnInterpreter.snapSmoothedHoodToInterpolated();
                                                knnInterpreter.snapSmoothedRpmToInterpolated();
                                            }),
                                    Commands.deadline(
                                            Commands.waitSeconds(kShotRampSeconds),
                                            Commands.run(this::applyLiveKnnShootingSetpoints, shooter)),
                                    Commands.parallel(
                                            Commands.run(this::applyLiveKnnShootingSetpoints, shooter),
                                            intakeCommands.getFeedToShooterCommand()))
                            .finallyDo(
                                    () -> {
                                        intake.stopHopper();
                                        shooter.stopShooter();
                                        shooter.stowHoodAndSyncDashboardAfterProfile();
                                        shooter.setDashboardSetpointControlEnabled(true);
                                    })
                            .withName("KNN held shot (inferred)");
                },
                Set.of(shooter, intake));
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

        // Hold: translate with left stick; rotate toward hub bearing + distance-scaled offset (DriveConstants).
        joystick.rightBumper()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .whileTrue(
                        drivetrain.applyRequest(
                                () -> {
                                    var pose = drivetrain.getState().Pose;
                                    double offsetDeg = hubAlignCalibration.getScaledOffsetDeg(pose);
                                    return driveHubAlign
                                            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                            .withRotationalRate(
                                                    DriveConstants.teleopOmegaTowardHub(
                                                            pose, offsetDeg));
                                })
                                .withName("Hub align (hold)"));

        // L3 (left stick press): at correct aim, record offset vs shot-map hub heading at this distance (see Hub Align/*).
        joystick.leftStick()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        hubAlignCalibration.recordAtCurrentPose(
                                                drivetrain.getState().Pose)));

        // Start + L3: clear hub-offset calibration.
        joystick.start()
                .and(joystick.leftStick())
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .onTrue(Commands.runOnce(hubAlignCalibration::clear));

        // R3 (driver): snap hood smoothing and apply smoothed IDW hood + RPM once.
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
                                    shooter.clearKnnHoodManualTuneBlock();
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

        // A (hold): ramp shooter then hopper — IDW inferred hood/RPM from current pose; runs until release.
        subsystems.a()
                .and(new Trigger(DriverStation::isTeleopEnabled))
                .whileTrue(getKnnHeldShotFromInferredCommand());
        // Back: center held shot (moved from A).
        subsystems.back().whileTrue(getCenterHeldShotSequenceCommand());
        subsystems.b().whileTrue(getWingHeldShotSequenceCommand());

        // Intake pivot manual: Y = stow (up) while held, X = collect (down) while held.
        // Use Start+Y for homing to mechanical stow stop to avoid overlap with shot button B.
        subsystems.y().and(subsystems.b().negate()).whileTrue(intakeCommands.getPivotTowardStowManualCommand());
        subsystems.x().whileTrue(intakeCommands.getPivotTowardCollectManualCommand());
        subsystems.start().and(subsystems.y()).onTrue(intakeCommands.getPivotHomingCommand());

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
     * When {@link frc.robot.knn.KnnConstants#kInterpolateHoodWhileDriving} is true, sets hood from smoothed IDW
     * each loop after commands, unless the operator moved the hood manually (subsystem POV); then skips until
     * {@link ShooterSubsystem#clearKnnHoodManualTuneBlock()} runs from a shot or R3 snap. Skips while another
     * command requires the shooter. Re-enable dashboard slider ownership when interpolation turns off.
     * Call from {@code Robot.robotPeriodic} after {@code CommandScheduler.run()}; then call
     * {@link ShooterSubsystem#applyHoodMotorClosedLoopTick()}.
     */
    public void applyKnnHoodInterpolation() {
        boolean interp = knnInterpreter.isInterpolateEnabled() && knnInterpreter.getMapSize() > 0;
        if (interp) {
            if (CommandScheduler.getInstance().requiring(shooter) == null) {
                shooter.setDashboardSetpointControlEnabled(false);
                if (!shooter.isKnnHoodBlockedAfterManualTune()) {
                    shooter.setHoodAngle(knnInterpreter.getSmoothedInterpolatedHoodDeg());
                }
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

    /** SmartDashboard {@code Hub Align/*} — offset, distance, scale (deg/m), heading error. */
    public void publishHubAlignTelemetry() {
        hubAlignCalibration.publishTelemetry(drivetrain.getState().Pose);
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

    /** Intake pivot homing for real robot; schedule when enabling (auto or teleop). */
    public Command getPivotHomingCommand() {
        return intakeCommands.getPivotHomingCommand();
    }

    /** Intake pivot homing to collect (down) stop; encoder zero at deployed, like hood homing. */
    public Command getPivotDownHomingCommand() {
        return intakeCommands.getPivotDownHomingCommand();
    }
}
