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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.DriveCommands;
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
    private static final String kDriveAssistTargetXKey = "Drive Assist/Target X (m)";
    private static final String kDriveAssistTargetYKey = "Drive Assist/Target Y (m)";
    private static final String kDriveAssistTargetHeadingKey = "Drive Assist/Target Heading (deg)";
    /** Robot-relative reverse translation speed (m/s) for named auto / driver assist. */
    private static final double kBackTranslationMps =
            0.45 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kNamedBackTranslationSeconds = 2.0;
    private static final double kShotRampSeconds = 1.0;
    private static final double kAutoShotFeedSeconds = 2;
    /** Same as Phoenix SwerveWithPathPlanner example: full kSpeedAt12Volts top speed. */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final KnnInterpreter knnInterpreter = new KnnInterpreter();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subsystems = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final DriveCommands driveCommands = new DriveCommands(drivetrain, kBackTranslationMps);
    /** Fuses Limelight pose estimates with drivetrain odometry when running periodically. */
    public final VisionMeasurement visionMeasurement = new VisionMeasurement(drivetrain);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterCommands shooterCommands = new ShooterCommands(shooter);
    private final IntakeCommands intakeCommands = new IntakeCommands(intake);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Center Shoot", getCenterAutoShotSequenceCommand());
        NamedCommands.registerCommand("Base Shoot", getCenterAutoShotSequenceCommand());
        NamedCommands.registerCommand("Wing Shot", getAutoShotSequenceCommand(true));
        NamedCommands.registerCommand(
            "Intake Sequence",
            Commands.deadline(
                Commands.waitSeconds(kAutoIntakeSequenceSeconds),
                intakeCommands.getIntakeCommand()
            )
        );
        NamedCommands.registerCommand(
            "Back Translation",
            driveCommands.getBackTranslationForSeconds(kNamedBackTranslationSeconds));

        AutoDiagnostics.publishRegisteredNamedCommands(
                "Center Shoot, Base Shoot, Intake Sequence, Back Translation");

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        autoChooser.addOption("Basic Shoot Auto", getBasicShootAutoCommand());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Debug: adjustable hood and shooter setpoints (run from SmartDashboard / robot dashboard)
        SmartDashboard.putData("Debug/Increment Hood (deg)", shooterCommands.getIncrementHoodDegreesCommand());
        SmartDashboard.putData("Debug/Decrement Hood (deg)", shooterCommands.getDecrementHoodDegreesCommand());
        SmartDashboard.putData("Debug/Increment Shooter RPM", shooterCommands.getIncrementShooterRpmCommand());
        SmartDashboard.putData("Debug/Decrement Shooter RPM", shooterCommands.getDecrementShooterRpmCommand());
        SmartDashboard.putString("Drive Assist/Source", "Nearest KNN map pose");
        SmartDashboard.putNumber(kDriveAssistTargetXKey, 0.0);
        SmartDashboard.putNumber(kDriveAssistTargetYKey, 0.0);
        SmartDashboard.putNumber(kDriveAssistTargetHeadingKey, 0.0);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses (matches SwerveWithPathPlanner example).
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private Command getBasicShootAutoCommand() {
        return getCenterAutoShotSequenceCommand();
    }

    private Command getAutoShotSequenceCommand(boolean wingShot) {
        Command rampShot = wingShot
            ? shooterCommands.getRunWingShotCommand()
            : shooterCommands.getRunCenterShotCommand();
        Command feedShot = wingShot
            ? shooterCommands.getRunWingShotCommand()
            : shooterCommands.getRunCenterShotCommand();
        return Commands.sequence(
            intakeCommands.getStopHopperCommand(),
            Commands.deadline(
                Commands.waitSeconds(kShotRampSeconds),
                rampShot
            ),
            Commands.deadline(
                Commands.waitSeconds(kAutoShotFeedSeconds),
                Commands.parallel(
                    feedShot,
                    intakeCommands.getFeedToShooterCommand()
                )
            )
        ).finallyDo(() -> intake.stopHopper());
    }

    private Command getHeldShotSequenceCommand(boolean wingShot) {
        Command rampShot = wingShot
            ? shooterCommands.getRunWingShotCommand()
            : shooterCommands.getRunCenterShotCommand();
        Command feedShot = wingShot
            ? shooterCommands.getRunWingShotCommand()
            : shooterCommands.getRunCenterShotCommand();
        return Commands.sequence(
            intakeCommands.getStopHopperCommand(),
            Commands.deadline(
                Commands.waitSeconds(kShotRampSeconds),
                rampShot
            ),
            Commands.parallel(
                feedShot,
                intakeCommands.getFeedToShooterCommand()
            )
        ).finallyDo(() -> intake.stopHopper());
    }

    private Command getCenterAutoShotSequenceCommand() {
        return getAutoShotSequenceCommand(false);
    }

    private Command getCenterHeldShotSequenceCommand() {
        return getHeldShotSequenceCommand(false);
    }

    private Command getWingHeldShotSequenceCommand() {
        return getHeldShotSequenceCommand(true);
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
        return Commands.parallel(
            Commands.startEnd(
                () -> visionMeasurement.setTeleopVisionFusionEnabled(true),
                () -> visionMeasurement.setTeleopVisionFusionEnabled(false),
                visionMeasurement
            ),
            drivetrain.pathfindToPose(this::getDriveAssistTargetPose)
                .andThen(drivetrain.applyRequest(() -> brake))
        );
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
                                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

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

        // Subsystems controller (1): triggers, bumpers, buttons, D-pad
        // R2: reverse shooter while held
        subsystems.rightTrigger().whileTrue(shooterCommands.getReverseShooterCommand());
        // L1: collect with intake while held
        subsystems.leftBumper().whileTrue(intakeCommands.getIntakeCommand());
        // L2: reverse intake while held
        subsystems.leftTrigger().whileTrue(intakeCommands.getSpitOutCommand());

        // A/B: run shot sequence while held (ramp then feed, stop on release).
        subsystems.a().whileTrue(getCenterHeldShotSequenceCommand());
        subsystems.b().whileTrue(getWingHeldShotSequenceCommand());

        // Intake pivot manual: Y = stow (up) while held, X = collect (down) while held.
        // Use Start+Y for homing to mechanical stow stop to avoid overlap with shot button B.
        subsystems.y().and(subsystems.b().negate()).whileTrue(intakeCommands.getPivotTowardStowManualCommand());
        subsystems.x().whileTrue(intakeCommands.getPivotTowardCollectManualCommand());
        subsystems.start().and(subsystems.y()).onTrue(intakeCommands.getPivotHomingCommand());

        subsystems.povUp().onTrue(shooterCommands.getIncrementHoodDegreesCommand());
        subsystems.povDown().onTrue(shooterCommands.getDecrementHoodDegreesCommand());
        subsystems.povLeft().onTrue(shooterCommands.getDecrementShooterRpmCommand());
        subsystems.povRight().onTrue(shooterCommands.getIncrementShooterRpmCommand());

        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            knnInterpreter.update(state.Pose);
        });
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
}
