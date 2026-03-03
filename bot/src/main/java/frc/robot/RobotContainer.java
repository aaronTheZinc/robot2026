// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.HoodHomingCommand;
import frc.robot.commands.test.RunMotorTestCommand;
import frc.robot.subsystems.MotorTestConstants;
import frc.robot.subsystems.MotorTestConstants.TestButton;
import frc.robot.generated.TunerConstants;
import frc.robot.knn.KnnInterpreter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorTestSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionMeasurement;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
    /** Fuses Limelight pose estimates with drivetrain odometry when running periodically. */
    public final VisionMeasurement visionMeasurement = new VisionMeasurement(drivetrain);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final MotorTestSubsystem motorTest = new MotorTestSubsystem(shooter, intake, drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        seedMotorTestNetworkTables();
        registerMotorTestCommandsOnSmartDashboard();

        configureBindings();

        // Run MotorTestSubsystem.periodic() every cycle so NT-driven motor test is applied when disabled
        new RunCommand(() -> {}, motorTest).ignoringDisable(true).schedule();
    }

    private void seedMotorTestNetworkTables() {
        var table = NetworkTableInstance.getDefault().getTable("MotorTest");
        table.getEntry("Speed").setDouble(MotorTestConstants.kDefaultSpeed);
        table.getEntry("Motor").setString("shooterLeft");
        table.getEntry("Enable").setBoolean(false);
        SmartDashboard.putNumber("MotorTest/Speed", MotorTestConstants.kDefaultSpeed);
        SmartDashboard.putString("MotorTest/Motor", "shooterLeft");
        SmartDashboard.putBoolean("MotorTest/Enable", false);
        for (String motorId : MotorTestConstants.getMotorIds()) {
            boolean defaultInvert = MotorTestConstants.getDefaultInvert(motorId);
            table.getEntry("Invert_" + motorId).setBoolean(defaultInvert);
            SmartDashboard.putBoolean("MotorTest/Invert_" + motorId, defaultInvert);
        }
    }

    /** Test speed from subsystems left stick Y; clamped to [-1, 1]. Positive stick forward = positive speed. */
    private double getTestSpeedFromAxis() {
        return Math.max(-1, Math.min(1, -subsystems.getLeftY()));
    }

    private void registerMotorTestCommandsOnSmartDashboard() {
        for (String motorId : MotorTestConstants.getMotorIds()) {
            String displayName = MotorTestConstants.getDisplayName(motorId);
            SmartDashboard.putData("Motor Test/" + displayName,
                    RunMotorTestCommand.atFixedSpeed(motorId, MotorTestConstants.getDefaultSpeed(), motorTest));
        }
    }

    /** Returns the trigger for the given button on the given controller. */
    private Trigger triggerFor(CommandXboxController controller, TestButton button) {
        return switch (button) {
            case A -> controller.a();
            case B -> controller.b();
            case X -> controller.x();
            case Y -> controller.y();
            case LEFT_BUMPER -> controller.leftBumper();
            case RIGHT_BUMPER -> controller.rightBumper();
            case POV_UP -> controller.povUp();
            case POV_DOWN -> controller.povDown();
        };
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Command driveCommand = drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
        driveCommand.addRequirements(visionMeasurement);
        drivetrain.setDefaultCommand(driveCommand);

        intake.setDefaultCommand(new RunCommand(intake::stow, intake));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Intake (subsystems controller): pivot + roller + hopper
        subsystems.a().whileTrue(new RunCommand(intake::intake, intake));
        subsystems.b().whileTrue(new RunCommand(intake::outtake, intake));
        subsystems.x().onTrue(intake.runOnce(intake::stow));
        subsystems.y().whileTrue(new RunCommand(intake::collect, intake));
        subsystems.rightBumper().whileTrue(new RunCommand(intake::feedToShooter, intake));
        subsystems.leftBumper().onTrue(intake.runOnce(intake::stopAll));

        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            knnInterpreter.update(state.Pose);
        });

        // Motor test (Test mode only): bindings from MotorTestConstants; speed from subsystems left Y
        Trigger inTest = new Trigger(Robot::inTestMode);
        for (var entry : MotorTestConstants.getController0Bindings().entrySet()) {
            inTest.and(triggerFor(joystick, entry.getKey()))
                    .whileTrue(new RunMotorTestCommand(entry.getValue(), this::getTestSpeedFromAxis, motorTest));
        }
        for (var entry : MotorTestConstants.getController1Bindings().entrySet()) {
            inTest.and(triggerFor(subsystems, entry.getKey()))
                    .whileTrue(new RunMotorTestCommand(entry.getValue(), this::getTestSpeedFromAxis, motorTest));
        }
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public ShooterSubsystem getShooter() {
        return shooter;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    /** Hood homing for real robot; schedule when enabling (auto or teleop). */
    public Command getHoodHomingCommand() {
        return new HoodHomingCommand(shooter);
    }
}
