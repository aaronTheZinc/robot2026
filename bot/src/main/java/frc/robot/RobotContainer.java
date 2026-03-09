// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.ShooterConstants;
import frc.robot.commands.DecrementHoodDegreesCommand;
import frc.robot.commands.DecrementShooterRpmCommand;
import frc.robot.commands.HoodHomingCommand;
import frc.robot.commands.IncrementHoodDegreesCommand;
import frc.robot.commands.IncrementShooterRpmCommand;
import frc.robot.commands.PivotToDeployCommand;
import frc.robot.commands.PivotToStowCommand;
import frc.robot.commands.ShootComamnd;
import frc.robot.generated.TunerConstants;
import frc.robot.knn.KnnInterpreter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Debug: adjustable hood and shooter setpoints (run from SmartDashboard / robot dashboard)
        SmartDashboard.putData("Debug/Increment Hood (deg)", new IncrementHoodDegreesCommand(shooter));
        SmartDashboard.putData("Debug/Decrement Hood (deg)", new DecrementHoodDegreesCommand(shooter));
        SmartDashboard.putData("Debug/Increment Shooter RPM", new IncrementShooterRpmCommand(shooter));
        SmartDashboard.putData("Debug/Decrement Shooter RPM", new DecrementShooterRpmCommand(shooter));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Command driveCommand = drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
        // Chassis default runs on drivetrain only; no vision requirement so teleop is uninterrupted.
        drivetrain.setDefaultCommand(driveCommand);

        // intake.setDefaultCommand(new RunCommand(intake::stow, intake));

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

        // Subsystems controller (1): triggers, bumpers, buttons, D-pad
        subsystems.rightTrigger().whileTrue(new ShootComamnd(shooter, intake));
        subsystems.leftTrigger().whileTrue(new RunCommand(intake::intake, intake));

        subsystems.y().onTrue(new PivotToStowCommand(intake));
        subsystems.a().onTrue(new PivotToDeployCommand(intake));

        subsystems.rightBumper().whileTrue(
            new RunCommand(() -> shooter.setShooterVoltage(-ShooterConstants.kTestSpeed * ShooterConstants.kMaxVoltageVolts), shooter)
                .finallyDo(() -> shooter.stopShooter()));
        subsystems.leftBumper().whileTrue(
            new RunCommand(intake::spitOut, intake).finallyDo(() -> {
                intake.stopRoller();
                intake.stopHopper();
            }));

        subsystems.povUp().onTrue(new IncrementHoodDegreesCommand(shooter));
        subsystems.povDown().onTrue(new DecrementHoodDegreesCommand(shooter));
        subsystems.povLeft().onTrue(new DecrementShooterRpmCommand(shooter));
        subsystems.povRight().onTrue(new IncrementShooterRpmCommand(shooter));

        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            knnInterpreter.update(state.Pose);
        });
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
