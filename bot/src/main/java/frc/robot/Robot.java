// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;

public class Robot extends TimedRobot {
    /** True when Driver Station is in Test mode; used to gate test bindings and motor test periodic. */
    private static boolean s_inTestMode;

    /** Returns true when the robot is in Test mode (Driver Station Test selected). */
    public static boolean inTestMode() {
        return s_inTestMode;
    }
    /** Limelight host for simulation; access at http://localhost:5801 when forwarded. */
    private static final String LIMELIGHT_HOST = "limelight.local";
    private static final int LIMELIGHT_PORT_START = 5800;
    private static final int LIMELIGHT_PORT_END = 5809;
    private UsbCamera m_driverCamera;

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final MatchSessionLogger m_matchSessionLogger = new MatchSessionLogger();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        if (!Utils.isSimulation()) {
            m_driverCamera = CameraServer.startAutomaticCapture();
            m_driverCamera.setResolution(320, 240);
            m_driverCamera.setFPS(20);
        }

        if (Utils.isSimulation()) {
            for (int port = LIMELIGHT_PORT_START; port <= LIMELIGHT_PORT_END; port++) {
                PortForwarder.add(port, LIMELIGHT_HOST, port);
            }
        }

        // m_robotContainer.flipDriveDirection();

        registerCommandSchedulerDiagnostics();
        m_matchSessionLogger.robotInit();
    }

    private void registerCommandSchedulerDiagnostics() {
        CommandScheduler sch = CommandScheduler.getInstance();
        sch.onCommandInitialize(
                cmd -> {
                    if (DriverStation.isAutonomous()) {
                        AutoDiagnostics.logEvent("cmd init: " + cmd.getName());
                    }
                });
        sch.onCommandFinish(
                cmd -> {
                    if (DriverStation.isAutonomous()) {
                        AutoDiagnostics.logEvent("cmd finish: " + cmd.getName());
                    }
                });
        sch.onCommandInterrupt(
                (Command cmd, Optional<Command> interruptor) -> {
                    if (DriverStation.isAutonomous()) {
                        String by = interruptor.map(Command::getName).orElse("(cancel/scheduler)");
                        AutoDiagnostics.logEvent("cmd interrupt: " + cmd.getName() + " <= " + by);
                    }
                });
    }

    @Override
    public void robotPeriodic() {
        AutoDiagnostics.periodicDebugTelemetrySync();
        m_timeAndJoystickReplay.update();
        CommandScheduler cs = CommandScheduler.getInstance();
        cs.run();
        m_robotContainer.applyKnnHoodInterpolation();
        m_robotContainer.getShooter().applyHoodMotorClosedLoopTick();
        if (DriverStation.isAutonomous()) {
            AutoDiagnostics.publishActiveDriveCommand(cs.requiring(m_robotContainer.drivetrain));
            AutoDiagnostics.publishAutonomousSchedulerSnap(m_autonomousCommand, cs, m_robotContainer.drivetrain);
        }
        m_robotContainer.updateDriverRumble();
        m_robotContainer.publishHubAlignTelemetry();
        m_matchSessionLogger.periodic(m_robotContainer.drivetrain, m_robotContainer.visionMeasurement);
    }

    @Override
    public void disabledInit() {
        m_matchSessionLogger.flush();
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.clearDriverRumble();
        m_robotContainer.getShooter().setShooterReady(false);
        m_robotContainer.getIntake().setPivotReady(false);
        AutoDiagnostics.publishDefaultDriveCanceledForAuto(false);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.clearDriverRumble();
        Command selectedAuto = m_robotContainer.getAutonomousCommand();
        m_autonomousCommand = selectedAuto;

        if (Utils.isSimulation()) {
            m_robotContainer.getShooter().setShooterReady(true);
            m_robotContainer.getIntake().setPivotReady(true);
        } else {
            // Hood homing first, then PathPlanner auto — avoids parallel init fighting PathPlannerAuto / follower
            // (which was preventing paths from running reliably). Odometry is seeded to the auto start pose before
            // this (see seedOdometryForAuto).
            m_robotContainer.getShooter().setShooterReady(false);
            m_robotContainer.seedOdometryForAuto(selectedAuto);
            Command hoodHoming = m_robotContainer.getHoodHomingCommand();
            if (selectedAuto != null) {
                m_autonomousCommand = hoodHoming.andThen(selectedAuto).withName("AutoWithHoodHoming");
            } else {
                m_autonomousCommand = hoodHoming.withName("HoodHomingOnly");
            }
        }

        // Chassis: match SwerveWithPathPlanner — schedule auto only; subsystem requirements handle the
        // default drive command.
        if (m_autonomousCommand != null) {
            AutoDiagnostics.logEvent("autonomousInit: schedule " + m_autonomousCommand.getName());
            SignalLogger.writeString("Auto/scheduledCommand", m_autonomousCommand.getName());
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        } else {
            DriverStation.reportWarning(
                    "Auto: SendableChooser returned null — no autonomous command was scheduled.", false);
            AutoDiagnostics.logEvent("autonomousInit: selected command is NULL");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        // Stop PathPlanner / nested commands so outputs do not persist. Do not reset pose to (0,0) here —
        // that broke path following on re-enable and disagreed with PathPlanner/vision; fused pose is kept
        // for teleop and the next auto (seedOdometryForAuto corrects only when odometry is still uninitialized).
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
        // cancel(m_autonomousCommand) is not enough: composed autos and PathPlanner internals may leave
        // other commands scheduled that still require the drivetrain. Clear the scheduler, then only add
        // what teleop needs (e.g. hood homing); default drive is re-scheduled automatically for swerve.
        CommandScheduler.getInstance().cancelAll();
        m_autonomousCommand = null;

        if (Utils.isSimulation()) {
            m_robotContainer.getShooter().setShooterReady(true);
            m_robotContainer.getIntake().setPivotReady(true);
        } else {
            CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomingCommand());
            // CommandScheduler.getInstance().schedule(m_robotContainer.getPivotHomingCommand());
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        m_robotContainer.clearDriverRumble();
    }

    @Override
    public void testInit() {
        s_inTestMode = true;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        s_inTestMode = false;
    }

    @Override
    public void simulationPeriodic() {}
}
