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
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        if (DriverStation.isAutonomous()) {
            Command requiring = CommandScheduler.getInstance().requiring(m_robotContainer.drivetrain);
            AutoDiagnostics.publishActiveDriveCommand(requiring);
        }
        // VisionMeasurement subsystem runs in its periodic() and fuses Limelight with drivetrain odometry.
    }

    @Override
    public void disabledInit() {
        m_robotContainer.getShooter().setShooterReady(false);
        m_robotContainer.getIntake().setPivotReady(false);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand == null) {
            DriverStation.reportWarning(
                    "Auto: SendableChooser returned null — no autonomous command was scheduled.",
                    false);
            AutoDiagnostics.logEvent("autonomousInit: selected command is NULL");
        } else {
            AutoDiagnostics.logEvent("autonomousInit: schedule " + m_autonomousCommand.getName());
            SignalLogger.writeString("Auto/scheduledCommand", m_autonomousCommand.getName());
        }

        if (Utils.isSimulation()) {
            m_robotContainer.getShooter().setShooterReady(true);
            m_robotContainer.getIntake().setPivotReady(true);
        } else {
            CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomingCommand());
            // CommandScheduler.getInstance().schedule(m_robotContainer.getPivotHomingCommand());
        }

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        

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
    public void teleopExit() {}

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
