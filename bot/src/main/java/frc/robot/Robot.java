// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    /** Limelight host for simulation; access at http://localhost:5801 when forwarded. */
    private static final String LIMELIGHT_HOST = "limelight.local";
    private static final int LIMELIGHT_PORT_START = 5800;
    private static final int LIMELIGHT_PORT_END = 5809;

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
        if (Utils.isSimulation()) {
            for (int port = LIMELIGHT_PORT_START; port <= LIMELIGHT_PORT_END; port++) {
                PortForwarder.add(port, LIMELIGHT_HOST, port);
            }
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        // VisionMeasurement subsystem runs in its periodic() and fuses Limelight with drivetrain odometry.
    }

    @Override
    public void disabledInit() {
        m_robotContainer.getShooter().setShooterReady(false);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        if (Utils.isSimulation()) {
            m_robotContainer.getShooter().setShooterReady(true);
        } else {
            CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomingCommand());
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
        } else {
            CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomingCommand());
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
