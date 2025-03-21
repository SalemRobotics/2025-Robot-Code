// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // DataLogManager.start("/logs/datalogs/"); // separate data and signal logs
    
    // // add drivestation logs (e.g. joystick data)
    // DriverStation.startDataLog(DataLogManager.getLog(), true);
    // // add signal logging for CTRE
    // SignalLogger.start();
    // SignalLogger.setPath("/logs/signallogs/");

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    SmartDashboard.putString("Aligned X", "Unknown (not disabled)");
    SmartDashboard.putString("Aligned Y", "Unknown (not disabled)");
    SmartDashboard.putNumber("Aligned X (num)", -1);
    SmartDashboard.putNumber("Aligned Y (num)", -1);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    m_robotContainer.teleExit();
  }

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
