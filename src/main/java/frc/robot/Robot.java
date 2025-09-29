// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.util.ClassPreloader;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import lombok.val;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  // alert to display if preallocated memory is
  private static final Notification kFullHeapNotification =
      new Notification()
          .withDescription(
              "Used memory is more than 90% of preallocated memory, should increase amount of preallocated memory")
          .withLevel(NotificationLevel.WARNING)
          .withNoAutoDismiss();

  private static final Notification kUnusedHeapNotification =
      new Notification()
          .withDescription(
              "Used memory is less than 50% of preallocated memory, should reduce amount of preallocated memory")
          .withLevel(NotificationLevel.WARNING)
          .withNoAutoDismiss();

  public Robot() {    
    // Preload subsystem classes
    ClassPreloader.preload(
        "frc.robot.RobotContainer",
        "frc.robot.subsystems.drive.Drive",
        "frc.robot.subsystems.drive.Module",
        "frc.robot.subsystems.elevator.Elevator",
        "frc.robot.subsystems.vision.Vision",
        "frc.robot.subsystems.end_effector.EndEffector",
        "frc.robot.subsystems.algae_arm.AlgaeArm",
        "frc.robot.subsystems.Superstructure");

    DriverStation.silenceJoystickConnectionWarning(true);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // disable redundant logging from CTRE
        SignalLogger.enableAutoLogging(false);
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    val rt = Runtime.getRuntime();

    val usedMem = (double)(rt.totalMemory() - rt.freeMemory());

    Logger.recordOutput("Robot/Utilized Memory %", usedMem / (double)rt.totalMemory());
    Logger.runEveryN(
        // run every second
        2500,
        () -> {
          if (usedMem >= 0.9 * Constants.TotalMemory) {
            // If memory utilization is too high, send an alert to increase allocated heap size in JVM args
            Elastic.sendNotification(kFullHeapNotification);
          } else if (usedMem <= 0.5 * Constants.TotalMemory) {
            // If memory utilization is too low, send an alert to decrease allocated heap size in JVM args
            Elastic.sendNotification(kUnusedHeapNotification);
          }
        });

    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    robotContainer.periodic();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Trigger a GC upon disable to ensure that all unnecessary objects are cleaned up.
    // This is especially important after auto.
    System.gc();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.stowElevator();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
