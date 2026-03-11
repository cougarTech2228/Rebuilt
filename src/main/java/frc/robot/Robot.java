// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.commands.AutoAimCommand;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private Command autoAimCommand;
  private RobotContainer robotContainer;

  @AutoLogOutput(key = "Shift Timer/Time Remaining")
  public static double shiftTimeRemaining;
  @AutoLogOutput(key = "Shift Timer/Shift")
  public static String shiftLabel;
  @AutoLogOutput(key = "Shift Timer/Hub Active")
  public static boolean hubActive;
  private double shiftStartTime;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
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
    SignalLogger.setPath("/u/logs");
    SignalLogger.start();
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    autoAimCommand = new AutoAimCommand(robotContainer.drive, robotContainer.turret);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    calculateShiftTime();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.testPeriodic();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    shiftStartTime = Timer.getFPGATimestamp();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
    CommandScheduler.getInstance().schedule(autoAimCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    shiftStartTime = Timer.getFPGATimestamp();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // kill the auto aim command that was running durring auto
    autoAimCommand.cancel();

    // Start a new auto aim command that is always running
    CommandScheduler.getInstance().schedule(autoAimCommand);

    robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    robotContainer.testPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }

  private void calculateShiftTime() {
    // Game specific message format from the manual:
    // The alliance will be provided as a single character representing
    // the color of the alliance whose goal will go inactive first
    // (i.e. 'R' = red, 'B' = blue). This alliance’s goal will be active in Shifts 2 and 4.

    if (!DriverStation.getAlliance().isPresent() || !DriverStation.isEnabled()) {
      hubActive = false;
      return;
    }
    Alliance currentAlliance = DriverStation.getAlliance().get();

    double elapsedTime = Timer.getFPGATimestamp() - shiftStartTime;

    if (DriverStation.isAutonomous()) {
      shiftLabel = "Autonomous";
      hubActive = true;
      shiftTimeRemaining = 20 - elapsedTime;
    } else if (DriverStation.isTeleop()) {
      if (elapsedTime <= 10) {
        shiftLabel = "Transition";
        hubActive = true;
        shiftTimeRemaining = 10 - elapsedTime;
      }
      else if (elapsedTime <= 35) {
        shiftLabel = "Shift 1/4";
        shiftTimeRemaining = 35 - elapsedTime;
        String gsm = DriverStation.getGameSpecificMessage();
        if (!gsm.isEmpty()) {
          hubActive = (currentAlliance == Alliance.Blue && gsm.startsWith("R")) ||
                      (currentAlliance == Alliance.Red && gsm.startsWith("B"));
        }
      }
      else if (elapsedTime <= 60) {
        shiftLabel = "Shift 2/4";
        shiftTimeRemaining = 60 - elapsedTime;
        String gsm = DriverStation.getGameSpecificMessage();
        if (!gsm.isEmpty()) {
          hubActive = (currentAlliance == Alliance.Blue && gsm.startsWith("B")) ||
                      (currentAlliance == Alliance.Red && gsm.startsWith("R"));
        }
      }
      else if (elapsedTime <= 85) {
        shiftLabel = "Shift 3/4";
        shiftTimeRemaining = 85 - elapsedTime;
        String gsm = DriverStation.getGameSpecificMessage();
        if (!gsm.isEmpty()) {
          hubActive = (currentAlliance == Alliance.Blue && gsm.startsWith("R")) ||
                      (currentAlliance == Alliance.Red && gsm.startsWith("B"));
        }
      }
      else if (elapsedTime <= 110) {
        shiftLabel = "Shift 4/4";
        shiftTimeRemaining = 110 - elapsedTime;
        String gsm = DriverStation.getGameSpecificMessage();
        if (!gsm.isEmpty()) {
          hubActive = (currentAlliance == Alliance.Blue && gsm.startsWith("B")) ||
                      (currentAlliance == Alliance.Red && gsm.startsWith("R"));
        }
      } else {
        shiftLabel = "Endgame";
        hubActive = true;
        shiftTimeRemaining = 140 - elapsedTime;
      }
    }
  }
}
