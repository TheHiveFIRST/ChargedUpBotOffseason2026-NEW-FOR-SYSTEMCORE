// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.OpModeRobot;

public class Robot extends OpModeRobot {

  private RobotContainer m_robotContainer;
  public CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void driverStationConnected() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void nonePeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
}
