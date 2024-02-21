// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

  }

  // @Override
  // public void robotPeriodic() {
  //   CommandScheduler.getInstance().run();

  //   m_robotContainer.periodic();

  // }

  // @Override
  // public void autonomousInit() {
  //   m_robotContainer.autoCommand().schedule();
  //   super.autonomousInit();
  // }

  // @Override
  // public void disabledInit() {
  //   m_robotContainer.disabledActions();
  // }

  @Override
  public void teleopInit() {
    LimelightHelpers.setLEDMode_ForceOff(null);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}
