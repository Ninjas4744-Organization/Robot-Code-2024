package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command _autonomousCommand;
  private String _autoSelected;
  private SendableChooser<String> _chooser;

  private RobotContainer _robotContainer;

  @Override
  public void robotInit() {
    _chooser = new SendableChooser<String>();
    ctreConfigs = new CTREConfigs();
    _robotContainer = new RobotContainer(); 

    _chooser.setDefaultOption("No Auto", "No Auto");

    _chooser.addOption("Amp", "Amp");
    _chooser.addOption("Amp Alt", "Amp Alt");
    _chooser.addOption("Amp Alt 2", "Amp Alt 2");
    _chooser.addOption("Amp Alt 3", "Amp Alt 3");

    _chooser.addOption("AmpExit", "AmpExit");
    _chooser.addOption("CenterExit", "CenterExit");
    _chooser.addOption("SourceExit", "SourceExit");
    _chooser.addOption("SourceOutake", "SourceOutake");

    _chooser.addOption("CenterAmp", "CenterAmp");
    _chooser.addOption("CenterAmp Alt", "CenterAmp Alt");
    _chooser.addOption("CenterAmp Alt 2", "CenterAmp Alt 2");

    SmartDashboard.putData("Autonomous Options", _chooser);
  }

  @Override
  public void robotPeriodic() {
    _robotContainer.periodic();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }

    _autoSelected = _chooser.getSelected();
    _autonomousCommand = _robotContainer.autoCommand(_autoSelected);
    _autonomousCommand.schedule();
  }

  @Override
  public void teleopInit() {
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }
    
    _robotContainer.Reset();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}