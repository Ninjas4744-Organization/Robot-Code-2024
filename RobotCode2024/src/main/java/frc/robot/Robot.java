package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    LimelightHelpers.setLEDMode_ForceOff(null);
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    
    _chooser.setDefaultOption("LeftAmp", "LeftAmp");
    _chooser.addOption("LeftAmp Alternate End", "LeftAmpAltEnd");

    _chooser.addOption("LeftAmp One Ground", "LeftAmpOneGround");
    _chooser.addOption("LeftAmp One Ground Alternate End", "LeftAmpOneGroundAltEnd");

    _chooser.addOption("LeftAmp Two Ground", "LeftAmpTwoGround");
    _chooser.addOption("LeftAmp Two Ground Alternate End", "LeftAmpTwoGroundAltEnd");


    _chooser.addOption("CenterAmp", "CenterAmp");
    _chooser.addOption("CenterAmp Alternate End", "CenterAmpAltEnd");

    _chooser.addOption("CenterAmp One Ground", "CenterAmpOneGround");
    _chooser.addOption("CenterAmp One Ground Alternate End", "CenterAmpOneGroundAltEnd");

    _chooser.addOption("CenterAmp Two Ground", "CenterAmpTwoGround");
    _chooser.addOption("CenterAmp Two Ground Alternate End", "CenterAmpTwoGroundAltEnd");
    
    
    _chooser.addOption("RightExit", "RightExit");
    _chooser.addOption("RightExit Alternate End", "RightExitAltEnd");
    
    SmartDashboard.putData("Autonomous Options", _chooser);
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    _robotContainer.periodic();
  }

  @Override
  public void autonomousInit() {
    _autoSelected = _chooser.getSelected();
    _robotContainer.autoCommand(_autoSelected).schedule();
    super.autonomousInit();
  }

  @Override
  public void disabledInit() {
    //_robotContainer.disableActions();
  }
  
  @Override
  public void teleopInit() {
    LimelightHelpers.setLEDMode_ForceOff(null);
    
    Trigger trigger = new Trigger(() -> {return true;});
    trigger.onTrue(_robotContainer.Reset());

    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // @Override
  // public void autonomousPeriodic() {
  //   switch (_autoSelected) {
  //     case _customAuto:
  //       m_robotContainer.autoCommand("AmpAndSource");
  //       break;
  //     case _defaultAuto:
  //     default:
  //       m_robotContainer.autoCommand("AmpOnly");
  //       break;
  //   }
  //   super.autonomousPeriodic();
  // }

}