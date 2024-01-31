package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InOutTake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  // Subsystems
  private final Elevator _elevator;
  private final InOutTake _inOutTake;
  private final Swerve _swerve;
  private final Vision _vision;
  
  // Other
  private final CommandPS5Controller _joystick;
  private final CommandPS5Controller _joystick2;
  private boolean override = false;
  private boolean manual = false;

  public RobotContainer() { 
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Constants.kJoystick2Port);
    _elevator = new Elevator();
    _inOutTake = new InOutTake();
    _swerve = new Swerve();
    _vision = new Vision();

    configureBindings();
  }

  public HashMap<Integer, Command> getAcceptCommands() {
    HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

    for(int i = 11; i <= 16; i++)
      _acceptCommands.put(i, _elevator.runAutoElevate(_inOutTake, () -> { return override; }, () -> {override = false;}));

    _acceptCommands.put(1, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation, () -> { return override; }, () -> {override = false;}));
    _acceptCommands.put(2, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation, () -> { return override; }, () -> {override = false;}));
    _acceptCommands.put(9, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation, () -> { return override; }, () -> {override = false;}));
    _acceptCommands.put(10, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation, () -> { return override; }, () -> {override = false;}));
    _acceptCommands.put(5, _inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation, () -> { return override; }, () -> {override = false;}));
    _acceptCommands.put(6, _inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation, () -> { return override; }, () -> {override = false;}));

    return _acceptCommands;
  }
  
  private void configureBindings() {
    //Driving:
    _swerve.setDefaultCommand(
      new TeleopSwerve(
          _swerve,
          _vision,
          () -> -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getLeftY() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getLeftX() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getRightX() * Constants.Swerve.kDriveCoefficient,
          () -> false,
          () -> false,
          () -> { return manual; }
      ).alongWith(new RunCommand(() -> LimelightHelpers.setLEDMode_ForceOff(null)))
    );

    //Auto:
    _joystick.cross().onTrue(new SelectCommand<Integer>(getAcceptCommands(), () -> { return getJoe(); }));
    _joystick.circle().onTrue(new InstantCommand(() -> { Override(); }));
    _joystick.R2().whileTrue(new TeleopSwerve(
      _swerve,
      _vision,
      () -> -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient,
      () -> -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient,
      () -> -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getLeftY() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getLeftX() * Constants.Swerve.kDriveCoefficient,
          () -> -_joystick2.getRightX() * Constants.Swerve.kDriveCoefficient,
      () -> true,
      () -> false,
      () -> { return manual; }
    ).alongWith(new RunCommand(() -> {
      if (!isNotTarget())
        LimelightHelpers.setLEDMode_ForceOn(null);
      else
        LimelightHelpers.setLEDMode_ForceOff(null);
    })));
    
    //Semi-Auto:
    _joystick.square().toggleOnTrue(_inOutTake.runClose());
    _joystick.povUp().toggleOnTrue(_elevator.runElevate());
    _joystick.povDown().onTrue(_inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation, () -> { return override; }, () -> {override = false;}));
    _joystick.povRight().onTrue(_inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation, () -> { return override; }, () -> {override = false;}));
    _joystick.povLeft().onTrue(_inOutTake.runAutoInOutTake(Constants.kTrapOpenHeight, Constants.kTrapOpenRotation, () -> { return override; }, () -> {override = false;}));
    //Temp
    _joystick.L1().onTrue(new InstantCommand(() -> {_elevator.Tag = (_elevator.Tag + 1) % 17; }, _elevator));

    //Manual:
    _joystick2.L2().whileTrue(new StartEndCommand(() -> {_inOutTake.intake();}, () -> {_inOutTake.stopTake();}, _inOutTake));
    _joystick2.R2().whileTrue(new StartEndCommand(() -> {_inOutTake.outake();}, () -> {_inOutTake.stopTake();}, _inOutTake));
    _joystick2.povUp().whileTrue(new StartEndCommand(() -> {_elevator.setMotor(1);}, () -> {_elevator.Reset();}, _elevator));
    _joystick2.povDown().whileTrue(new StartEndCommand(() -> {_elevator.setMotor(-1);}, () -> {_elevator.Reset();}, _elevator));
    _joystick2.L1().whileTrue(new StartEndCommand(() -> {_inOutTake.setElevatorMotor(-1);}, () -> {_inOutTake.stopElevator();}, _inOutTake));
    _joystick2.R1().whileTrue(new StartEndCommand(() -> {_inOutTake.setElevatorMotor(1);}, () -> {_inOutTake.stopElevator();}, _inOutTake));
    _joystick2.triangle().whileTrue(new StartEndCommand(() -> {_inOutTake.setRotationMotor(1);}, () -> {_inOutTake.stopRotation();}, _inOutTake));
    _joystick2.cross().whileTrue(new StartEndCommand(() -> {_inOutTake.setRotationMotor(-1);}, () -> {_inOutTake.stopRotation();}, _inOutTake));
  }

  private void Override(){
    _inOutTake.Override();
    _elevator.Override();
    _swerve.Override();
    override = true;
  }

  private int getTagID(){
    return _vision.getTag().ID;
  }

  private int getJoe(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    int tag = getTagID();
    List<Integer> blueIDs = Arrays.asList(6, 14, 15, 16, 1, 2);
    List<Integer> redIDs = Arrays.asList(5, 11, 12, 13, 9, 10);

    if(ally.get() == Alliance.Blue){
      if(blueIDs.contains(tag))
        return tag;
    }
    else
    {
      if(redIDs.contains(tag))
        return tag;
    }

  return -1;
}

  public void disableActions(){
    _elevator.Reset();
    _inOutTake.Reset();
    _swerve.zeroGyro();
    _swerve.resetOdometry(new Pose2d());
  }

  public Boolean isNotTarget() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = _swerve.getLastCalculatedPosition();
    return !LimelightHelpers.getTV(null) ||
    Math.hypot(targetPose.getX() - current_pos.getX(),targetPose.getY() - current_pos.getY()) > 2;
  }

  public Command autoCommand(){
    PathPlannerPath _path = PathPlannerPath.fromPathFile("Init");
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose());
      }),
      AutoBuilder.buildAuto("basic")
    );
  }
}