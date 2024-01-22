package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InOutTake;

public class RobotContainer {
  // Subsystems
  private final Elevator _elevator;
  private final InOutTake _inOutTake;

  // Other
  private final CommandPS5Controller _joystick;

  public RobotContainer() { 

    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _elevator = new Elevator();
    _inOutTake = new InOutTake();
    
    configureBindings();
  }

  public HashMap<Integer, Command> getAcceptCommands() {
    HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

    for(int i = 11; i <= 16; i++)
      _acceptCommands.put(i, _elevator.runAutoElevate(_inOutTake));

    _acceptCommands.put(1, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation));
    _acceptCommands.put(2, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation));
    _acceptCommands.put(9, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation));
    _acceptCommands.put(10, _inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation));
    _acceptCommands.put(5, _inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation));
    _acceptCommands.put(6, _inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation));

    return _acceptCommands;
  }
  
  private void configureBindings() {
    //Driving:
    // TODO: add driving controls for both PS5 and Logitech with swerve

    //Auto:
    _joystick.cross().onTrue(new SelectCommand<Integer>(getAcceptCommands(), () -> { return getJoe(); }));
    _joystick.circle().onTrue(new InstantCommand(() -> { Override(); }));
    //_joystick.R2().onFalse(new InstantCommand(() -> { _vision.go(); }));

    //Semi-Auto:
    _joystick.square().toggleOnTrue(_inOutTake.runClose());
    _joystick.povUp().toggleOnTrue(_elevator.runElevate());
    _joystick.povDown().onTrue(_inOutTake.runAutoInOutTake(Constants.kSourceOpenHeight, Constants.kSourceOpenRotation));
    _joystick.povRight().onTrue(_inOutTake.runAutoInOutTake(Constants.kAmpOpenHeight, Constants.kAmpOpenRotation));
    _joystick.povLeft().onTrue(_inOutTake.runAutoInOutTake(Constants.kTrapOpenHeight, Constants.kTrapOpenRotation));
    //_joystick.L1().onTrue(new InstantCommand(() -> {_elevator.Tag = (_elevator.Tag + 1) % 17; }));

    //Manual:
    // TODO: add manual controls
  }

  private void Override(){
    _inOutTake.Override();
    _elevator.Override();
  }

  private int getTagID(){
    return 0;//_elevator.Tag;
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
  }
}