package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InOutTake;

public class RobotContainer {
  // Subsystems
  private final Elevator _elevator;
  private final InOutTake _inOutTake;
  
  // Commands
  
  // Other
  private final CommandPS5Controller _joystick;
  private HashMap<String, Command> _commands;

  public RobotContainer() { 

    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _elevator = new Elevator();
    _inOutTake = new InOutTake();

    _commands = new HashMap<String, Command>();
    
    configureBindings();
  }

  public HashMap<String, Command> getCommands() {
    _commands.put("Elevate", 
      new StartEndCommand(
        () -> { _elevator.setHeight(Constants.kMaxElevator); }, 
        () -> { _elevator.setHeight(0); }
      ));

    _commands.put("InOutTake", new ConditionalCommand(
      new StartEndCommand(
        () -> { _inOutTake.outake(); }, // OUTTAKE 
        () -> { _inOutTake.stopMotors(); }
      ).withTimeout(1), // TIME UNTIL MOTORS STOP

      new StartEndCommand(
        () -> { _inOutTake.intake(); }, // INTAKE
        () -> { _inOutTake.stopMotors(); }
      ).until(() -> { return _inOutTake.isNote();}),


      () -> { return _inOutTake.isNote(); } // CONDITION
    ));
    return _commands;
  }
  
  private void configureBindings() {
    //SysId
    _joystick.square().whileTrue(_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    _joystick.circle().whileTrue(_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    _joystick.R2().whileTrue(_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    _joystick.R1().whileTrue(_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //Elevator
    _joystick.cross().toggleOnTrue(new SelectCommand<String>( getCommands(), () -> { return "Elevate"; }));

    //Collection
    _joystick.triangle().onTrue(new SelectCommand<String>( getCommands(), () -> { return "InOutTake"; }));
  }

  public void disableActions(){
    _elevator.Reset();
  }
}