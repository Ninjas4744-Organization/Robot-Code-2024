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
    _commands.put("ClimbElevate", 
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

    _commands.put("CollectionElevate", 
      new StartEndCommand(
        () -> { _inOutTake.setHeight(Constants.kMaxInOutTakeElevator); }, 
        () -> { _inOutTake.setHeight(0); }
    ));

    _commands.put("RotateUp", new StartEndCommand(
        () -> { _inOutTake.setRotation(Constants.kMaxInOutTakeRotation); },
        () -> { _inOutTake.stopMotors(); }
      ).until(() -> { return _inOutTake.isRotationTop();})
    );

    _commands.put("RotateDown", new StartEndCommand(
        () -> { _inOutTake.setRotation(0); },
        () -> { _inOutTake.stopMotors(); }
      ).until(() -> { return _inOutTake.isRotationBottom();})
    );

    return _commands;
  }
  
  private void configureBindings() {
    //Elevator
    _joystick.triangle().toggleOnTrue(new SelectCommand<String>( getCommands(), () -> { return "ClimbElevate"; }));

    //Collection
    _joystick.cross().onTrue(new SelectCommand<String>( getCommands(), () -> { return "InOutTake"; }));
    _joystick.square().toggleOnTrue(new SelectCommand<String>( getCommands(), () -> { return "CollectionElevate"; }));
    _joystick.R1().onTrue(new SelectCommand<String>( getCommands(), () -> { return "RotateUp"; }));
    _joystick.L1().onTrue(new SelectCommand<String>( getCommands(), () -> { return "RotateDown"; }));
  }

  public void disableActions(){
    _elevator.Reset();
  }
}