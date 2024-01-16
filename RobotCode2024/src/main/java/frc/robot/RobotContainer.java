package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InOutTake;

public class RobotContainer {
  // Subsystems
  private final Elevator _elevator;
  private final InOutTake _inOutTake;
  
  // Commands
  
  // Other
  private final CommandPS5Controller _joystick;
  
  public RobotContainer() { 
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _elevator = new Elevator();
    _inOutTake = new InOutTake();

    configureBindings();
  }
  
  private void configureBindings() {

    //Elevator
    _joystick.cross().toggleOnTrue(
      new StartEndCommand(
        () -> { _elevator.setHeight(Constants.kMaxElevator); }, 
        () -> { _elevator.setHeight(0); }
      )
    );

    //Collection
    _joystick.triangle().onTrue(new ConditionalCommand(
      new StartEndCommand(
        () -> { _inOutTake.outake(); }, // OUTTAKE 
        () -> { _inOutTake.stopMotors(); }
      ).withTimeout(1), // TIME UNTIL MOTORS STOP

      new StartEndCommand(
        () -> { _inOutTake.intake(); }, // INTAKE
        () -> { _inOutTake.stopMotors(); }
      ).withTimeout(1.5), // TIME UNTIL MOTORS STOP

      () -> { return _inOutTake.isNote(); } // CONDITION
    ));
  }

  public void disableActions(){
    _elevator.Reset();
  }
}