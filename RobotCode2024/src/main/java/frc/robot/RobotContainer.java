package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.drivers.GenericSparkMaxSubsystem;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.Lift;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.Rotation;

public class RobotContainer {
  // Subsystems
  private Swerve _swerve;
  private Vision _vision;
  private Climber _climber;
  private Rotation _rotation;
  private Rollers _rollers;
  private Lift _elevator;
  // private Leds _leds;

  // Misc
  private CommandBuilder _commandBuilder;
  private CommandPS5Controller _joystick;
  private CommandPS5Controller _joystick2;
  private Boolean onTag = false;
  private String Mode = "";

  public RobotContainer() {
    // Shuffleboard.getTab("Game").add("Mode", Mode);
    // Shuffleboard.getTab("Debug").add("Mode", Mode);

    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Constants.kJoystick2Port);
    _vision = new Vision();
    _swerve = new Swerve(_vision::estimationsSupplier);

    _climber = new Climber(GenericSparkMaxSubsystem.createSparkMaxMotorGroup(Constants.Elevator.kElevatorConstants));
    _rotation = new Rotation(GenericSparkMaxSubsystem.createSparkMaxMotorGroup(Constants.Elevator.kElevatorConstants));
    _rollers = new Rollers(GenericSparkMaxSubsystem.createSparkMaxMotorGroup(Constants.Elevator.kElevatorConstants));
    _elevator = new Lift(GenericSparkMaxSubsystem.createSparkMaxMotorGroup(Constants.Elevator.kElevatorConstants));
    // _leds = new Leds();
    
    

    // NamedCommands.registerCommand("Outake", CommandBuilder.runAutoOutake());
    NamedCommands.registerCommand("Reset", CommandBuilder.reset(_elevator, _rotation, _rollers, _climber, _swerve));

    configureBindings();
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
    configureManualBindings();
  }



  private void configureDriverBindings(){
    _swerve.setDefaultCommand(
      new TeleopSwerve(
        _swerve,
        _vision,
        () -> { return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient * Constants.Swerve.kDriveRotationCoefficient; },
        () -> { return false; },
        () -> { return false; }
      )
    );

    _joystick.R2().whileTrue(
      new TeleopSwerve(
        _swerve,
        _vision,
        () -> { return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient * Constants.Swerve.kDriveRotationCoefficient; },
        () -> { return true; },
        () -> { return false; }
      )
    );
   
    _joystick.L1().onTrue(Commands.runOnce(() -> { _swerve.zeroGyro(); }, _swerve));
  }

  private void configureOperatorBindings(){
    // _joystick2.cross().onTrue(Commands.select(getAcceptCommands(), () -> {return getAcceptId();}));

    _joystick2.cross().onTrue(
      Commands.parallel(
        _elevator.reset(),
        _rotation.reset()
      )
    );

    _joystick2.square().onTrue(
      Commands.parallel(
        _elevator.close(),
        _rotation.runOpenClose(Constants.Rotation.States.kUpRotation)
      )
    );

    _joystick2.triangle().onTrue(CommandBuilder.runInOutTake(_elevator, _rotation, _rollers, _joystick2.getHID()::getTriangleButton));

    _joystick2.circle().onTrue(CommandBuilder.runOutakeTrap(_elevator, _rotation, _rollers, _joystick2.getHID()::getCircleButton));
  }

  private void configureManualBindings(){
    _joystick2.povRight().whileTrue(
      _rollers.runMotors(1)
    );

    _joystick2.povLeft().whileTrue(
      _rollers.runMotors(-1)
    );

    _joystick2.povUp().whileTrue(
      _climber.runMotors(1)
    );

    _joystick2.povDown().whileTrue(
      _climber.runMotors(-1)
    );

    _joystick2.R2().whileTrue(
      _elevator.runMotors(0.3)
    );

    _joystick2.L2().whileTrue(
      _elevator.runMotors(-0.4)
    );

    _joystick2.R1().whileTrue(
      _rotation.runMotors(0.1)
    );

    _joystick2.L1().whileTrue(
      _rotation.runMotors(-0.07)
    );
  }

  


  public void Reset(){
    _swerve.resetOdometry(new Pose2d());
    CommandBuilder.reset(_elevator, _rotation, _rollers, _climber, _swerve).schedule();
  }

  public Command autoCommand(String auto) {
    return _commandBuilder.autoCommand(auto);
  }
}