package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.Elevator;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.Rotation;

public class RobotContainer {
  // Subsystems
  private Swerve _swerve;
  private Vision _vision;
  private Climber _climber;
  private Elevator _elevator;
  private Rotation _rotation;
  private Rollers _rollers;
  // private Leds _leds;

  // Misc
  private CommandBuilder _commandBuilder;
  private CommandPS5Controller _joystick;
  private CommandPS5Controller _joystick2;
  private Boolean withTag = false;
  private Boolean onTag = false;
  private String Mode = "";

  public RobotContainer() {
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Constants.kJoystick2Port);
    _vision = new Vision();
    _swerve = new Swerve(_vision::estimationsSupplier);

    _climber = new Climber();
    _elevator = new Elevator();
    _rotation = new Rotation();
    _rollers = new Rollers();
    // _leds = new Leds();

    _commandBuilder = new CommandBuilder(_swerve, _climber, _elevator, _rotation, _rollers);

    NamedCommands.registerCommand("Outake", _commandBuilder.runAutoOutake());
    NamedCommands.registerCommand("Reset", _commandBuilder.Reset());

    configureBindings();
  }

  private void configureBindings() {
    configureMiscBindings();
    configureDriverBindings();
    configureOperatorBindings();
    configureManualBindings();
  }

  private void configureMiscBindings(){
    // Trigger tNote = new Trigger(() -> {return _rollers.isNote();});
    // Trigger tIntake = new Trigger(() -> {return _rollers.getMotor() == -1;});
    // Trigger tClimb = new Trigger(() -> {return _climber.getHeight() > 0.02;});

    Trigger tClose = new Trigger(() -> {return _elevator.isHeight(0) && _rotation.isRotation(0);});
    Trigger tAmp = new Trigger(() -> {return _elevator.isHeight(Constants.Elevator.States.kAmpOpenHeight);});
    Trigger tSrc = new Trigger(() -> {return _elevator.isHeight(Constants.Elevator.States.kSourceOpenHeight);});
    Trigger tUp = new Trigger(() -> {return _rotation.isRotation(Constants.Rotation.States.kUpRotation);});
    Trigger tTrap = new Trigger(() -> {return _elevator.isHeight(Constants.Elevator.States.kTrapOpenHeight);});

    // tNote.whileFalse(_leds.setColor(255, 0, 0));
    // tNote.whileTrue(_leds.setColor(0, 255, 0));
    // tIntake.whileTrue(_leds.setColorBeep(0, 255, 0, 0.2));
    // tClimb.whileTrue(_leds.setColor(255, 255, 0));
    // tTrap.whileTrue(_leds.setColorBeep(255, 255, 0, 0.2));

    tClose.onTrue(Commands.runOnce(() -> {Mode = "Close";}));
    tAmp.onTrue(Commands.runOnce(() -> {Mode = "Amp";}));
    tSrc.onTrue(Commands.runOnce(() -> {Mode = "Source";}));
    tUp.onTrue(Commands.runOnce(() -> {Mode = "Saving";}));
    tTrap.onTrue(Commands.runOnce(() -> {Mode = "Trap";}));
  }

  private void configureDriverBindings(){
    _swerve.setDefaultCommand(
      new TeleopSwerve(
        _swerve,
        _vision,
        () -> { return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient * Constants.Swerve.kDriveRotationCoefficient; },
        () -> { return withTag; },
        () -> { return onTag; },
        () -> { return true; }
      )
    );

    _joystick.R2().whileTrue(
      Commands.startEnd(
        () -> withTag = true,
        () -> withTag = false
      )
    );
    _joystick.R2().onTrue(
      Commands.startEnd(
        () -> onTag = true,
        () -> onTag = false
      )
    );

    _joystick.L1().onTrue(Commands.runOnce(() -> { _swerve.zeroGyro(); }, _swerve));
  }

  private void configureOperatorBindings(){
    // _joystick2.cross().onTrue(Commands.select(getAcceptCommands(), () -> {return getAcceptId();}));

    _joystick2.cross().onTrue(
      Commands.parallel(
        _elevator.Reset(),
        _rotation.Reset()
      )
    );

    _joystick2.square().onTrue(
      Commands.parallel(
        _elevator.runClose(),
        _rotation.runOpen(Constants.Rotation.States.kUpRotation)
      )
    );

    _joystick2.triangle().onTrue(_commandBuilder.runInOutTake(
        Constants.Elevator.States.kAmpOpenHeight, Constants.Rotation.States.kAmpOpenRotation,
        _joystick2.getHID()::getTriangleButton));

    _joystick2.circle().onTrue(_commandBuilder.runOutake(Constants.Elevator.States.kTrapOpenHeight,
        Constants.Rotation.States.kTrapOpenRotation, _joystick2.getHID()::getCircleButton));
  }

  private void configureManualBindings(){
    _joystick2.povRight().whileTrue(
        Commands.startEnd(
            () -> {
              _rollers.setMotor(1);
            },
            () -> {
              _rollers.Stop();
            },
            _rollers));

    _joystick2.povLeft().whileTrue(
        Commands.startEnd(
            () -> {
              _rollers.setMotor(-1);
            },
            () -> {
              _rollers.Stop();
            },
            _rollers));

    _joystick2.povUp().whileTrue(
        Commands.startEnd(
            () -> {
              _climber.setMotor(1);
            },
            () -> {
              _climber.Stop();
            },
            _climber));

    _joystick2.povDown().whileTrue(
        Commands.startEnd(
            () -> {
              _climber.setMotor(-1);
            },
            () -> {
              _climber.Stop();
            },
            _climber));

    _joystick2.R2().whileTrue(
        Commands.startEnd(
            () -> {
              _elevator.setMotor(0.4);
            },
            () -> {
              _elevator.Stop();
            },
            _elevator));

    _joystick2.L2().whileTrue(
        Commands.startEnd(
            () -> {
              _elevator.setMotor(-0.4);
            },
            () -> {
              _elevator.Stop();
            },
            _elevator));

    _joystick2.R1().whileTrue(
        Commands.startEnd(
            () -> {
              _rotation.setMotor(0.1);
            },
            () -> {

              _rotation.Stop();
            },
            _rotation));

    _joystick2.L1().whileTrue(
        Commands.startEnd(
            () -> {
              _rotation.setMotor(-0.07);
            },
            () -> {
              _rotation.Stop();
            },
            _rotation));
  }

  public void periodic(){
    SmartDashboard.putString("Mode", Mode);
  }

  private HashMap<Integer, Command> getAcceptCommands() {
    HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

    // Source
    _acceptCommands.put(1, _commandBuilder.runIntake());
    _acceptCommands.put(2, _commandBuilder.runIntake());
    _acceptCommands.put(9, _commandBuilder.runIntake());
    _acceptCommands.put(10, _commandBuilder.runIntake());

    // Amp
    for(int i = 5; i <= 6; i++)
      _acceptCommands.put(i, _commandBuilder.runOutake(
        Constants.Elevator.States.kAmpOpenHeight,
        Constants.Rotation.States.kAmpOpenRotation,
        _joystick2.getHID()::getCrossButton)
      );

    // Stage
    for(int i = 11; i <= 16; i++)
      _acceptCommands.put(i, _commandBuilder.runOutake(Constants.Elevator.States.kTrapOpenHeight,
          Constants.Rotation.States.kTrapOpenRotation, _joystick2.getHID()::getCrossButton));

    return _acceptCommands;
  }

  private int getAcceptId() {
    int tag = _vision.getTag().ID;
    Optional<Alliance> ally = DriverStation.getAlliance();
    List<Integer> blueIDs = Arrays.asList(6, 14, 15, 16, 1, 2);
    List<Integer> redIDs = Arrays.asList(5, 11, 12, 13, 9, 10);

    if (ally.get() == Alliance.Blue) {
      if (blueIDs.contains(tag))
        return tag;
    } else {
      if (redIDs.contains(tag))
        return tag;
    }

    return -1;
  }

  public void Reset(){
    _commandBuilder.Reset().schedule();
  }
}