package frc.robot;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SequentialStandByCommandGroup;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.Elevator;
import frc.robot.subsystems.Intake.Rotation;
import frc.robot.Constants;
import frc.robot.Constants.Rollers;

public class RobotContainer {
  // Subsystems
  private Swerve _swerve;
  private Vision _vision;
  private Climber _climber;
  private Elevator _elevator;
  private Rotation _rotation;
  private Rollers _rollers;
  private Leds _leds;
  private Trigger _start_game_trigger;

  // Misc
  private CommandPS5Controller _joystick;
  private CommandPS5Controller _joystick2;
  private Boolean withTag = false;

  public RobotContainer() {
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Constants.kJoystick2Port);
    _vision = new Vision();
    _swerve = new Swerve(_vision::estimationsSupplier);

    _climber = new Climber();
    _elevator = new Elevator();
    _rotation = new Rotation();
    _rollers = new Rollers();
    _leds = new Leds();

    NamedCommands.registerCommand("Outake", runAutoOutake());

    configureBindings();
  }

  public void configureBindings() {
    // Misc:
    

    tNote.whileFalse(_leds.setColor(255, 0, 0));
    tNote.whileTrue(_leds.setColor(0, 255, 0));
    tIntake.whileTrue(_leds.setColorBeep(0, 255, 0, 0.1));
    // tIntake.onFalse(Commands.none());

    _start_game_trigger = new Trigger(() -> {
      return DriverStation.getMatchTime() < 1;
    });
    // Driving:
    _swerve.setDefaultCommand(
        new TeleopSwerve(
            _swerve,
            _vision,
            () -> {
              return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient;
            },
            () -> {
              return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient;
            },
            () -> {
              return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient
                  * Constants.Swerve.kDriveRotationCoefficient;
            },
            () -> {
              return withTag;
            },
            () -> {
              return false;
            }));

    // Driver:
    // _joystick.cross().onTrue(Commands.select(getAcceptCommands(), () -> {return
    // getAcceptId();}));

    _joystick.R2().whileTrue(
        Commands.startEnd(
            () -> withTag = true,
            () -> withTag = false));

    // Semi-Auto:
    // _joystick.cross().onTrue(
    //     Commands.parallel(
    //         _elevator.runClose(),
    //         _rotation.runClose()));

    // _joystick.square().onTrue(
    //     Commands.parallel(
    //         _elevator.runClose(),
    //         _rotation.runOpen(Constants.Rotation.States.kUpRotation)));

    // _joystick.triangle().onTrue(runInOutTake(
    //     Constants.Elevator.States.kAmpOpenHeight, Constants.Rotation.States.kAmpOpenRotation,
    //     _joystick2.getHID()::getTriangleButtonReleased));

    // _joystick.circle().onTrue(runOutake(
    //     Constants.Elevator.States.kTrapOpenHeight, Constants.Rotation.States.kSourceOpenRotation+10,
    //     _joystick.getHID()::getCircleButtonReleased));

    _joystick.L1().onTrue(Commands.runOnce(() -> {
      _swerve.zeroGyro();

      System.out.println("ZEROING");
    }, _swerve));

    // Operator:


    _joystick2.cross().onTrue(
        Commands.parallel(
            _elevator.runClose(),
            _rotation.runClose()));

    _joystick2.square().onTrue(
        Commands.parallel(
            _elevator.runClose(),
            _rotation.runOpen(Constants.Rotation.States.kUpRotation)));

    _joystick2.triangle().onTrue(runInOutTake(
        Constants.Elevator.States.kAmpOpenHeight, Constants.Rotation.States.kAmpOpenRotation,
        _joystick2.getHID()::getTriangleButton));

    _joystick2.circle().onTrue(runOutake(Constants.Elevator.States.kTrapOpenHeight-0.27,
        Constants.Rotation.States.kSourceOpenRotation+30, _joystick2.getHID()::getCircleButtonReleased));

    _joystick2.L2().onTrue(_climber.runClimb());

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
              _elevator.setMotor(-0.3);
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

  public Command runInOutTake(double elevatorHeight, double rotation, BooleanSupplier condition) {
    return Commands.either(
        runIntake(),
        runOutake(elevatorHeight, rotation, condition),
        () -> {
          return !_rollers.isNote();
        });
  }

  public Command runIntake() {
    return Commands.sequence(
        Commands.parallel(
            _elevator.runOpen(Constants.Elevator.States.kSourceOpenHeight),
            _rotation.runOpen(Constants.Rotation.States.kSourceOpenRotation)),

        _rollers.runIntake().until(_rollers::isNote),

        Commands.parallel(
            _elevator.runClose(),
            _rotation.runClose()));
  }

  public Command runOutake(double height, double rotation, BooleanSupplier condition) {
    // return new SequentialStandByCommandGroup(
    // condition,

    // Commands.run(() -> {}),

    // Commands.parallel(
    // _elevator.runOpen(height),
    // _rotation.runOpen(rotation)
    // // Commands.run(() -> {})
    // ),

    // Commands.sequence(
    // _rollers.runIntake().raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake)),

    // Commands.parallel(
    // _elevator.runClose(),
    // _rotation.runOpen(Constants.Rotation.States.kUpRotation)
    // )
    // )
    // );

    return Commands.sequence(
      Commands.parallel(
        _elevator.runOpen(height),
        _rotation.runOpen(rotation)
      ),
      Commands.waitUntil(condition),
      Commands.waitUntil(condition),

      _rollers.runIntake().raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake*3)),
      Commands.waitUntil(condition),

      Commands.parallel(
        _elevator.runClose(),
        _rotation.runOpen(Constants.Rotation.States.kUpRotation))
    );
  }

  public Command runAutoOutake() {
    return Commands.sequence(
        Commands.parallel(
            _elevator.runOpen(Constants.Elevator.States.kAmpOpenHeight),
            _rotation.runOpen(Constants.Rotation.States.kAmpOpenRotation)),

        Commands.waitUntil(
            () -> {
              return _elevator.isHeight(Constants.Elevator.States.kAmpOpenHeight)
                  && _rotation.isRotation(Constants.Rotation.States.kAmpOpenRotation);
            }),

        _rollers.runIntake().raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake)),

        Commands.parallel(
            _elevator.runClose(),
            _rotation.runClose()));
  }

  private Command runTrap(BooleanSupplier condition) {
    // return new SequentialStandByCommandGroup(
    // _joystick2.getHID()::getCircleButtonReleased,

    // Commands.run(() -> {}),

    // _climber.runClimb(),

    // _climber.setHeight(Constants.Climber.kTrapChainHeight),

    // Commands.sequence(
    // Commands.parallel(
    // _elevator.runOpen(0),
    // _rotation.runOpen(Constants.Rotation.States.kUpRotation)
    // // Commands.run(() -> {})
    // ),
    // Commands.parallel(
    // _elevator.runOpen(Constants.Elevator.States.kTrapOpenHeight),
    // _rotation.runOpen(Constants.Rotation.States.kTrapOpenRotation)
    // // Commands.run(() -> {})
    // )
    // ),

    // _climber.runClimb(),

    // _rollers.runIntake()
    // );

    return Commands.sequence(
        _climber.runClimb(),
        Commands.waitUntil(condition),

        _climber.setHeight(Constants.Climber.kTrapChainHeight),
        Commands.waitUntil(condition),

        Commands.sequence(
            Commands.parallel(
                _elevator.runOpen(0),
                _rotation.runOpen(Constants.Rotation.States.kUpRotation)),
            Commands.parallel(
                _elevator.runOpen(Constants.Elevator.States.kTrapOpenHeight),
                _rotation.runOpen(Constants.Rotation.States.kTrapOpenRotation))),
        Commands.waitUntil(condition),

        _climber.runClimb(),
        Commands.waitUntil(condition),

        _rollers.runIntake());
  }

  public HashMap<Integer, Command> getAcceptCommands() {
    HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

    // Source
    _acceptCommands.put(1, runIntake());
    _acceptCommands.put(2, runIntake());
    _acceptCommands.put(9, runIntake());
    _acceptCommands.put(10, runIntake());

    // Amp
    _acceptCommands.put(5,
        runOutake(
            Constants.Elevator.States.kAmpOpenHeight,
            Constants.Rotation.States.kAmpOpenRotation,
            _joystick.getHID()::getTriangleButtonReleased));

    _acceptCommands.put(6,
        runOutake(
            Constants.Elevator.States.kAmpOpenHeight,
            Constants.Rotation.States.kAmpOpenRotation,
            _joystick.getHID()::getTriangleButtonReleased));

    // Stage
    _acceptCommands.put(11, runTrap(_joystick.getHID()::getTriangleButtonReleased));
    _acceptCommands.put(12, runTrap(_joystick.getHID()::getTriangleButtonReleased));
    _acceptCommands.put(13, runTrap(_joystick.getHID()::getTriangleButtonReleased));
    _acceptCommands.put(14, runTrap(_joystick.getHID()::getTriangleButtonReleased));
    _acceptCommands.put(15, runTrap(_joystick.getHID()::getTriangleButtonReleased));
    _acceptCommands.put(16, runTrap(_joystick.getHID()::getTriangleButtonReleased));

    return _acceptCommands;
  }

  private int getAcceptId() {
    // try{
    // Optional<Alliance> ally = DriverStation.getAlliance();
    // int tag = _vision.getTag().ID;

    // AprilTagFieldLayout blue =
    // AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    // blue.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

    // AprilTagFieldLayout red =
    // AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    // red.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

    // if(ally.get() == DriverStation.Alliance.Red)
    // }
    // catch (Exception e) {
    // e.printStackTrace();
    // }

    Optional<Alliance> ally = DriverStation.getAlliance();
    int tag = _vision.getTag().ID;
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

  public Command autoCommand(String auto) {
    PathPlannerPath _path = PathPlannerPath.fromPathFile("LeftToAmp");

    return Commands.sequence(
        Commands.run(() -> {
          _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose());
        }, _swerve).withTimeout(0.5),
        AutoBuilder.buildAuto(auto));
  }

  // public Command Reset() {
  //   _swerve.zeroGyro();
  //   // _swerve.resetOdometry(new Pose2d());

  //   return Commands.parallel(
  //       _elevator.Reset(),
  //       _rotation.Reset(),
  //       _climber.Reset());
  // }
}