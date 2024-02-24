package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  // Subsystems
  private final Swerve _swerve;
  private final Vision _vision;
  // private final Climber _climber;
  // private final IntakeElevator _intakeElevator;
  // private final IntakeRotation _intakeRotation;
  // private final IntakeRollers _intakeRollers;
  // private final FloorIntakeRollers _floorIntakeRollers;
  // private final FloorIntakeRotation _floorIntakeRotation;

  // Other
  private final CommandPS5Controller _joystick;
  // private final CommandPS5Controller _joystick2;
  // private boolean withTag = false;
  // private double coefficientFactor = 1;

  public RobotContainer() {
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    // _joystick2 = new CommandPS5Controller(Ports.Joystick.kJoystick2Port);
    _vision = new Vision();
    _swerve = new Swerve(_vision::estimationsSupplier);

    // _climber = new Climber();
    // // _intakeElevator = new IntakeElevator();
    // _intakeRotation = new IntakeRotation();
    // _intakeRollers = new IntakeRollers();
    // _floorIntakeRollers = new FloorIntakeRollers();
    // _floorIntakeRotation = new FloorIntakeRotation();

    // NamedCommands.registerCommand("Outake",
    // runAutonomyOutake(IntakeStates.kAmpOpenHeight,
    // IntakeStates.kAmpOpenRotation));

    configureBindings();
  }

  private void configureBindings() {
    _joystick.getHID().getCircleButtonPressed();
    // Driving:
    // _swerve.setDefaultCommand(
    // new TeleopSwerve(
    // _swerve,
    // _vision,
    // () -> {return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient *
    // coefficientFactor - _joystick2.getLeftY() *
    // Constants.Swerve.kDriveCoefficient * coefficientFactor;},
    // () -> {return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient *
    // coefficientFactor - _joystick2.getLeftX() *
    // Constants.Swerve.kDriveCoefficient * coefficientFactor;},
    // () -> {return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient *
    // coefficientFactor - _joystick2.getRightX() *
    // Constants.Swerve.kDriveCoefficient * coefficientFactor;},
    // () -> {return withTag;},
    // () -> {return false;}
    // )
    // );

    // Auto:
    // _joystick.cross().onTrue(Commands.select(getAcceptCommands(), () -> {
    // return getAcceptId();
    // }));

    // _joystick.circle().onTrue(Override());

    // _joystick.R2().whileTrue(Commands.startEnd(() -> withTag = true, () ->
    // withTag = false));

    // //Semi-Auto:
    // _joystick.cross().onTrue(Commands.parallel(
    // // _intakeElevator.runClose(),
    // _intakeRotation.runClose())
    // );

    // _joystick.square().onTrue(Commands.parallel(
    // // _intakeElevator.runClose(),
    // _intakeRotation.runOpen(IntakeStates.kGameStartRotation)
    // ));

    // _joystick.triangle().onTrue(runInOutTake(
    // IntakeStates.kAmpOpenHeight, IntakeStates.kAmpOpenRotation)
    // );

    // _joystick.circle().onTrue(runAutoOutakeTrap(
    // IntakeStates.kTrapOpenHeight, IntakeStates.kTrapOpenRotation)
    // );

    // _joystick.povUp().whileTrue(Commands.startEnd(
    // () -> {_climber.setMotor(1);},
    // () -> {_climber.stopMotor();}, _climber)
    // );

    // _joystick.povDown().whileTrue(Commands.startEnd(
    // () -> {_climber.setMotor(-1);},
    // () -> {_climber.stopMotor();}, _climber)
    // );

    // _joystick.povUp().onTrue(
    // _climber.runElevateUp()
    // );

    // _joystick.povDown().onTrue(Commands.sequence(
    // _climber.runElevateDown(),
    // runAutoOutake(Constants.IntakeStates.kTrapOpenHeight,
    // Constants.IntakeStates.kTrapOpenRotation)
    // ));

    // _joystick.circle().onTrue(
    // Commands.either(
    // _climber.runElevateUp(),
    // Commands.run(() -> {}),
    // () -> { return _climber.getHeight() == 0; })
    // );

    // _joystick.circle().whileTrue(
    // Commands.startEnd(
    // () -> { _climber.setMotor(-0.7); },
    // () -> { _climber.setMotor(0); } ,
    // _climber
    // )
    // );

    // Manual:
    // _joystick2.povRight().whileTrue(Commands.startEnd(
    // () -> { _intakeRollers.intake();},
    // () -> {_intakeRollers.stopTake();},
    // _intakeRollers)
    // );

    // _joystick2.povLeft().whileTrue(Commands.startEnd(
    // () -> {_intakeRollers.outake();},
    // () -> {_intakeRollers.stopTake();},
    // _intakeRollers)
    // );

    // _joystick2.povUp().whileTrue(Commands.startEnd(
    // () -> {_climber.setMotor(1);},
    // () -> {_climber.stopMotor();}, _climber)
    // );

    // _joystick2.povDown().whileTrue(Commands.startEnd(
    // () -> {_climber.setMotor(-1);},
    // () -> {_climber.stopMotor();}, _climber)
    // );

    // _joystick2.R2().whileTrue(Commands.startEnd(
    // () -> {_intakeElevator.setElevatorMotor(0.2);},
    // () -> {_intakeElevator.stopElevator();},
    // _intakeElevator)
    // );

    // _joystick2.L2().whileTrue(Commands.startEnd(
    // () -> {_intakeElevator.setElevatorMotor(-0.2);},
    // () -> {_intakeElevator.stopElevator();},
    // _intakeElevator)
    // );

    // _joystick2.R1().whileTrue(Commands.startEnd(
    // () -> {_intakeRotation.setRotationMotor(0.07);},
    // () -> {_intakeRotation.stopRotation();},
    // _intakeRotation)
    // );

    // _joystick2.L1().whileTrue(Commands.startEnd(
    // () -> {_intakeRotation.setRotationMotor(-0.07);},
    // () -> {_intakeRotation.stopRotation();},
    // _intakeRotation)
    // );
  }

  // private Command runAutoClimb() {
  // return Commands.sequence(
  // _climber.runElevateUp(),
  // _intakeElevator.runOpen(Constants.IntakeStates.kTrapOpenHeight),
  // _intakeRotation.runOpen(Constants.IntakeStates.kTrapOpenRotation),
  // Commands.waitUntil(() -> {
  // return _climber.isMax();
  // }),
  // _intakeRollers.runOutake());
  // }

  // private Command runAutoIntake() {
  // return Commands.sequence(
  // Commands.parallel(
  // // _intakeElevator.runOpen(Constants.IntakeStates.kSourceOpenHeight),
  // _intakeRotation.runOpen(Constants.IntakeStates.kSourceOpenRotation)
  // ),
  // Commands.waitUntil(() -> {
  // return //_intakeElevator.isMax(Constants.IntakeStates.kSourceOpenHeight)
  // _intakeRotation.isMax(Constants.IntakeStates.kSourceOpenRotation);
  // }),
  // _intakeRollers.runIntake(),
  // Commands.waitUntil(() -> {return _intakeRollers.isNote();}),
  // Commands.parallel(
  // // _intakeElevator.runClose(),
  // _intakeRotation.runClose()
  // )
  // );
  // }

  // private Command runAutoOutake(double height, double rotation) {
  // return Commands.sequence(
  // Commands.parallel(
  // //_intakeElevator.runOpen(height),
  // _intakeRotation.runOpen(rotation)
  // //Commands.run(() -> {coefficientFactor = 0.25;})
  // ),
  // Commands.waitUntil(() -> {
  // return _joystick.triangle().getAsBoolean();
  // }),
  // _intakeRollers.runOutake(),
  // Commands.waitSeconds(Constants.kTimeToOutake),
  // Commands.parallel(
  // //_intakeElevator.runClose(),
  // _intakeRotation.runClose()
  // //Commands.run(() -> {coefficientFactor = 1;})
  // )
  // );
  // }

  // private Command runAutonomyOutake(double height, double rotation) {
  // return Commands.sequence(
  // Commands.parallel(
  // // _intakeElevator.runOpen(height),
  // _intakeRotation.runOpen(rotation)
  // ),
  // Commands.waitUntil(() -> {
  // return /*_intakeElevator.isMax(height) &&*/ _intakeRotation.isMax(rotation);
  // }),
  // _intakeRollers.runOutake(),
  // Commands.waitSeconds(Constants.kTimeToOutake),
  // Commands.parallel(
  // // _intakeElevator.runClose(),
  // _intakeRotation.runClose()
  // )
  // );
  // }

  // private Command runAutoOutakeTrap(double height, double rotation) {
  // return Commands.sequence(
  // Commands.parallel(
  // // _intakeElevator.runOpen(height),
  // _intakeRotation.runOpen(rotation)
  // //Commands.run(() -> {coefficientFactor = 0.25;})
  // ),
  // Commands.waitUntil(() -> {
  // return _joystick.circle().getAsBoolean();
  // }),
  // _intakeRollers.runOutake(),
  // Commands.waitSeconds(Constants.kTimeToOutake),
  // Commands.parallel(
  // // _intakeElevator.runClose(),
  // _intakeRotation.runClose()
  // //Commands.run(() -> {coefficientFactor = 1;})
  // )
  // );
  // }

  // public Command runInOutTake(double height, double rotation) {
  // return Commands.either(
  // runAutoIntake(),
  // runAutoOutake(height, rotation),
  // () -> { return !(_intakeRollers.isNote());
  // }
  // );
  // }

  // private Command runAutoFloorIntake(){
  // return Commands.sequence(
  // _floorIntakeRotation.runClose(),
  // Commands.waitUntil(() -> { return _floorIntakeRotation.getRotation() == 0;
  // }),
  // _floorIntakeRollers.runIntake()
  // ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command runAutoFloorOutake(){
  // return Commands.sequence(
  // _floorIntakeRotation.runOpen(),
  // Commands.waitUntil(() -> { return _floorIntakeRotation.isMax(); }),
  // _floorIntakeRollers.runOutake()
  // ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command Override() {
  // return Commands.parallel(
  // Commands.run(() -> {_swerve.Override();}, _swerve),
  // // Commands.run(() -> {_climber.Reset();}, _climber));
  // // Commands.run(() -> {_intakeElevator.Override();}, _intakeElevator),
  // Commands.run(() -> {_intakeRotation.Override();}, _intakeRotation)
  // // Commands.run(() -> {_intakeRollers.Reset();}, _intakeRollers));
  // // _floorIntakeRollers.Override();
  // // _floorIntakeRotation.Override();
  // );
  // }

  // public Command Reset() {
  // _swerve.zeroGyro();
  // _swerve.resetOdometry(new Pose2d());
  // );
  // }
}