package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Ports;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FloorIntakeRollers;
import frc.robot.subsystems.FloorIntakeRotation;
import frc.robot.subsystems.IntakeElevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeRotation;
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
  private final CommandPS5Controller _joystick2;
  private boolean override = false;
  private boolean withTag = false;

  public RobotContainer() { 
    _joystick = new CommandPS5Controller(Ports.Joystick.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Ports.Joystick.kJoystick2Port);
    _swerve = new Swerve();
    _vision = new Vision();
    // _climber = new Climber();
    // _intakeElevator = new IntakeElevator();
    // _intakeRotation = new IntakeRotation();
    // _intakeRollers = new IntakeRollers();
    // _floorIntakeRollers = new FloorIntakeRollers();
    // _floorIntakeRotation = new FloorIntakeRotation();

    //NamedCommands.registerCommand("Outake", _intakeRollers.runOutake());
    //NamedCommands.registerCommand("Intake", _intakeRollers.runIntake());
    //NamedCommands.registerCommand("Floor Outake", runAutoFloorOutake());
    //NamedCommands.registerCommand("Floor Intake", runAutoFloorIntake());

    configureBindings();
  }

  // public HashMap<Integer, Command> getAcceptCommands() {
  //   HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

  //   for(int i = 11; i <= 16; i++)
  //     _acceptCommands.put(i, runAutoClimb());

  //   _acceptCommands.put(1, runAutoIntake());
  //   _acceptCommands.put(2, runAutoIntake());
  //   _acceptCommands.put(9, runAutoIntake());
  //   _acceptCommands.put(10,runAutoIntake());
  //   _acceptCommands.put(5, runAutoOutake(Constants.IntakeStates.kAmpOpenHeight, Constants.IntakeStates.kAmpOpenRotation));
  //   _acceptCommands.put(6, runAutoOutake(Constants.IntakeStates.kAmpOpenHeight, Constants.IntakeStates.kAmpOpenRotation));

  //   return _acceptCommands;
  // }
  
  private void configureBindings() {
    //Driving:
    _swerve.setDefaultCommand(
      new TeleopSwerve(
          _swerve,
          _vision,
          () -> { return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient /*-_joystick2.getLeftY() * Constants.Swerve.kDriveCoefficient*/; },
          () -> { return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient /*-_joystick2.getLeftX() * Constants.Swerve.kDriveCoefficient*/; },
          () -> { return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient /*-_joystick2.getRightX() * Constants.Swerve.kDriveCoefficient*/; },
          () -> { return withTag; },
          () -> { return false; }
      )
    );

    //Auto:
    // _joystick.cross().onTrue(Commands.select(getAcceptCommands(), () -> { return getAcceptId(); }));
    // _joystick.circle().onTrue(Commands.run(() -> { Override(); }));
    // _joystick.triangle().toggleOnTrue(Commands.startEnd(() -> { runAutoFloorIntake().execute(); }, () -> { runAutoFloorOutake().execute(); }));
    _joystick.R2().whileTrue(Commands.run(() -> withTag = true));
    _joystick.R2().whileFalse(Commands.run(() -> withTag = false));
    
    //Semi-Auto:
    // _joystick.square().onTrue(Commands.sequence(
    //   _intakeElevator.runClose(),
    //   _intakeRotation.runClose()
    // ));
    // _joystick.povUp().toggleOnTrue(_climber.runElevate());
    // _joystick.povDown().onTrue(runAutoIntake());
    // _joystick.povRight().onTrue(runAutoOutake(Constants.IntakeStates.kAmpOpenHeight, Constants.IntakeStates.kAmpOpenRotation));
    // _joystick.povLeft().onTrue(runAutoOutake(Constants.IntakeStates.kTrapOpenHeight, Constants.IntakeStates.kTrapOpenRotation));

    //Manual:
    // _joystick2.L2().whileTrue(      Commands.startEnd(() -> { _intakeRollers.intake(); }, () -> { _intakeRollers.stopTake(); }, _intakeRollers));
    // _joystick2.R2().whileTrue(      Commands.startEnd(() -> { _intakeRollers.outake(); }, () -> { _intakeRollers.stopTake(); }, _intakeRollers));
    // _joystick2.povUp().whileTrue(   Commands.startEnd(() -> { _climber.setMotor(1); }, () -> { _climber.Reset(); }, _climber));
    // _joystick2.povDown().whileTrue( Commands.startEnd(() -> { _climber.setMotor(-1); }, () -> { _climber.Reset(); }, _climber));
    // _joystick2.L1().whileTrue(      Commands.startEnd(() -> { _intakeElevator.setElevatorMotor(-1); }, () -> { _intakeElevator.stopElevator(); }, _intakeElevator));
    // _joystick2.R1().whileTrue(      Commands.startEnd(() -> { _intakeElevator.setElevatorMotor(1); }, () -> { _intakeElevator.stopElevator(); }, _intakeElevator));
    // _joystick2.triangle().whileTrue(Commands.startEnd(() -> { _intakeRotation.setRotationMotor(1); }, () -> { _intakeRotation.stopRotation(); }, _intakeRotation));
    // _joystick2.cross().whileTrue(   Commands.startEnd(() -> { _intakeRotation.setRotationMotor(-1); }, () -> { _intakeRotation.stopRotation(); }, _intakeRotation));
  }

  public void periodic(){
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = _swerve.getLastCalculatedPosition();
    SmartDashboard.putNumber("distance", Math.hypot(targetPose.getX() - current_pos.getX(),targetPose.getY() - current_pos.getY()));
  }

  // private Command runAutoClimb(){
  //   return Commands.sequence(
  //     _climber.runElevateUp(),
  //     _intakeElevator.runOpen(Constants.IntakeStates.kTrapOpenHeight),
  //     _intakeRotation.runOpen(Constants.IntakeStates.kTrapOpenRotation),
  //     Commands.waitUntil(() -> { return _climber.isMax(); }),
  //     _intakeRollers.runOutake()
  //   ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command runAutoIntake(){
  //   return Commands.sequence(
  //     _intakeElevator.runOpen(Constants.IntakeStates.kSourceOpenHeight),
  //     _intakeRotation.runOpen(Constants.IntakeStates.kSourceOpenRotation),
  //     Commands.waitUntil(() -> { return _intakeElevator.isMax(Constants.IntakeStates.kSourceOpenHeight) 
  //       && _intakeRotation.isMax(Constants.IntakeStates.kSourceOpenRotation); }),
  //     _intakeRollers.runIntake()
  //   ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command runAutoOutake(double height, double rotation){
  //   return Commands.sequence(
  //     _intakeElevator.runOpen(height),
  //     _intakeRotation.runOpen(rotation),
  //     Commands.waitUntil(() -> { return _intakeElevator.isMax(height) && _intakeRotation.isMax(rotation); }),
  //     _intakeRollers.runOutake()
  //   ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command runAutoFloorIntake(){
  //   return Commands.sequence(
  //     _floorIntakeRotation.runClose(),
  //     Commands.waitUntil(() -> { return _floorIntakeRotation.getRotation() == 0; }),
  //     _floorIntakeRollers.runIntake()
  //   ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  // private Command runAutoFloorOutake(){
  //   return Commands.sequence(
  //     _floorIntakeRotation.runOpen(),
  //     Commands.waitUntil(() -> { return _floorIntakeRotation.isMax(); }),
  //     _floorIntakeRollers.runOutake()
  //   ).until(() -> { return override; }).andThen(() -> { override = false; });
  // }

  private void Override(){
    _swerve.Override();
    // _intakeElevator.Override();
    // _intakeRotation.Override();
    // _intakeRollers.Override();
    // _climber.Override();
    // _floorIntakeRollers.Override();
    // _floorIntakeRotation.Override();
    override = true;
  }

  private int getTagID(){
    return _vision.getTag().ID;
  }

  private int getAcceptId(){
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
    // _climber.Reset();
    // _intakeElevator.Reset();
    // _intakeRotation.Reset();
    // _intakeRollers.Reset();
    // _floorIntakeRollers.Reset();
    // _floorIntakeRotation.Reset();
    _swerve.zeroGyro();
    _swerve.resetOdometry(new Pose2d());
  }

  public Boolean isNotTarget() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = _swerve.getLastCalculatedPosition();
    return !LimelightHelpers.getTV(null) ||
    Math.hypot(targetPose.getX() - current_pos.getX(),targetPose.getY() - current_pos.getY()) > 2;
  }

  public Command autoCommand(String auto){
    PathPlannerPath _path = PathPlannerPath.fromPathFile("Amp");
    
    return Commands.sequence(
      Commands.run(() -> { _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose()); }, _swerve).withTimeout(0.5),
      AutoBuilder.buildAuto("Go")
    );
  }
}