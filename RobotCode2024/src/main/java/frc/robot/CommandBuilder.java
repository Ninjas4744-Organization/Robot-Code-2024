package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake.Lift;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.Rotation;

public class CommandBuilder {
 

  public static Command runInOutTake( Lift _elevator ,Rotation _rotation, Rollers _rollers, BooleanSupplier condition) {
    return Commands.either(
      runIntake(_elevator, _rotation, _rollers),
      runOutakeAmp(_elevator, _rotation, _rollers, condition),
      () -> { return !_rollers.isNote(); }
    );
  }
  public Command runInOutT(double elevatorHeight, double rotation, BooleanSupplier condition) {
    return Commands.none();
  }

  public static Command runIntake( Lift _elevator ,Rotation _rotation, Rollers _rollers) {
    return Commands.sequence(
      Commands.parallel(
        _elevator.runProfile(new State(Constants.Elevator.States.kSourceOpenHeight,0)),
        _rotation.runOpenClose(Constants.Rotation.States.kSourceOpenRotation)
      ),

      _rollers.runIntake().until(_rollers::isNote),

      Commands.runOnce(
        () ->{
          _elevator.close();
        _rotation.runOpenClose(Constants.Rotation.States.kUpRotation);
        },
        _elevator,_rollers,_rotation
      )
    );
  }

  public static Command runOutakeAmp(Lift _elevator ,Rotation _rotation, Rollers _rollers, BooleanSupplier condition) {
    return Commands.sequence(
      Commands.parallel(
        _elevator.openAmp(),
        _rotation.runOpenClose(Constants.Rotation.States.kAmpOpenRotation)
      ),
      Commands.waitUntil(condition),

      _rollers.runIntake(),

      Commands.parallel(
        _elevator.close(),
        _rotation.runOpenClose(Constants.Rotation.States.kUpRotation)

      )
    );
  }
  public static Command runOutakeTrap(Lift _elevator ,Rotation _rotation, Rollers _rollers, BooleanSupplier condition) {
    return Commands.sequence(
      Commands.parallel(
        _elevator.openAmp(),
        _rotation.runOpenClose(Constants.Rotation.States.kTrapOpenRotation)
      ),
      Commands.waitUntil(condition),

      _rollers.runIntake(),

      Commands.parallel(
        _elevator.close(),
        _rotation.runOpenClose(Constants.Rotation.States.kUpRotation)

      )
    );
  }


  public Command autoCommand(String auto) {
    // PathPlannerPath _path = PathPlannerPath.fromPathFile("Score");
    // _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose());

    return AutoBuilder.buildAuto(auto);
  }

  public static Command reset(Lift _elevator ,Rotation _rotation, Rollers _rollers,Climber _climber,Swerve _swerve) {
    _swerve.zeroGyro();
    _swerve.resetOdometry(new Pose2d());

    return Commands.parallel(
      _elevator.reset(),
      _rotation.reset(),
      _climber.reset()
  );
  }
}