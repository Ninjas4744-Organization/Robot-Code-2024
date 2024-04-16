package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

public class TeleopSwerve extends Command {
  private Swerve _swerve;
  private Vision _vision;
  private BooleanSupplier _robotCentricSup;
  BooleanSupplier _withTag;

  private DoubleSupplier _translationSup;
  private DoubleSupplier _strafeSup;
  private DoubleSupplier _rotationSup;

  private PIDController _xController;
  private PIDController _yController;
  private PIDController _aController;

  private SlewRateLimiter _translationRateLimiter;
  private SlewRateLimiter _strafeRateLimiter;
  private SlewRateLimiter _rotationRateLimiter;
  private final double _rangeToCenter = 1.5;
  public TeleopSwerve(
      Swerve swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup) {

    _swerve = swerve;
    _vision = vision;
    _robotCentricSup = robotCentricSup;
    _withTag = withTag;

    _translationSup = translationSup;
    _strafeSup = strafeSup;
    _rotationSup = rotationSup;

    _aController = new PIDController(0.03, 0, 0);
    _xController = new PIDController(1, 0, 0);
    _yController = new PIDController(1, 0, 0);

    _translationRateLimiter = new SlewRateLimiter(3);
    _strafeRateLimiter = new SlewRateLimiter(3);
    _rotationRateLimiter = new SlewRateLimiter(10);

    addRequirements(_swerve, _vision);
  }

  @Override
  public void execute() {
    if(_withTag.getAsBoolean())
      LimelightHelpers.setLEDMode_ForceOn(null);

    Pose2d tagPos = _vision.getTagPose();
    Pose2d Pos = _swerve.getLastCalculatedPosition();
    Pose2d targetPos = tagPos;

    SmartDashboard.putNumber("PosX", Pos.getX());
    SmartDashboard.putNumber("PosY", Pos.getY());
    SmartDashboard.putNumber("Target PosX", targetPos.getX());
    SmartDashboard.putNumber("Target PosY", targetPos.getY());
  
    double aError = targetPos.rotateBy(Rotation2d.fromDegrees(180)).getRotation().minus(Pos.getRotation()).getDegrees();

    // int allianceMinus = _vision.getAlliance() ? 1 : -1;

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(_rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(new PathPoint(new Translation2d(Pos.getX(), Pos.getY())));
    pathPoints.add(new PathPoint(new Translation2d(targetPos.getX(), targetPos.getY() - 2)));

    GoalEndState endState = new GoalEndState(0, targetPos.rotateBy(Rotation2d.fromDegrees(180)).getRotation());

    PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, AutoConstants.constraints, endState);
    AutoBuilder.followPath(path);

    // translationVal = _withTag.getAsBoolean() && LimelightHelpers.getTV(null)
    // ? _xController.calculate(Pos.getX(), targetPos.getX()) :
    //   _translationRateLimiter.calculate(translationVal);

    // strafeVal = _withTag.getAsBoolean() && LimelightHelpers.getTV(null)
    // ? _yController.calculate(Pos.getY(), targetPos.getY()) :
    //   _strafeRateLimiter.calculate(strafeVal);

    // rotationVal = _withTag.getAsBoolean()
    // ? -_aController.calculate(aError)
    // : /*_rotationRateLimiter.calculate(*/rotationVal/*)*/;
    
    _swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity,
      !_robotCentricSup.getAsBoolean(),
      true
    );
  }
}