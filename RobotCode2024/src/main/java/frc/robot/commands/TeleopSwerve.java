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
  private DoubleSupplier _rotationXSup;
  private DoubleSupplier _rotationYSup;

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
      DoubleSupplier rotationXSup,
      DoubleSupplier rotationYSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup) {

    _swerve = swerve;
    _vision = vision;
    _robotCentricSup = robotCentricSup;
    _withTag = withTag;

    _translationSup = translationSup;
    _strafeSup = strafeSup;
    _rotationXSup = rotationXSup;
    _rotationYSup = rotationYSup;

    _aController = new PIDController(0.2, 0, 2);
    _aController.enableContinuousInput(-1.5 * Math.PI, 0.5 * Math.PI);
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
    
    double currentRotation = _swerve.getYaw().getRadians();
    double targetRotation = currentRotation;
    if(Math.abs(_rotationYSup.getAsDouble()) >= 0.1 || Math.abs(_rotationXSup.getAsDouble()) >= 0.1)
      targetRotation = -Math.atan2(_rotationYSup.getAsDouble(), _rotationXSup.getAsDouble()) + 0.5 * Math.PI;

    //add snap to 45 angles

    SmartDashboard.putNumber("TargetRotation", targetRotation * 57.29578);
    SmartDashboard.putNumber("CurrentRotation", currentRotation * 57.29578);
    SmartDashboard.putNumber("RotationPID", _aController.calculate(currentRotation, targetRotation));

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = _aController.calculate(currentRotation, targetRotation);

    ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(new PathPoint(new Translation2d(Pos.getX(), Pos.getY())));
    pathPoints.add(new PathPoint(new Translation2d(targetPos.getX(), targetPos.getY() - 2)));

    GoalEndState endState = new GoalEndState(0, targetPos.rotateBy(Rotation2d.fromDegrees(180)).getRotation());

    PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, AutoConstants.constraints, endState);
    AutoBuilder.followPath(path);
    
    _swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity,
      !_robotCentricSup.getAsBoolean(),
      true
    );
  }
}