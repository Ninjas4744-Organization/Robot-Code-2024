package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve _swerve;
  private Vision _vision;
  private BooleanSupplier _robotCentricSup;
  private BooleanSupplier _whileTag;
  private BooleanSupplier _onTag;
  private Pose2d _targetPose = new Pose2d();

  private DoubleSupplier _translationSup;
  private DoubleSupplier _strafeSup;
  private DoubleSupplier _rotationSup;

  private PIDController _xController;
  private PIDController _yController;
  private PIDController _0Controller;

  private SlewRateLimiter _translationRateLimiter;
  private SlewRateLimiter _strafeRateLimiter;
  private SlewRateLimiter _rotationRateLimiter;

  public TeleopSwerve(
      Swerve swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier whileTag,
      BooleanSupplier onTag,
      BooleanSupplier robotCentricSup) {

    _swerve = swerve;
    _vision = vision;
    _robotCentricSup = robotCentricSup;
    _whileTag = whileTag;
    _onTag = onTag;
    
    _translationSup = translationSup;
    _strafeSup = strafeSup;
    _rotationSup = rotationSup;

    _0Controller = new PIDController(0.01111111 * 3.5, 0, 0);
    _xController = new PIDController(0.6666666666666667 * 2.5, 0, 0.1);
    _yController = new PIDController(0.6666666666666667 * 2.5, 0, 0.1);
    
    _translationRateLimiter = new SlewRateLimiter(3);
    _strafeRateLimiter = new SlewRateLimiter(3);
    _rotationRateLimiter = new SlewRateLimiter(3);

    addRequirements(_swerve, _vision);
  }

  @Override
  public void execute() {
    Pose2d tagPose = _vision.getTagPose();
    Pose2d currentPose = _swerve.getLastCalculatedPosition();

    // Vision Target Pose
    double distFromTag = currentPose.getTranslation().getDistance(tagPose.getTranslation());
    if(_onTag.getAsBoolean()){
      Rotation2d tagDir = tagPose.getRotation().plus(Rotation2d.fromDegrees(90)); //Getting normal
      Translation2d distTargetFromTag = new Translation2d(
        distFromTag * Math.cos(tagDir.getRadians()),
        distFromTag * Math.sin(tagDir.getRadians())
      );
      _targetPose = new Pose2d(
        tagPose.getX() + distTargetFromTag.getX(),
        tagPose.getY() + distTargetFromTag.getY(),
        tagDir.plus(Rotation2d.fromDegrees(180))
      );
    }
    
    // Apply Deadband
    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(_rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    // Final Values
    translationVal = _translationRateLimiter.calculate(
        _whileTag.getAsBoolean() && inrange(tagPose, currentPose) && _vision.isRelaventTag()
            ? _yController.calculate(currentPose.getY(), _targetPose.getY()) * Constants.Swerve.kActionCoefficient
            + translationVal * Math.sin(_targetPose.getRotation().getRadians()) * Constants.Swerve.kActionCoefficient
            : translationVal * (_whileTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));

    strafeVal = _strafeRateLimiter.calculate(
        _whileTag.getAsBoolean() && inrange(tagPose, currentPose) && _vision.isRelaventTag()
            ? _xController.calculate(currentPose.getX(), _targetPose.getX()) * Constants.Swerve.kActionCoefficient
            + translationVal * Math.cos(_targetPose.getRotation().getRadians()) * Constants.Swerve.kActionCoefficient
            : strafeVal * (_whileTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));

    rotationVal = _rotationRateLimiter.calculate(_whileTag.getAsBoolean()
        ? -_0Controller.calculate(_targetPose.getRotation().getDegrees())
        : rotationVal * Constants.Swerve.maxAngularVelocity);

    _swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      rotationVal,
      !_robotCentricSup.getAsBoolean(),
      true
    );
    
    // Dashboard
    SmartDashboard.putNumber("Target Pose X", _targetPose.getX());
    SmartDashboard.putNumber("Target Pose Y", _targetPose.getY());
    SmartDashboard.putNumber("Dist From Tag", distFromTag);
    // SmartDashboard.putNumber("X PID", _xController.calculate(currentPose.getX(), _targetPose.getX()));
    // SmartDashboard.putNumber("Y PID", _yController.calculate(currentPose.getY(), _targetPose.getY()));
    // SmartDashboard.putNumber("X Movement", translationVal * Math.cos(_targetPose.getRotation().getRadians()));
    // SmartDashboard.putNumber("Y Movement", translationVal * Math.sin(_targetPose.getRotation().getRadians()));
    // SmartDashboard.putNumber("X Final Movement", strafeVal);
    // SmartDashboard.putNumber("Y Final Movement", translationVal);
    // SmartDashboard.putNumber("X distance", tagPose.getX() - currentPose.getX());
    // SmartDashboard.putNumber("Y distance", tagPose.getY() - currentPose.getY());
    // SmartDashboard.putNumber("Tag X", tagPose.getX());
    // SmartDashboard.putNumber("Tag Y", tagPose.getY());
    // SmartDashboard.putNumber("Tag 0", tagPose.getRotation().getDegrees());
  }

  private boolean inrange(Pose2d tagPose, Pose2d currentPose) {
    double distFromTag = currentPose.getTranslation().getDistance(tagPose.getTranslation());
    return distFromTag < 1.5;
  }
}