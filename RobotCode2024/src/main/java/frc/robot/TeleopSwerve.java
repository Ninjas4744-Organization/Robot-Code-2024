package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve _swerve;
  private Vision _vision;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private PIDController _controller_x;
  private PIDController _controller_theta_pid;
  private SlewRateLimiter _translationRateLimiter;
  private SlewRateLimiter _strafeRateLimiter;
  private SlewRateLimiter _rotationRateLimiter;

  public TeleopSwerve(
      Swerve swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this._vision = vision;
    _controller_theta_pid = new PIDController(0.01111111 * 3.5, 0, 0);
    _controller_x = new PIDController(0.6666666666666667 * 2.5, 0, 0.1);
    this._withTag = withTag;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    _translationRateLimiter = new SlewRateLimiter(3);
    _strafeRateLimiter = new SlewRateLimiter(3);
    _rotationRateLimiter = new SlewRateLimiter(3);

    addRequirements(_swerve);
  }

  @Override
  public void execute() {
    Pose2d currentPos = _swerve.getLastCalculatedPosition();
    Pose2d targetPose = _vision.getTagPose();
    SmartDashboard.putNumber("X distance", targetPose.getX() - currentPos.getX());
    
    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(_rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    translationVal = _translationRateLimiter.calculate(
        _withTag.getAsBoolean() && inrange(targetPose, current_pos) && _vision.isRelaventTag()
            ? -_controller_x.calculate(targetPose.getX() - current_pos.getX())
            : translationVal * (_withTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));

    strafeVal = _strafeRateLimiter.calculate(
        _withTag.getAsBoolean() && inrange(targetPose, current_pos) && _vision.isRelaventTag()
            ? MathUtil.clamp(strafeVal, -0.3, 0.3)
            : strafeVal * (_withTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));

    rotationVal = _rotationRateLimiter.calculate(_withTag.getAsBoolean()
        ? -_controller_theta_pid.calculate(
            targetPose.rotateBy(Rotation2d.fromDegrees(180))
                .getRotation().minus(current_pos.getRotation()).getDegrees())
        : rotationVal * Constants.Swerve.maxAngularVelocity);

    _swerve.drive(
        new Translation2d(
            translationVal,
            strafeVal).times(Constants.Swerve.maxSpeed).rotateBy(_withTag.getAsBoolean() && !_vision.isAmp()? targetPose.getRotation():Rotation2d.fromDegrees(0)),
            rotationVal,
        !robotCentricSup.getAsBoolean(),
        true);

  }

  private boolean inrange(Pose2d targetPose, Pose2d current_pos) {
    return targetPose.getX() - current_pos.getX() < 1.5;
  }
}