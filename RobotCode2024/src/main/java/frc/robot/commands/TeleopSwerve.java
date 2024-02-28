package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  private Swerve s_Swerve;
  private Vision _vision;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private ProfiledPIDController _theta_controller;
  private PIDController _controller_x;
  private PIDController _controller_theta_pid;
  private SlewRateLimiter _translationRateLimiter;
  private SlewRateLimiter _strafeRateLimiter;
  private SlewRateLimiter _rotationRateLimiter;

  BooleanSupplier _withTag;

  public TeleopSwerve(
      Swerve s_Swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this._vision = vision;
    _controller_theta_pid = new PIDController(0.01111111 * 3.5, 0, 0);
    _theta_controller = new ProfiledPIDController(0.01111111 *3, 0, 0,
        Constants.AutoConstants.kThetaControllerConstraints);
    _controller_x = new PIDController(0.6666666666666667 * 2.5, 0, 0.1);
    this._withTag = withTag;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    
    _translationRateLimiter = new SlewRateLimiter(1);
    _strafeRateLimiter = new SlewRateLimiter(1);
    _rotationRateLimiter = new SlewRateLimiter(1);

    addRequirements(s_Swerve, _vision);
  }

  @Override
  public void execute() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = s_Swerve.getLastCalculatedPosition();
    // SmartDashboard.putNumber("X error",
    //     _controller_x.calculate(targetPose.getX() - current_pos.getX()));
    SmartDashboard.putNumber("X distance",
        targetPose.getX() - current_pos.getX());
    
    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    translationVal = _translationRateLimiter.calculate(translationVal * (_withTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));
    strafeVal = _strafeRateLimiter.calculate(strafeVal * (_withTag.getAsBoolean() ? Constants.Swerve.kActionCoefficient : 1));
    rotationVal = _rotationRateLimiter.calculate(rotationVal);

    s_Swerve.drive(
        new Translation2d(
          _withTag.getAsBoolean() && inrange(targetPose, current_pos)?-_controller_x.calculate(targetPose.getX() - current_pos.getX()):
          translationVal,
            _withTag.getAsBoolean() && inrange(targetPose, current_pos)
                ?MathUtil.clamp(strafeVal, -0.3, 0.3) 
                : 
                strafeVal)
            .times(Constants.Swerve.maxSpeed),
        _withTag.getAsBoolean()
            ? -_controller_theta_pid.calculate(
                targetPose.rotateBy(Rotation2d.fromDegrees(180)).getRotation().minus(current_pos.getRotation()).getDegrees())
            : rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
    
  }

  private boolean inrange(Pose2d targetPose, Pose2d current_pos) {
    return targetPose.getX() - current_pos.getX() < 1.5;
  }
}
