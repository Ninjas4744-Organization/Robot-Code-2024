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
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private PIDController _controller;
  private PIDController _controller_x;
  BooleanSupplier _withTag;
  SlewRateLimiter _translationRateLimiter;
  SlewRateLimiter _strafeRateLimiter;
  SlewRateLimiter _rotationRateLimiter;

  public TeleopSwerve(
      Swerve _swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup
      ) {

    this._swerve = _swerve;
    this._vision = vision;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this._withTag = withTag;
    this.robotCentricSup = robotCentricSup;
    
    _controller = new PIDController(0.01111111*4, 0, 0);
    _controller_x = new PIDController(0.6666666666666667*2, 0, 0.1);
    _translationRateLimiter = new SlewRateLimiter(2);//2
    _strafeRateLimiter = new SlewRateLimiter(2);//2
    _rotationRateLimiter = new SlewRateLimiter(1.5);//2

    addRequirements(_swerve);
  }

  @Override
  public void execute() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = _swerve.getLastCalculatedPosition();
    SmartDashboard.putNumber("distance", Math.hypot(targetPose.getX() - current_pos.getX(),targetPose.getY() - current_pos.getY()));

    /* Get Values, Deadband*/
    double translationVal =
      MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal =
      MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal =
      MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    
    translationVal = _translationRateLimiter.calculate(translationVal);
    strafeVal = _strafeRateLimiter.calculate(strafeVal);
    rotationVal = _rotationRateLimiter.calculate(rotationVal);

    _swerve.drive(
        new Translation2d(translationVal,
         _withTag.getAsBoolean() && inrange(targetPose,current_pos) ? 
         -_controller_x.calculate(targetPose.getX() - current_pos.getX() + 0.1) : strafeVal).times(Constants.Swerve.maxSpeed),
         _withTag.getAsBoolean() ?
         -_controller.calculate(targetPose.
         rotateBy(Rotation2d.fromDegrees(180)).
         minus(current_pos).getRotation().getDegrees())
         : rotationVal * Constants.Swerve.maxAngularVelocity,
         !robotCentricSup.getAsBoolean(),
        true
    );
  }

  private boolean inrange(Pose2d targetPose, Pose2d current_pos) {
    return targetPose.getX() - current_pos.getX() < 1.5;
  }
}