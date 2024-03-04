package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve _swerve;
  private Vision _vision;
  private BooleanSupplier _robotCentricSup;
  BooleanSupplier _withTag;

  private DoubleSupplier _translationSup;
  private DoubleSupplier _strafeSup;
  private DoubleSupplier _rotationSup;

  private PIDController _controller_x;
  private PIDController _controller_theta_pid;

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

    _controller_theta_pid = new PIDController(0.01111111 * 3.5, 0, 0);
    _controller_x = new PIDController(0.6666666666666667 * 2.5, 0, 0.1);

    _translationRateLimiter = new SlewRateLimiter(3);
    _strafeRateLimiter = new SlewRateLimiter(3);
    _rotationRateLimiter = new SlewRateLimiter(10);

    addRequirements(_swerve, _vision);
  }

  @Override
  public void execute() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = _swerve.getLastCalculatedPosition();

    // double error = _vision.isAmp() ? (targetPose.getX() - current_pos.getX()) : _vision.getCalculatedError(current_pos);
    double error = _vision.getAlliance()?1:-1*(targetPose.getX() - current_pos.getX()) ;
    double degree_error = targetPose.rotateBy(Rotation2d.fromDegrees(180)).getRotation()
        .minus(current_pos.getRotation()).getDegrees();

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(_rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    translationVal = 
    _withTag.getAsBoolean() && LimelightHelpers.getTV(null)  && _vision.isAmp()
            ? -_controller_x.calculate(error)
            :
            _translationRateLimiter.calculate(

            
             translationVal );
    strafeVal = _strafeRateLimiter.calculate(
        _withTag.getAsBoolean() && LimelightHelpers.getTV(null)  && _vision.isAmp() ? MathUtil.clamp(strafeVal, -0.5, 0.5): 
        strafeVal );

    rotationVal = _withTag.getAsBoolean() ?-_controller_theta_pid.calculate(degree_error):_rotationRateLimiter.calculate(
       
         rotationVal * Constants.Swerve.maxAngularVelocity);
    
    _swerve.drive(
        new Translation2d(
            translationVal,
            strafeVal).times(Constants.Swerve.maxSpeed),
            // .rotateBy(
            //     // _withTag.getAsBoolean() && !_vision.isAmp() ? targetPose.getRotation() : Rotation2d.fromDegrees(0)
            //     Rotation2d.fromDegrees(0)
            //     ),
        rotationVal,
        !_robotCentricSup.getAsBoolean(),
        true);

  }
private boolean inrange(Pose2d targetPose,Pose2d current_pos){//check delta
  return Math.abs(targetPose.getX() - current_pos.getX()) > 1.5;
}
}
