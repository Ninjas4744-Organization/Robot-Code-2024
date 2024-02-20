package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private Vision _vision;
  // private limeLight ll;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private PIDController _controller;
  private ProfiledPIDController _theta_controller;
  private PIDController _controller_x;

  BooleanSupplier _withTag;

  public TeleopSwerve(
      Swerve s_Swerve,
      Vision vision,
      // limeLight lime,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier withTag,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this._vision = vision;
    _controller = new PIDController(0.01111111 * 4, 0, 0);
    _controller_x = new PIDController(0.6666666666666667 * 2, 0, 0.1);
    _theta_controller = new ProfiledPIDController(0, 0, 0, null);
    // this.ll = lime;
    addRequirements(s_Swerve);
    this._withTag = withTag;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {

    Pose2d targetPose = _vision.getTagPose();
    Pose2d current_pos = s_Swerve.getLastCalculatedPosition();
    SmartDashboard.putNumber("distance",
        Math.hypot(targetPose.getX() - current_pos.getX(), targetPose.getY() - current_pos.getY()));

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    s_Swerve.drive(
        new Translation2d(translationVal,
            _withTag.getAsBoolean() && inrange(targetPose, current_pos)
                ? -_controller_x.calculate(targetPose.getX() - current_pos.getX() + 0.1)
                : strafeVal)
            .times(Constants.Swerve.maxSpeed),
        _withTag.getAsBoolean()
            ? -_controller.calculate(
                targetPose.rotateBy(Rotation2d.fromDegrees(180)).minus(current_pos).getRotation().getDegrees())
            : rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }

  private boolean inrange(Pose2d targetPose, Pose2d current_pos) {
    return targetPose.getX() - current_pos.getX() < 1.5;
  }
}
