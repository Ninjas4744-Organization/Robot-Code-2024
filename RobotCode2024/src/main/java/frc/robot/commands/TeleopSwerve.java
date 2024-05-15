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
  private BooleanSupplier _tornadoSup;

  private DoubleSupplier _translationSup;
  private DoubleSupplier _strafeSup;
  private DoubleSupplier _rotationXSup;
  private DoubleSupplier _rotationYSup;

  private PIDController _aController;

  public TeleopSwerve(
      Swerve swerve,
      Vision vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationXSup,
      DoubleSupplier rotationYSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier tornadoSup) {

    _swerve = swerve;
    _vision = vision;
    _robotCentricSup = robotCentricSup;
    _tornadoSup = tornadoSup;

    _translationSup = translationSup;
    _strafeSup = strafeSup;
    _rotationXSup = rotationXSup;
    _rotationYSup = rotationYSup;

    _aController = new PIDController(Constants.Swerve.smartAngleKP, Constants.Swerve.smartAngleKI, Constants.Swerve.smartAngleKD);
    _aController.enableContinuousInput(-1.5 * Math.PI, 0.5 * Math.PI);

    addRequirements(_swerve, _vision);
  }

  @Override
  public void execute() {
    double currentRotation = _swerve.getYaw().getRadians();
    double targetRotation = currentRotation;

    if(Math.abs(_rotationYSup.getAsDouble()) >= 0.1 || Math.abs(_rotationXSup.getAsDouble()) >= 0.1)
      targetRotation = -Math.atan2(_rotationYSup.getAsDouble(), _rotationXSup.getAsDouble()) + 0.5 * Math.PI;

    double roundedRotation = Math.round(targetRotation / (Math.PI / 4)) * (Math.PI / 4);
    targetRotation = Math.abs(roundedRotation - targetRotation) <= 15 * 0.017453 ? roundedRotation : targetRotation;

    // SmartDashboard.putNumber("TargetRotation", targetRotation * 57.29578);
    // SmartDashboard.putNumber("CurrentRotation", currentRotation * 57.29578);
    // SmartDashboard.putNumber("RotationPID", _aController.calculate(currentRotation, targetRotation));

    double translationVal = MathUtil.applyDeadband(_translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(_strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    double rotationVal = _aController.calculate(currentRotation, targetRotation);
    SmartDashboard.putNumber("Angle Error", targetRotation - currentRotation);
    SmartDashboard.putNumber("Gyro", _swerve.getYaw().getDegrees());
    
    _swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
      (_tornadoSup.getAsBoolean() ? 1 : rotationVal) * Constants.Swerve.maxAngularVelocity,
      !_robotCentricSup.getAsBoolean(),
      true
    );
  }
}