package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutonomySwerve extends Command {
  private Swerve _swerve;
  private Vision _vision;
  private PIDController _controller_0;
  private PIDController _controller_x;
  private PIDController _controller_z;

  public AutonomySwerve(Swerve swerve, Vision vision){
    _swerve = swerve;
    _vision = vision;
    _controller_0 = new PIDController(0.01111111*4, 0, 0);
    _controller_x = new PIDController(0.6666666666666667*2, 0, 0.1);
    _controller_z = new PIDController(0.6666666666666667*2, 0, 0.1);//change later
    
    addRequirements(_swerve);
  }
  
  @Override
  public void execute() {
    Pose2d targetPose = _vision.getTagPose();
    Pose2d currentPos = _swerve.getLastCalculatedPosition();
   
    if(!isTag(targetPose, currentPos))
      return;

    _swerve.drive(
        new Translation2d(
          -_controller_z.calculate(targetPose.getZ() - currentPos.getZ() + 0.1).times(Constants.Swerve.maxSpeed),//fix later
          -_controller_x.calculate(targetPose.getX() - currentPos.getX() + 0.1)).times(Constants.Swerve.maxSpeed
        ),

        -_controller_0.calculate(targetPose.
        rotateBy(Rotation2d.fromDegrees(180)).
        minus(currentPos).getRotation().getDegrees()),

        true,
        true
    );
  }

  private boolean isTag(Pose2d targetPose, Pose2d currentPos) {
    if(targetPose != null)//check later if can do that
      return targetPose.getX() - currentPos.getX() < 1.5;
    return false;
  }
}
