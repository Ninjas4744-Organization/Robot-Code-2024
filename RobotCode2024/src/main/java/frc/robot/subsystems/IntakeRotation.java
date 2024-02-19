package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Ports;

public class IntakeRotation extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private ProfiledPIDController _controller;
  
  public IntakeRotation() {
    _motor = new CANSparkMax(Ports.Intake.kRotationMotor, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchRotation);

    _controller = new ProfiledPIDController(
        PIDConstants.IntakeRotation.kP,
        PIDConstants.IntakeRotation.kI,
        PIDConstants.IntakeRotation.kD,
        PIDConstants.IntakeRotation.kConstraints);
  }

  public void setRotation(double rotation){
    _controller.setGoal(rotation);
  }

  public double getRotation(){
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max){
    return Math.abs(max - getRotation()) < 0.2;
  }

  public void setRotationMotor(double percent){
    _motor.set(percent);
  }

  public void stopRotation(){
    _controller.setGoal(getRotation());
    _motor.set(0);
  }

  public void Override(){
    stopRotation();
  }

  public void Reset() {
    _motor.getEncoder().setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitch.get())//Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Rotation", getRotation());
  }

  public Command runOpen(double rotation){
    return Commands.run(() -> { this.setRotation(rotation); }, this);
  }

  public Command runClose(){
    return Commands.run(() -> { this.setRotation(0); }, this);
  }

  public Command runOpenClose(double rotation){
    return Commands.either(
      runOpen(rotation),
      runClose(),
      () -> { return this.getRotation() == 0; }
    );
  }
}