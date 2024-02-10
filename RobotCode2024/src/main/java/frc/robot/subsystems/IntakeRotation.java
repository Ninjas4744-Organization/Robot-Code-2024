package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRotation extends SubsystemBase {
  private TalonFX _rotationMotor;
  private DigitalInput _limitSwitchRotation;
  
  public IntakeRotation() {
    _rotationMotor = new TalonFX(Constants.Ports.kIntakeRotationMotor);
    _limitSwitchRotation = new DigitalInput(Constants.Ports.kIntakeLimitSwitchRotation);
  }

  public void setRotation(double rotation){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(rotation);
    _rotationMotor.setControl(mmReq);
  }

  public double getRotation(){
    return _rotationMotor.getPosition().getValue();
  }

  public boolean isMax(double max){
    return Math.abs(max - getRotation()) < 0.2;
  }

  public void setRotationMotor(double percent){
    _rotationMotor.set(percent);
  }

  public void stopRotation(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getRotation());
    _rotationMotor.setControl(mmReq);
    _rotationMotor.set(0);
  }

  public void Override(){
    stopRotation();
  }

  public void Reset() {
    _rotationMotor.setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitchRotation.get())//Check ! later
      _rotationMotor.setPosition(0);

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