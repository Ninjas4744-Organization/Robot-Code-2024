package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeElevator extends SubsystemBase {
  private TalonFX _elevatorMotor;
  private DigitalInput _limitSwitchElevator;
    
  public IntakeElevator() {
    _elevatorMotor = new TalonFX(Constants.Ports.kIntakeElevatorMotor);
    _limitSwitchElevator = new DigitalInput(Constants.Ports.kIntakeLimitSwitchElevator);
  }
  
  public void setHeight(double height){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(height);
    _elevatorMotor.setControl(mmReq);
  }

  public void setElevatorMotor(double percent){
    _elevatorMotor.set(percent);
  }
  
  public double getHeight(){
    return _elevatorMotor.getPosition().getValue();
  }

  public boolean isMax(double max){
    return Math.abs(max - getHeight()) < 0.2;
  }
  
  public void stopElevator(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getHeight());
    _elevatorMotor.setControl(mmReq);
    _elevatorMotor.set(0);
  }

  public void Override(){
    stopElevator();
  }

  public void Reset() {
    _elevatorMotor.setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitchElevator.get())//Check ! later
      _elevatorMotor.setPosition(0);

    SmartDashboard.putNumber("Intake Elevator Height", getHeight());
  }

  public Command runOpen(double height){
    return Commands.run(() -> { this.setHeight(height); }, this);
  }

  public Command runClose(){
    return Commands.run(() -> { this.setHeight(0); }, this);
  }

  public Command runOpenClose(double height){
    return Commands.either(
      runOpen(height),
      runClose(),
      () -> { return this.getHeight() == 0; }
    );
  }
}