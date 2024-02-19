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

public class IntakeElevator extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private ProfiledPIDController _controller;
    
  public IntakeElevator() {
    _motor = new CANSparkMax(Ports.Intake.kElevatorMotor, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchElevator);

    _controller = new ProfiledPIDController(
        PIDConstants.IntakeElevator.kP,
        PIDConstants.IntakeElevator.kI,
        PIDConstants.IntakeElevator.kD,
        PIDConstants.IntakeElevator.kConstraints);
  }
  
  public void setHeight(double height){
    _controller.setGoal(height);
  }

  public void setElevatorMotor(double percent){
    _motor.set(percent);
  }
  
  public double getHeight(){
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max){
    return Math.abs(max - getHeight()) < 0.2;
  }
  
  public void stopElevator(){
    _controller.setGoal(getHeight());
    _motor.set(0);
  }

  public void Override(){
    stopElevator();
  }

  public void Reset() {
    _motor.getEncoder().setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitch.get())//Check ! later
      _motor.getEncoder().setPosition(0);

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