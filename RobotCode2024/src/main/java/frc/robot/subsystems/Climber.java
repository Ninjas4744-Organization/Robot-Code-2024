package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax _motor1;
  private CANSparkMax _motor2;
  private DigitalInput _limitSwitch;
  private ProfiledPIDController _controller;

  public Climber() {
    _motor1 = new CANSparkMax(Ports.Climber.kMotor1, MotorType.kBrushless);
    _motor2 = new CANSparkMax(Ports.Climber.kMotor2, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Climber.kLimitSwitch);

    _controller = new ProfiledPIDController(
        PIDConstants.Climber.kP,
        PIDConstants.Climber.kI,
        PIDConstants.Climber.kD,
        PIDConstants.Climber.kConstraints);
    
    _motor2.follow(_motor1);
  }

  public void setMotor(double percent){
    _motor1.set(percent);
  }

  public void setHeight(double height){
    _controller.setGoal(height);
  }
  
  public double getHeight(){
    return _motor1.getEncoder().getPosition();
  }

  public boolean isMax(){
    return Math.abs(Constants.kMaxClimber - getHeight()) < 0.2;
  }

  public void Override(){
    _controller.setGoal(getHeight());
    _motor1.set(0);
  }
  
  public void Reset() {
    _motor1.getEncoder().setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(_limitSwitch.get())
    _motor1.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Climber Height", getHeight());
  }
  
  public Command runElevateUp(){
    return Commands.run(
        () -> { this.setHeight(Constants.kMaxClimber); }, 
        this
    );
  }

  public Command runElevateDown(){
    return Commands.run(
        () -> { this.setHeight(0); }, 
        this
    );
  }

  public Command runElevate(){
    return Commands.either(
        this.runElevateUp(), 
        this.runElevateDown(),
        () -> { return Math.abs(this.getHeight()) < 0.2; }
    ).until(() -> { return this.isMax(); });
  }
}
