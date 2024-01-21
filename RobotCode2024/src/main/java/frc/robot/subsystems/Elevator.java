package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX _motor1;
  private TalonFX _motor2;
  private DigitalInput _limitSwitchTop;
  private DigitalInput _limitSwitchBottom;

  public Elevator() {
    _motor1 = new TalonFX(Constants.kElevatorMotor1);
    _motor2 = new TalonFX(Constants.kElevatorMotor2);
    _limitSwitchTop = new DigitalInput(Constants.kElevatorLimitSwitchTop);
    _limitSwitchBottom = new DigitalInput(Constants.kElevatorLimitSwitchBottom);
    
    Follower _followReq = new Follower(Constants.kElevatorMotor1, true);
    _motor2.setControl(_followReq);
  }

  public void setHeight(double height){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(height);
    _motor1.setControl(mmReq);
  }
  
  public double getHeight(){
    return _motor1.getPosition().getValue();
  }

  public void Override(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getHeight());
    _motor1.setControl(mmReq);
    _motor1.set(0);
  }
  
  public void Reset() {
    _motor1.setPosition(0);
  }

  @Override
  public void periodic() {
    if(_limitSwitchBottom.get())
      _motor1.setPosition(0);
    
    if(_limitSwitchTop.get())
      _motor1.setPosition(Constants.kElevatorLimitSwitchTop);

    SmartDashboard.putNumber("Height", getHeight());
    SmartDashboard.putBoolean("Elevator Top", _limitSwitchTop.get());
    SmartDashboard.putBoolean("Elevator Bottom", _limitSwitchBottom.get());
  }

  public Command runElevate(){
    return new StartEndCommand(
        () -> { this.setHeight(Constants.kMaxElevator); }, 
        () -> { this.setHeight(0); }
    );
  }
}
