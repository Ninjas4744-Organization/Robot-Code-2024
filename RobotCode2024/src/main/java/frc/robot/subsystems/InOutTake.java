package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InOutTake extends SubsystemBase {
  private TalonSRX _collectionMotor;
  private TalonFX _elevatorMotor;
  private TalonFX _rotationMotor;
  //private DigitalInput _limitSwitchNote;
  private DigitalInput _limitSwitchElevator;
  private DigitalInput _limitSwitchRotationTop;
  private DigitalInput _limitSwitchRotationBottom;
  
  public InOutTake() {
    _collectionMotor = new TalonSRX(Constants.kInOutTakeMotor);
    _elevatorMotor = new TalonFX(Constants.kInOutTakeElevatorMotor);
    _rotationMotor = new TalonFX(Constants.kInOutTakeRotationMotor);
    //_limitSwitchNote = new DigitalInput(Constants.kInOutTakeLimitSwitchNote);
    _limitSwitchElevator = new DigitalInput(Constants.kInOutTakeLimitSwitchElevator);
    _limitSwitchRotationTop = new DigitalInput(Constants.kInOutTakeLimitSwitchTop);
    _limitSwitchRotationBottom = new DigitalInput(Constants.kInOutTakeLimitSwitchBottom);
  }

  public boolean isNote(){
    return false;
  }

  public boolean isRotationTop(){
    return _limitSwitchRotationTop.get();
  }

  public boolean isRotationBottom(){
    return _limitSwitchRotationBottom.get();
  }

  public void intake(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0.7);
  }

  public void outake(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, -0.7);
  }

  public void setRotation(double rotation){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(rotation);
    _rotationMotor.setControl(mmReq);
  }

  public double getRotation(){
    return _rotationMotor.getPosition().getValue();
  }

  public void Override(){
    MotionMagicVoltage mmReq1 = new MotionMagicVoltage(getHeight());
    MotionMagicVoltage mmReq2 = new MotionMagicVoltage(getRotation());
    _elevatorMotor.setControl(mmReq1);
    _rotationMotor.setControl(mmReq2);

    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
    _elevatorMotor.set(0);
    _rotationMotor.set(0);
  }

  public void setHeight(double height){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(height);
    _elevatorMotor.setControl(mmReq);
  }
  
  public double getHeight(){
    return _elevatorMotor.getPosition().getValue();
  }

  public void Reset() {
    _elevatorMotor.setPosition(0);
    _rotationMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    if(_limitSwitchElevator.get())
      _elevatorMotor.setPosition(0);

    SmartDashboard.putNumber("Collection Height", getHeight());
  }
}