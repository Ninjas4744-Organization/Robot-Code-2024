package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemBase {
  private TalonFX _motor1;
  private TalonFX _motor2;
  private MotionMagicVoltage _mmReq;
  private DigitalInput _limitSwitch;
  private SysIdRoutine _routine;

  public Elevator() {
    _motor1 = new TalonFX(Constants.kElevatorMotor1);
    _motor2 = new TalonFX(Constants.kElevatorMotor2);
    _limitSwitch = new DigitalInput(Constants.kElevatorLimitSwitch);
    
    Follower _followReq = new Follower(Constants.kElevatorMotor1, true);

    _motor2.setControl(_followReq);
    _mmReq = Utils.getMotionMagicConfig();

    _routine = Utils.getSysIdRoutine(_motor1, this);
  }

  public void setHeight(double height){
    _motor1.setControl(_mmReq.withPosition(height).withSlot(0));
  }
  
  public double getHeight(){
    return _motor1.getPosition().getValue();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return _routine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return _routine.dynamic(direction);
  }
  
  public void Reset() {
    _motor1.setPosition(0);
    _motor2.setPosition(0);
  }

  @Override
  public void periodic() {
    if(_limitSwitch.get())
      _motor1.setPosition(0);

    SmartDashboard.putNumber("Height", getHeight());
  }
}
