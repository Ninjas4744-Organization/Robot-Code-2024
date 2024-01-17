package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InOutTake extends SubsystemBase {
  //private TalonSRX _motor1;  off until prototype is ready
  //private TalonSRX _motor2;  off until prototype is ready
  
  private TalonFX _motor1;
  private TalonFX _motor2;
  private DigitalInput _limitSwitch;
  
  public InOutTake() {
    //_motor1 = new TalonSRX(Constants.kInOutTakeMotor1);  off until prototype is ready
    //_motor2 = new TalonSRX(Constants.kInOutTakeMotor2);  off until prototype is ready

    _motor1 = new TalonFX(Constants.kInOutTakeMotor1);
    _motor2 = new TalonFX(Constants.kInOutTakeMotor2);
    _limitSwitch = new DigitalInput(Constants.kInOutTakeLimitSwitch);

    Follower m_followReq = new Follower(Constants.kInOutTakeMotor1, true);
    _motor2.setControl(m_followReq);
  }

  public boolean isNote(){
    return _limitSwitch.get();
  }

  public void intake(){
    _motor1.set(0.7);
  }

  public void outake(){
    _motor1.set(-0.7);
  }

  public void stopMotors(){
    _motor1.set(0);
  }
  @Override
  public void periodic() {
      super.periodic();
  }
}