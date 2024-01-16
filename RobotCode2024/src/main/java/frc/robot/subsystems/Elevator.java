package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;

public class Elevator extends SubsystemBase {
  private TalonFX m_motor1;
  private TalonFX m_motor2;
  private MotionMagicVoltage m_mmReq;
  private DigitalInput m_limitSwitch;

  public Elevator() {
    m_motor1 = new TalonFX(Constants.kElevatorMotor1);
    m_motor2 = new TalonFX(Constants.kElevatorMotor2);
    m_limitSwitch = new DigitalInput(Constants.kElevatorLimitSwitch);
    
    Follower m_followReq = new Follower(Constants.kElevatorMotor1, true);

    m_motor2.setControl(m_followReq);
    m_mmReq = Utils.getMotionMagicConfig();
  }

  public void setHeight(double height){
    m_motor1.setControl(m_mmReq.withPosition(height).withSlot(0));
  }
  
  public double getHeight(){
    return m_motor1.getPosition().getValue();
  }

  public void Reset() {
    m_motor1.setPosition(0);
    m_motor2.setPosition(0);
  }

  @Override
  public void periodic() {
    if(m_limitSwitch.get())
      m_motor1.setPosition(0);

    SmartDashboard.putNumber("Height", getHeight());
  }
}
