package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX _motor1;
  private TalonFX _motor2;
  private DigitalInput _limitSwitch;

  public Climber() {
    _motor1 = new TalonFX(Constants.Ports.kClimberMotor1);
    _motor2 = new TalonFX(Constants.Ports.kClimberMotor2);
    _limitSwitch = new DigitalInput(Constants.Ports.kClimberLimitSwitch);
    
    Follower _followReq = new Follower(Constants.Ports.kClimberMotor1, true);
    _motor2.setControl(_followReq);
  }

  public void setMotor(double percent){
    _motor1.set(percent);
  }

  public void setHeight(double height){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(height);
    _motor1.setControl(mmReq);
  }
  
  public double getHeight(){
    return _motor1.getPosition().getValue();
  }

  public boolean isMax(){
    return Math.abs(Constants.kMaxClimber - getHeight()) < 0.2;
  }

  public void Override(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getHeight());
    _motor1.setControl(mmReq);
    _motor1.set(0);
  }
  
  public void Reset() {
    _motor1.setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(_limitSwitch.get())
      _motor1.setPosition(0);

    SmartDashboard.putNumber("Elevator Height", getHeight());
    SmartDashboard.putBoolean("Elevator Bottom", _limitSwitch.get());
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
