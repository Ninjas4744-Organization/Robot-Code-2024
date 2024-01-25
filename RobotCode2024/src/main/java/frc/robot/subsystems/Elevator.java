package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX _motor1;
  private TalonFX _motor2;
  private DigitalInput _limitSwitch;

  public Elevator() {
    _motor1 = new TalonFX(Constants.kElevatorMotor1);
    _motor2 = new TalonFX(Constants.kElevatorMotor2);
    _limitSwitch = new DigitalInput(Constants.kElevatorLimitSwitch);
    
    Follower _followReq = new Follower(Constants.kElevatorMotor1, true);
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

  public void Override(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getHeight());
    _motor1.setControl(mmReq);
    _motor1.set(0);
  }
  
  public void Reset() {
    _motor1.setPosition(0);
    _motor1.set(0);
  }

  @Override
  public void periodic() {
    if(_limitSwitch.get())
      _motor1.setPosition(0);

    SmartDashboard.putNumber("Height", getHeight());
  }

  public Command runElevateUp(){
    return new InstantCommand(
        () -> { this.setHeight(Constants.kMaxElevator); }
    );
  }

  public Command runElevateDown(){
    return new InstantCommand(
        () -> { this.setHeight(0); }
    );
  }

  public Command runElevate(){
    return new ConditionalCommand(
        this.runElevateUp(), 
        this.runElevateDown(),
        () -> { return Math.abs(this.getHeight()) < 0.2; }
    ).withTimeout(Constants.kTimeToClimbUntilTrap);
  }

  public Command runAutoElevate(InOutTake inOutTake, BooleanSupplier override, Runnable disableOverride){
    return Commands.sequence(
      this.runElevate(),
      Commands.waitSeconds(Constants.kTimeToClimbUntilTrap),
      inOutTake.runAutoInOutTake(Constants.kTrapOpenHeight, Constants.kTrapOpenRotation, override, disableOverride)
    ).until(override).andThen(disableOverride);
  }
}
