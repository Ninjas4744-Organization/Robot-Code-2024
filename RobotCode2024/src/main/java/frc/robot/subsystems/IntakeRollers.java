package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _beamBreaker;
  
  public IntakeRollers() { 
    _motor = new CANSparkMax(Ports.Intake.kRollersMotor, MotorType.kBrushless);
    _beamBreaker = new DigitalInput(Ports.Intake.kBeamBreaker);
  }

  public boolean isNote(){
    //////////////
    // _led.setData(_ledBuffer);
    // _led.start();
    //////////////
    return _beamBreaker.get();
  }

  public void intake(){
    _motor.set(0.7);
  }

  public void outake(){
    _motor.set(-0.7);
  }

  public void stopTake(){
    _motor.set(0);
    //////////////
    // _led.stop();
    //////////////
  }

  public void Override(){
    stopTake();
  }

  public void Reset() {
    Override();
    if(this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Note", isNote());
  }

  public Command runIntake(){
    return Commands.startEnd(
      () -> { intake(); },
      () -> { stopTake(); }, 
      this).until(() -> { return isNote(); });
  }

  public Command runOutake(){
    return Commands.startEnd(
      () -> { outake(); },
      () -> { stopTake(); }, 
      this).withTimeout(Constants.kTimeToOutake);
  }
}