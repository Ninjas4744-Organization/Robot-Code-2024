package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntakeRollers extends SubsystemBase {
  private TalonSRX _collectionMotor;
  private DigitalInput _beamBreakerNote;
  
  public FloorIntakeRollers() {
    _collectionMotor = new TalonSRX(Constants.Ports.FloorIntakeConstants.kFloorIntakeMotor);
    _beamBreakerNote = new DigitalInput(Constants.Ports.FloorIntakeConstants.kFloorIntakeBeamBreaker);
  }

  public boolean isNote(){
    return _beamBreakerNote.get();
  }

  public void intake(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0.7);
  }

  public void outake(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, -0.7);
  }

  public void stopTake(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
  
  public void Override(){
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void Reset() {
    Override();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("FloorIntake Note", isNote());
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