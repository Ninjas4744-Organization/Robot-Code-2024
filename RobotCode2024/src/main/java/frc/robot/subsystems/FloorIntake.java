package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntake extends SubsystemBase {
  private TalonSRX _collectionMotor;
  private TalonFX _rotationMotor;
  private DigitalInput _beamBreakerNote;
  private DigitalInput _limitSwitchRotation;
  
  public FloorIntake() {
    _collectionMotor = new TalonSRX(Constants.kFloorIntakeMotor);
    _rotationMotor = new TalonFX(Constants.kFloorIntakeRotationMotor);
    _beamBreakerNote = new DigitalInput(Constants.kFloorIntakeBeamBreaker);
    _limitSwitchRotation = new DigitalInput(Constants.kFloorIntakeLimitSwitch);
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

  public void setRotation(double rotation){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(rotation);
    _rotationMotor.setControl(mmReq);
  }

  public double getRotation(){
    return _rotationMotor.getPosition().getValue();
  }

  public void setRotationMotor(double percent){
    _rotationMotor.set(percent);
  }

  public void stopRotation(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getRotation());
    _rotationMotor.setControl(mmReq);
    _rotationMotor.set(0);
  }
  
  public void Override(){
    stopRotation();
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void Reset() {
    _rotationMotor.setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitchRotation.get())
      _rotationMotor.setPosition(0); 

    SmartDashboard.putBoolean("Note", isNote());
    SmartDashboard.putNumber("Floor Intake Rotation", getRotation());
  }

  public Command runUp(){
    return new InstantCommand(() -> {this.setRotation(Constants.kFloorUpPositon); }, this);
  }

  public Command runDown(){
    return new InstantCommand(() -> { this.setRotation(0); }, this);
  }

  public Command runFloorIntake(){
    return new ConditionalCommand(
      new StartEndCommand(
        this::outake,
        this::stopTake, 
        this
      ).withTimeout(1),

      new StartEndCommand(
        this::intake,
        this::stopTake, 
        this
      ).until(this::isNote),

      this::isNote
    );
  }

  public Command runAutoFloorIntake(BooleanSupplier override, Runnable disableOverride){
    return this.runDown()
    .andThen(Commands.waitSeconds(Constants.kTimeToOpenFloorIntake))//fix later: maybe the system is already open, waste of time
    .andThen(this.runFloorIntake())
    .andThen(this.runUp())//Check later: idk when conditional command ends
    .andThen(Commands.waitSeconds(Constants.kTimeToOpenFloorIntake))
    .andThen(this.runFloorIntake())
    .until(override).andThen(disableOverride);//Check later: idk if this until will work like in the inoutake as this is not Commands.Sequence
  }
}