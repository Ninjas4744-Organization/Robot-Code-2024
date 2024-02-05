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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class InOutTake extends SubsystemBase {
  private TalonSRX _collectionMotor;
  private TalonFX _elevatorMotor;
  private TalonFX _rotationMotor;
  private DigitalInput _beamBreakerNote;
  private DigitalInput _limitSwitchElevator;
  private DigitalInput _limitSwitchRotation;
  ///////////////////////////////////////////
  // private AddressableLED _led;
  // private AddressableLEDBuffer _ledBuffer;
  //////////////////////////////////////////
  
  public InOutTake() {
    _collectionMotor = new TalonSRX(Constants.kInOutTakeMotor);
    _rotationMotor = new TalonFX(Constants.kInOutTakeRotationMotor);
    _elevatorMotor = new TalonFX(Constants.kInOutTakeElevatorMotor);
    _beamBreakerNote = new DigitalInput(Constants.kInOutTakeBeamBreakerNote);
    _limitSwitchElevator = new DigitalInput(Constants.kInOutTakeLimitSwitchElevator);
    _limitSwitchRotation = new DigitalInput(Constants.kInOutTakeLimitSwitchRotation);
    //////////////////////////////////
    // _led = new AddressableLED(0);
    // _ledBuffer = new AddressableLEDBuffer(60);
    // _led.setLength(_ledBuffer.getLength());
    //////////////////////////////////


  }

  public boolean isNote(){
    //////////////
    // _led.setData(_ledBuffer);
    // _led.start();
    //////////////
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
    //////////////
    // _led.stop();
    //////////////
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
  
  public void setHeight(double height){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(height);
    _elevatorMotor.setControl(mmReq);
  }
  
  public double getHeight(){
    return _elevatorMotor.getPosition().getValue();
  }
  
  public void stopElevator(){
    MotionMagicVoltage mmReq = new MotionMagicVoltage(getHeight());
    _elevatorMotor.setControl(mmReq);
    _elevatorMotor.set(0);
  }

  public void setElevatorMotor(double percent){
    _elevatorMotor.set(percent);
  }

  public void Override(){
    stopElevator();
    stopRotation();
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void Reset() {
    _elevatorMotor.setPosition(0);
    _rotationMotor.setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if(!_limitSwitchElevator.get())//Check ! later
      _elevatorMotor.setPosition(0);

    if(!_limitSwitchRotation.get())//Check ! later
      _rotationMotor.setPosition(0);

    SmartDashboard.putBoolean("Note", isNote());
    SmartDashboard.putNumber("Collection Height", getHeight());
    SmartDashboard.putNumber("Collection Rotation", getRotation());
  }

  public Command runOpen(double height, double rotation){
    return new InstantCommand(() -> { this.setHeight(height); this.setRotation(rotation); }, this);
  }

  public Command runClose(){
    return new InstantCommand(() -> { this.setHeight(0); this.setRotation(0); }, this);
  }

  public Command runOpenClose(double height, double rotation){
    return new ConditionalCommand(
      runOpen(height, rotation),
      runClose(),
      () -> { return this.getHeight() == 0; }
    );
  }

  public Command runInOutTake(){
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

  public Command runAutoInOutTake(double height, double rotation, BooleanSupplier override, Runnable disableOverride){
    return Commands.sequence(
      this.runOpen(height, rotation),
      Commands.waitSeconds(Constants.kTimeToOpenCollection),//fix later: maybe the system is already open, waste of time
      this.runInOutTake()
    ).until(override).andThen(disableOverride);
  }
}