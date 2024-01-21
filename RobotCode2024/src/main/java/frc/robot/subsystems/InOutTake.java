package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
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

  public void Override(){
    stopElevator();
    stopRotation();
    _collectionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void Reset() {
    _elevatorMotor.setPosition(0);
    _rotationMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    if(_limitSwitchElevator.get())
      _elevatorMotor.setPosition(0);

    if(_limitSwitchRotationTop.get())
      _rotationMotor.setPosition(Constants.kMaxInOutTakeRotation);

    if(_limitSwitchRotationBottom.get())
      _rotationMotor.setPosition(0);

    SmartDashboard.putNumber("Collection Height", getHeight());
  }

  public Command runOpen(double height, double rotation){
    return new InstantCommand(() -> { this.setHeight(height); this.setRotation(rotation); });
  }

  public Command runClose(){
    return new InstantCommand(() -> { this.setHeight(0); this.setRotation(0); });
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
        this::stopTake
      ).withTimeout(1),

      new StartEndCommand(
        this::intake,
        this::stopTake
      ).until(this::isNote),

      this::isNote
    );
  }

  public Command runAutoInOutTake(double height, double rotation){
    return Commands.sequence(
      this.runOpen(height, rotation),
      Commands.waitSeconds(Constants.kTimeToOpenCloseSystem),
      this.runInOutTake()
    );
  }
}