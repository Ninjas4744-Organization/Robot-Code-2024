package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax _motor1;
  private CANSparkMax _motor2;
  private DigitalInput _limitSwitch;
//   private SparkPIDController _controller;

  public Climber() {
    _motor1 = new CANSparkMax(Constants.Climber.kMotor1ID, MotorType.kBrushless);
    _motor1.restoreFactoryDefaults();
    _motor1.setSmartCurrentLimit(Constants.kCurrentLimit);
    _motor2 = new CANSparkMax(Constants.Climber.kMotor2ID, MotorType.kBrushless);
    _motor2.restoreFactoryDefaults();
    _motor2.setSmartCurrentLimit(Constants.kCurrentLimit);
    _limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitchID);

    _motor1.getEncoder().setPositionConversionFactor(Constants.Climber.ControlConstants.kConversionPosFactor);
    _motor1.getEncoder().setVelocityConversionFactor(Constants.Climber.ControlConstants.kConversionVelFactor);

//     _controller = _motor1.getPIDController();
//     _controller.setP(Constants.Climber.ControlConstants.kP);
//     _controller.setI(Constants.Climber.ControlConstants.kI);
//     _controller.setD(Constants.Climber.ControlConstants.kD);

//     _motor2.follow(_motor1, true);
    _motor2.setInverted(true);
    _motor1.burnFlash();
    _motor2.burnFlash();
  }

//   public Command setHeight(double height){
//     return new TrapezoidProfileCommand(
//         new TrapezoidProfile(Constants.Climber.ControlConstants.kConstraints),
//         output -> _controller.setReference(output.position, ControlType.kPosition),
//         () -> new TrapezoidProfile.State(height, 0),
//         () -> new TrapezoidProfile.State(getHeight(), 0),
//         this);
//   }

  public double getHeight() {
    return _motor1.getEncoder().getPosition();
  }

  public boolean isHeight(double height) {
    return Math.abs(height - getHeight()) < 0.02;
  }

  public void setMotor(double percent) {
    if (isLimitSwitch() && percent < 0) 
      Stop(); 
    else{
      _motor1.set(percent);
      _motor2.set(percent);
    }
  }

  public double getMotor() {
    return _motor1.get();
  }

  public void Stop() {
    _motor1.stopMotor();
    _motor2.stopMotor();
  }

  public boolean isLimitSwitch() {
    return !_limitSwitch.get();
  }
  
  @Override
  public void periodic() {
    if(isLimitSwitch())
      _motor1.getEncoder().setPosition(0);

    if(isLimitSwitch() && getMotor() < 0)
      Stop();

    SmartDashboard.putBoolean("Climb limit", isLimitSwitch());
  }

  public Command Reset(){
    return Commands.startEnd(
      () -> {setMotor(-0.5);},
      () -> {Stop();},
      this
    ).until(() -> {return isLimitSwitch();});
  }

  // public Command runClimb(){
  //   return Commands.either(
  //     setHeight(Constants.Climber.kMaxClimber),
  //     setHeight(0),
  //     () -> {return !_limitSwitch.get();}
  //   );
  // }
}