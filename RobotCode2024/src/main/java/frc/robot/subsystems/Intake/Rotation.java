package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class Rotation extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public Rotation() {
    _motor = new CANSparkMax(Constants.Rotation.kRotationMotor, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.getEncoder().setPositionConversionFactor(Constants.Rotation.ControlConstants.kConversionPosFactor);
    _motor.getEncoder().setVelocityConversionFactor(Constants.Rotation.ControlConstants.kConversionVelFactor);
    _limitSwitch = new DigitalInput(Constants.Rotation.kLimitSwitch);

    _controller = _motor.getPIDController();
    _controller.setP(Constants.Rotation.ControlConstants.kP);
    _controller.setI(Constants.Rotation.ControlConstants.kI);
    _controller.setD(Constants.Rotation.ControlConstants.kD);
    _controller.setIZone(3);

    _controller.setOutputRange(-0.55, 0.55);

    _motor.burnFlash();
  }

  public Command setRotation(double rotation) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(Constants.Rotation.ControlConstants.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(rotation, 0),
        () -> new TrapezoidProfile.State(getRotation(), 0),
        this);
  }

  public double getRotation() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isRotation(double rotation) {
    return Math.abs(rotation - getRotation()) < 0.07;
  }

  public void setMotor(double percent) {
    _motor.set(percent);
  }

  public void Stop() {
    _motor.set(0);
  }

  public Command Reset() {
    return Commands.startEnd(
        () -> {setMotor(-0.1);},
        () -> {Stop();},
        this
      ).until(() -> {return !_limitSwitch.get();});
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putBoolean("Rotation Limit", !_limitSwitch.get());
  }

  public Command runOpen(double rotation) {
    return setRotation(rotation);
  }

  public Command runClose() {
    return this.setRotation(0);
  }

  public Command runOpenClose(double rotation) {
    return Commands.either(
      runOpen(rotation),
      runClose(),
      () -> {return this.getRotation() == 0;}
    );
  }
}