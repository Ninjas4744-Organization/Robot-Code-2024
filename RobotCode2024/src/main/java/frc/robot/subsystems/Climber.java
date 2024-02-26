package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax _motor1;
  private CANSparkMax _motor2;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public Climber() {
    _motor1 = new CANSparkMax(Constants.Climber.kMotor1, MotorType.kBrushless);
    _motor1.restoreFactoryDefaults();
    _motor1.getEncoder().setPositionConversionFactor(Constants.Climber.ControlConstants.kConversionPosFactor);
    _motor1.getEncoder().setVelocityConversionFactor(Constants.Climber.ControlConstants.kConversionVelFactor);
    _motor2 = new CANSparkMax(Constants.Climber.kMotor2, MotorType.kBrushless);
    _motor2.restoreFactoryDefaults();
    _limitSwitch = new DigitalInput(Constants.Climber.kLimitSwitch);

    _controller = _motor1.getPIDController();
    _controller.setP(Constants.Climber.ControlConstants.kP);
    _controller.setI(Constants.Climber.ControlConstants.kI);
    _controller.setD(Constants.Climber.ControlConstants.kD);

    _motor1.enableSoftLimit(SoftLimitDirection.kForward, true);

    _motor1.setSoftLimit(SoftLimitDirection.kForward, 0.6f);
    _motor2.setSoftLimit(SoftLimitDirection.kReverse, 0);

    _motor2.follow(_motor1, true);

    _motor1.burnFlash();
    _motor2.burnFlash();
  }

  public void setMotor(double percent) {
    _motor1.set(percent);
  }

  public void stopMotor() {
    _motor1.stopMotor();
  }

  public Command setHeight(double height) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(Constants.Climber.ControlConstants.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(height, 0),
        () -> new TrapezoidProfile.State(getHeight(), 0),
        this);
  }

  public double getHeight() {
    return _motor1.getEncoder().getPosition();
  }

  public boolean isMax() {
    return Math.abs(Constants.Climber.kMaxClimber - getHeight()) < 0.2;
  }

  public Command Override() {
    return Commands.sequence(
        Commands.run(() -> {
          _motor1.stopMotor();
        }));
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())
      _motor1.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Climber Height", getHeight());
    SmartDashboard.putBoolean("Climber Limit", !_limitSwitch.get());
  }

  public Command runElevateUp() {
    return setHeight(Constants.Climber.kMaxClimber);
  }

  public Command runElevateDown() {
    return setHeight(0);
  }

  public Command runElevate() {
    return Commands.either(
        this.runElevateUp(),
        this.runElevateDown(),
        () -> {
          return Math.abs(this.getHeight()) < 0.2;
        }).until(() -> {
          return this.isMax();
        });
  }
}