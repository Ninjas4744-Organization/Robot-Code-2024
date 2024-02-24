package frc.robot.subsystems;

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

public class Elevator extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public Elevator() {
    _motor = new CANSparkMax(Constants.Elevator.kElevatorMotor, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.setInverted(true);
    _motor.getEncoder().setPositionConversionFactor(Constants.Elevator.ControlConstants.kConversionPosFactor);
    _motor.getEncoder().setVelocityConversionFactor(Constants.Elevator.ControlConstants.kConversionVelFactor);
    _limitSwitch = new DigitalInput(Constants.Elevator.kLimitSwitchElevator);

    _controller = _motor.getPIDController();
    _controller.setP(Constants.Elevator.ControlConstants.kP);
    _controller.setI(Constants.Elevator.ControlConstants.kI);
    _controller.setD(Constants.Elevator.ControlConstants.kD);
    _controller.setIZone(0.1);

    _motor.setSoftLimit(SoftLimitDirection.kForward, 0.65f);
    _motor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    _controller.setOutputRange(-0.6, 1);

    _motor.burnFlash();
  }

  public Command setHeight(double height) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(Constants.Elevator.ControlConstants.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(height, 0),
        () -> new TrapezoidProfile.State(getHeight(), 0),
        this);
  }

  public void setElevatorMotor(double percent) {
    _motor.set(percent);
  }

  public double getHeight() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max) {
    return Math.abs(max - getHeight()) < 0.02;
  }

  public void stopElevator() {
    _motor.stopMotor();
  }

  public Command Override() {
    return Commands.sequence(
        // Commands.run(() -> {
        // if (this.getCurrentCommand() != null)
        // this.getCurrentCommand().cancel();
        // }),
        Commands.run(() -> {
          _motor.stopMotor();
        }));
  }

  public Command Reset() {
    return Override();
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("In. Ele. Height", getHeight());
    SmartDashboard.putBoolean("In. Ele. Limit", !_limitSwitch.get());
  }

  public Command runOpen(double height) {
    return setHeight(height);
  }

  public Command runClose() {
    return setHeight(0);
  }

  public Command runOpenClose(double height) {
    return Commands.either(
        runOpen(height),
        runClose(),
        () -> {
          return this.getHeight() == 0;
        });
  }

}