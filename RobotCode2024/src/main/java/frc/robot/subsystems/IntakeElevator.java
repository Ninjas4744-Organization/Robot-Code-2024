package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Ports;

public class IntakeElevator extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public IntakeElevator() {
    _motor = new CANSparkMax(Ports.Intake.kElevatorMotor, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchElevator);

    _controller = _motor.getPIDController();
    _controller.setP(PIDConstants.IntakeElevator.kP);
    _controller.setI(PIDConstants.IntakeElevator.kI);
    _controller.setD(PIDConstants.IntakeElevator.kD);
  }

  public Command setHeight(double height) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(PIDConstants.IntakeElevator.kConstraints),
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
    return Math.abs(max - getHeight()) < 0.2;
  }

  public void stopElevator() {
    _motor.set(0);
  }

  public Command Override() {
    return setHeight(getHeight());
  }

  public void Reset() {
    _motor.getEncoder().setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Elevator Height", getHeight());
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