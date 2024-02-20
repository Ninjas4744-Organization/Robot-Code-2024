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

public class IntakeRotation extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public IntakeRotation() {
    _motor = new CANSparkMax(Ports.Intake.kRotationMotor, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchRotation);

    _controller = _motor.getPIDController();
    _controller.setP(PIDConstants.IntakeRotation.kP);
    _controller.setI(PIDConstants.IntakeRotation.kI);
    _controller.setD(PIDConstants.IntakeRotation.kD);
  }

  public Command setRotation(double rotation) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(PIDConstants.IntakeRotation.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(rotation, 0),
        () -> new TrapezoidProfile.State(getRotation(), 0),
        this);
  }

  public double getRotation() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max) {
    return Math.abs(max - getRotation()) < 0.2;
  }

  public void setRotationMotor(double percent) {
    _motor.set(percent);
  }

  public void stopRotation() {
    _motor.set(0);
  }

  public Command Override() {
    return setRotation(getRotation());
  }

  public void Reset() {
    _motor.getEncoder().setPosition(0);
    Override();
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Rotation", getRotation());
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
        () -> {
          return this.getRotation() == 0;
        });
  }
}