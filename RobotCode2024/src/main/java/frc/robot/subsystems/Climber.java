package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.Ports;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax _motor1;
  private CANSparkMax _motor2;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public Climber() {
    _motor1 = new CANSparkMax(Ports.Climber.kMotor1, MotorType.kBrushless);
    _motor2 = new CANSparkMax(Ports.Climber.kMotor2, MotorType.kBrushless);
    _limitSwitch = new DigitalInput(Ports.Climber.kLimitSwitch);

    _controller = _motor1.getPIDController();
    _controller.setP(PIDConstants.Climber.kP);
    _controller.setI(PIDConstants.Climber.kI);
    _controller.setD(PIDConstants.Climber.kD);

    _motor2.follow(_motor1);
  }

  public void setMotor(double percent) {
    _motor1.set(percent);
  }

  public Command setHeight(double height) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(PIDConstants.Climber.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(height, 0),
        () -> new TrapezoidProfile.State(getHeight(), 0),
        this);
  }

  public double getHeight() {
    return _motor1.getEncoder().getPosition();
  }

  public boolean isMax() {
    return Math.abs(Constants.kMaxClimber - getHeight()) < 0.2;
  }

  public Command Override() {
    return setHeight(getHeight());
  }

  public void Reset() {
    if (this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    if (_limitSwitch.get())
      _motor1.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Climber Height", getHeight());
  }

  public Command runElevateUp() {
    return setHeight(Constants.kMaxClimber);
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