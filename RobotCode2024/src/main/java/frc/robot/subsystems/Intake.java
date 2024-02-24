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

public class Intake extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  public Intake() {
    _motor = new CANSparkMax(Constants.Intake.kRotationMotor, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.getEncoder().setPositionConversionFactor(Constants.Intake.ControlConstants.kConversionPosFactor);
    _motor.getEncoder().setVelocityConversionFactor(Constants.Intake.ControlConstants.kConversionVelFactor);
    _limitSwitch = new DigitalInput(Constants.Intake.kLimitSwitchRotation);

    _controller = _motor.getPIDController();
    _controller.setP(Constants.Intake.ControlConstants.kP);
    _controller.setI(Constants.Intake.ControlConstants.kI);
    _controller.setD(Constants.Intake.ControlConstants.kD);
    _controller.setIZone(3);

    _motor.setSoftLimit(SoftLimitDirection.kForward, 210);
    _motor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    _controller.setOutputRange(-0.55, 0.55);

    _motor.burnFlash();
  }

  public Command setRotation(double rotation) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(Constants.Intake.ControlConstants.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(rotation, 0),
        () -> new TrapezoidProfile.State(getRotation(), 0),
        this);
  }

  public double getRotation() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max) {
    return Math.abs(max - getRotation()) < 0.07;
  }

  public void setRotationMotor(double percent) {
    _motor.set(percent);
  }

  public void stopRotation() {
    _motor.set(0);
  }

  public Command Override() {
    // return Commands.sequence(
    // Commands.run(() -> {
    // if (this.getCurrentCommand() != null)
    // this.getCurrentCommand().cancel();
    // }),
    return Commands.run(() -> {
      _motor.stopMotor();
    }, this);
    // );
  }

  public Command Reset() {
    return Commands.sequence(
        Override().withTimeout(0.1),
        Commands.run(() -> {
          this.setRotationMotor(-0.08);
        }, this).until(() -> {
          return !_limitSwitch.get();
        }),
        // Commands.waitUntil(() -> {return !_limitSwitch.get();}),
        Commands.run(() -> {
          this.setRotationMotor(0);
        }, this));
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Rotation", getRotation());
    // SmartDashboard.putNumber("Intake Rotation Velocity",
    // _motor.getEncoder().getVelocity());
    SmartDashboard.putBoolean("In. Rot. Limit", !_limitSwitch.get());
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