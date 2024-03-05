package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rollers extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _beamBreaker;

  public Rollers() {
    _motor = new CANSparkMax(Constants.Rollers.kMotorID, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.setSmartCurrentLimit(35);
    _motor.burnFlash();
    _beamBreaker = new DigitalInput(Constants.Rollers.kBeamBreakerID);
  }

  public boolean isNote() {
    return !_beamBreaker.get();
  }

  public void setMotor(double speed) {
    _motor.set(speed);
  }

  public double getMotor() {
    return _motor.get();
  }

  public void Stop() {
    _motor.set(0);
  }

  public void Reset() {
    Stop();
    if (this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    // Shuffleboard.getTab("Game").add("Intake Note", isNote());

    // Shuffleboard.getTab("Debug").add("Intake Note", isNote());
    // Shuffleboard.getTab("Debug").add("Rollers Percent", getMotor());
  }

  public Command runIntake() {
    return Commands.either(
      Commands.startEnd(
        () -> {setMotor(1);},
        () -> {Stop();},
        this
      ),
      Commands.startEnd(
        () -> {setMotor(-1);},
        () -> {Stop();}
      ),
      this::isNote
    );
  }

  public Command runIntake(double speed) {
    return Commands.either(
      Commands.startEnd(
        () -> {setMotor(speed);},
        () -> {Stop();},
        this
      ),
      Commands.startEnd(
        () -> {setMotor(-speed);},
        () -> {Stop();}
      ),
      this::isNote
    );
  }
}