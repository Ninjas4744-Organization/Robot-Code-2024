package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rollers extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _beamBreaker;

  public Rollers() {
    _motor = new CANSparkMax(Constants.Rollers.kRollersMotor, MotorType.kBrushless);
    _beamBreaker = new DigitalInput(Constants.Rollers.kBeamBreaker);
  }

  public boolean isNote() {
    //////////////
    // _led.setData(_ledBuffer);
    // _led.start();
    //////////////
    return !_beamBreaker.get();
  }

  public void setRollers(double speed) {
    _motor.set(speed);
  }

  public void stopTake() {
    _motor.set(0);
    //////////////
    // _led.stop();
    //////////////
  }

  public void Override() {
    stopTake();
  }

  public void Reset() {
    Override();
    if (this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Note", isNote());
  }

  public Command runIntake() {
    return Commands.either(Commands.startEnd(
        () -> {
          setRollers(-1);
        },
        () -> {
          stopTake();
        },
        this),
        Commands.startEnd(
            () -> {
              setRollers(1);
            },
            () -> {
              stopTake();
            }),
        this::isNote);
  }

}