package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.GenericSparkMaxSubsystem;
import frc.lib.drivers.NinjaMotorController;
import frc.lib.genericInterfaces.NinjaSubsytem;
import frc.robot.Constants;

public class Rollers extends NinjaSubsytem {
  private DigitalInput _beamBreaker;

  public Rollers(NinjaMotorController master) {
    super(master);

    _beamBreaker = new DigitalInput(Constants.Rollers.kBeamBreakerID);
  }

  

  public boolean isNote() {
    return !_beamBreaker.get();
  }

  public Command runIntake() {
    return Commands.either(
        runMotors(1),
        runMotors(1),
        this::isNote);
  }


}