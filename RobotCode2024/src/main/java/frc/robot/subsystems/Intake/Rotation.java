package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.GenericSparkMaxSubsystem;
import frc.lib.drivers.NinjaMotorController;
import frc.lib.genericInterfaces.NinjaSubsytem;

public class Rotation extends NinjaSubsytem {
  private DigitalInput _limitSwitch;

  public Rotation(NinjaMotorController master) {
    super(master);

  }

  

  @Override
  public void periodic() {
    if (!_limitSwitch.get()) {
      _master.setPosition(0);
    }
    super.periodic();
  }

  public Command runOpenClose(double rotation) {
    return Commands.either(
        runProfile(new State(rotation, 0)),
        close(),
        _limitSwitch::get);
  }
}