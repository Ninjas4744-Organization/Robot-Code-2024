package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivers.NinjaMotorController;
import frc.lib.genericInterfaces.NinjaSubsytem;

public class Climber extends NinjaSubsytem {

  private DigitalInput _limitSwitch;

  public Climber(NinjaMotorController master) {
    super(master);

  }

  public boolean isHeight(double height) {
    return Math.abs(height - _master.get()) < 0.02;
  }

  public boolean isLimitSwitch() {
    return !_limitSwitch.get();
  }

  @Override
  public void periodic() {
    if (isLimitSwitch())
      _master.setPosition(0);

    if (isLimitSwitch() && _master.get() < 0)
      _master.stop();

    SmartDashboard.putBoolean("Climb limit", isLimitSwitch());
  }

  @Override
  public Command reset() {
    return runMotors(0.5).until(() -> {
      return isLimitSwitch();
    });
  }

}