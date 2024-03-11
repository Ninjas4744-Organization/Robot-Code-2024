// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.NinjaMotorController;
import frc.lib.genericInterfaces.NinjaSubsytem;
import frc.robot.Constants;

public class Lift extends NinjaSubsytem {
  private DigitalInput _limitSwitch;

  /** Creates a new Lift. */
  public Lift(NinjaMotorController master) {
    super(master);

  }

  public boolean isHeight(double height) {
    return Math.abs(height - _master.get()) < 0.02;
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get()) {
      _master.setPosition(0);
    }
  }

  public Command openAmp() {
    return Commands.runOnce(() -> {
      runProfile(new State(Constants.Elevator.States.kAmpOpenHeight, 0));
    }, this);
  }

  public Command openTrap() {
    return Commands.runOnce(() -> {
      runProfile(new State(Constants.Elevator.States.kTrapOpenHeight, 0));
    }, this);
  }

  @Override
  public Command reset() {
    return runMotors(-0.3).until(_limitSwitch::get);

  }
}
