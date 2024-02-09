// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rollers extends SubsystemBase {
  CANSparkMax _rollers;
  DigitalInput _isNote;

  /** Creates a new Rollers. */
  public Rollers() {
    _rollers = new CANSparkMax(Constants.Rollers.motor_id, MotorType.kBrushless);
    _isNote = new DigitalInput(Constants.Rollers.digital_input_id);
  }

  public void setSpeed(double speed) {
    _rollers.setVoltage(speed * 12);
  }

  public Boolean isNote() {
    return _isNote.get();
  }

  public Command defaultCommand() {
    return Commands.runOnce(() -> {
      setSpeed(0);
    }, this);
  }

  public Command runIntake(Boolean flag) {
    return Commands.either(
        Commands.startEnd(() -> setSpeed(Constants.Rollers.outake_voltage), () -> setSpeed(0), this),
        Commands.startEnd(() -> setSpeed(Constants.Rollers.intake_voltage), () -> setSpeed(0), this),
        flag == null ? this::isNote : () -> flag);
  }

}
