// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private CANSparkMax _climber_motor,_climber_motor_slave;
  private RelativeEncoder _climber_endcoder;
  private SparkPIDController _climber_pid;
  private TrapezoidProfile _climber_profile;

  /** Creates a new Climber. */
  public Climber() {
    _climber_motor = new CANSparkMax(Constants.Climber.motor_id, MotorType.kBrushless);
    _climber_endcoder = _climber_motor.getEncoder();
    _climber_pid = _climber_motor.getPIDController();
    _climber_profile = new TrapezoidProfile(Constants.Climber.ClimberConstants);
    _climber_motor_slave = new CANSparkMax(Constants.Climber.slave_id,MotorType.kBrushless);

    configDriveMotor();
  }

  private void configDriveMotor() {
    _climber_motor.restoreFactoryDefaults();
    _climber_motor.setInverted(Constants.Climber.toInvert);
    _climber_endcoder.setInverted(Constants.Climber.toInvert);
    _climber_motor.setIdleMode(Constants.Climber.ClimberNeutralMode);
    _climber_endcoder.setPositionConversionFactor(Constants.Climber.ClimberConversionPositionFactor);
    _climber_endcoder.setVelocityConversionFactor(Constants.Climber.ClimberConversionVelocityFactor);
    _climber_pid.setP(Constants.Climber.climbKP);
    _climber_pid.setD(Constants.Climber.climbKD);

    _climber_motor_slave.follow(_climber_motor);
    _climber_motor_slave.setInverted(!Constants.Climber
    .toInvert);


    // burns to spark max
    _climber_motor.burnFlash();
    // resets encoder position to 0
    _climber_endcoder.setPosition(0.0);
  }

  public Command setPoseCommand(Supplier<TrapezoidProfile.State> goal) {
    return new TrapezoidProfileCommand(
        _climber_profile,
        pos -> {
          _climber_pid.setReference(pos.position, ControlType.kPosition);
        }, goal, () -> {
          return new TrapezoidProfile.State(_climber_endcoder.getPosition(), _climber_endcoder.getVelocity());
        }, this);
  }

  
}
