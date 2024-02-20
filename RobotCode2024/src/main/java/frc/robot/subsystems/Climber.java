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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class Climber extends SubsystemBase {

  private CANSparkMax _climber_motor, _climber_motor_slave;
  private RelativeEncoder _climber_endcoder;
  private SparkPIDController _climber_pid;
  private TrapezoidProfile _climber_profile;
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            // set states for all 4 modules
          _climber_motor.setVoltage(volts.in(Volts));

          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // set states for all 4 modules

            log.motor("climber")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        _climber_motor.getBusVoltage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(_climber_endcoder.getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(_climber_endcoder.getVelocity(), MetersPerSecond));

          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("drive")
          this));

  /** Creates a new Climber. */
  public Climber() {
    _climber_motor = new CANSparkMax(Constants.Climber.motor_id, MotorType.kBrushless);
    _climber_endcoder = _climber_motor.getEncoder();
    _climber_pid = _climber_motor.getPIDController();
    _climber_profile = new TrapezoidProfile(Constants.Climber.ClimberConstants);
    _climber_motor_slave = new CANSparkMax(Constants.Climber.slave_id, MotorType.kBrushless);

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
    _climber_motor_slave.setInverted(!Constants.Climber.toInvert);

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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  
}
