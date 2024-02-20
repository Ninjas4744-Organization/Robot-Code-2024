// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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

public class Lift extends SubsystemBase {

    private CANSparkMax _lift_motor, _lift_motor_slave;
    private RelativeEncoder _lift_endcoder;
    private SparkPIDController _lift_pid;
    private TrapezoidProfile _lift_profile;
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

                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // set states for all 4 modules

                        log.motor("lift")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                _lift_motor.getBusVoltage() * RobotController.getBatteryVoltage(),
                                                Volts))
                                .linearPosition(m_distance.mut_replace(_lift_endcoder.getPosition() / 100, Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(_lift_endcoder.getVelocity() / 100, MetersPerSecond));

                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

    /** Creates a new Lift. */
    public Lift() {

        _lift_motor = new CANSparkMax(Constants.Lift.motor_id, MotorType.kBrushless);
        _lift_motor_slave = new CANSparkMax(Constants.Lift.slave_id, MotorType.kBrushless);

        _lift_endcoder = _lift_motor.getEncoder();
        _lift_pid = _lift_motor.getPIDController();
        _lift_profile = new TrapezoidProfile(Constants.Lift.LiftConstants);
        configliftMotor();
    }

    private void configliftMotor() {
        _lift_motor.restoreFactoryDefaults();
        _lift_motor.setInverted(Constants.Lift.toInvert);
        _lift_endcoder.setInverted(Constants.Lift.toInvert);
        _lift_motor.setIdleMode(Constants.Lift.liftNeutralMode);
        _lift_endcoder.setPositionConversionFactor(Constants.Lift.liftConversionPositionFactor);
        _lift_endcoder.setVelocityConversionFactor(Constants.Lift.liftConversionVelocityFactor);
        _lift_pid.setP(Constants.Lift.liftKP);
        _lift_pid.setD(Constants.Lift.liftKD);

        _lift_motor_slave.follow(_lift_motor);
        _lift_motor_slave.setInverted(!Constants.Lift.toInvert);

        // burns to spark max
        _lift_motor.burnFlash();
        // resets encoder position to 0
        _lift_endcoder.setPosition(0.0);
    }

    public Command setPoseCommand(Supplier<TrapezoidProfile.State> goal) {
        return new TrapezoidProfileCommand(
                _lift_profile,
                pos -> {
                    _lift_pid.setReference(pos.position, ControlType.kPosition);
                }, goal, () -> {
                    return new TrapezoidProfile.State(_lift_endcoder.getPosition(), _lift_endcoder.getVelocity());
                }, this);
    }

    public Command intakeConfiguration() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Lift.intake_setpoint, 0));
    }

    public Command outakeConfiguration() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Lift.outake_setpoint, 0));
    }

    public Command defaultCommand() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Lift.default_setpoint, 0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
