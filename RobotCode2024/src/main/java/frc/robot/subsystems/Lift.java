// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;

public class Lift extends SubsystemBase {

    private CANSparkMax _lift_motor;
    private RelativeEncoder _lift_endcoder;
    private SparkPIDController _lift_pid;
    private TrapezoidProfile _lift_profile;

    /** Creates a new Lift. */
    public Lift() {
        _lift_motor = new CANSparkMax(Constants.Lift.motor_id, MotorType.kBrushless);
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
        _lift_endcoder.setPositionConversionFactor(Constants.Lift.LiftConversionPositionFactor);
        _lift_endcoder.setVelocityConversionFactor(Constants.Lift.LiftConversionVelocityFactor);
        _lift_pid.setP(Constants.Lift.liftKP);
        _lift_pid.setD(Constants.Lift.liftKD);
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
}
