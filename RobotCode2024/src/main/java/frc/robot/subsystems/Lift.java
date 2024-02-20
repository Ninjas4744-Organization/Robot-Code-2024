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

    private CANSparkMax _angle_motor;
    private RelativeEncoder _angle_endcoder;
    private SparkPIDController _angle_pid;
    private TrapezoidProfile _angle_profile;

    /** Creates a new Lift. */
    public Lift() {
        _angle_motor = new CANSparkMax(Constants.Lift.motor_id, MotorType.kBrushless);
        _angle_endcoder = _angle_motor.getEncoder();
        _angle_pid = _angle_motor.getPIDController();
        _angle_profile = new TrapezoidProfile(Constants.Lift.IntakeConstants);
        configDriveMotor();
    }

    private void configDriveMotor() {
        _angle_motor.restoreFactoryDefaults();
        _angle_motor.setInverted(Constants.Lift.toInvert);
        _angle_endcoder.setInverted(Constants.Lift.toInvert);
        _angle_motor.setIdleMode(Constants.Lift.driveNeutralMode);

        _angle_pid.setP(Constants.Lift.driveKP);
        _angle_pid.setD(Constants.Lift.driveKD);
        // burns to spark max
        _angle_motor.burnFlash();
        // resets encoder position to 0
        _angle_endcoder.setPosition(0.0);
    }

    public Command setPoseCommand(Supplier<TrapezoidProfile.State> goal) {
        return new TrapezoidProfileCommand(
                _angle_profile,
                pos -> {
                    _angle_pid.setReference(pos.position, ControlType.kPosition);
                }, goal, () -> {
                    return new TrapezoidProfile.State(_angle_endcoder.getPosition(), _angle_endcoder.getVelocity());
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
