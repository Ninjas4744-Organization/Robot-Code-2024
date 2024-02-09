// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    TalonFX _master;
    Mechanism2d mechanism = new Mechanism2d(3, 3);

    
    /** Creates a new Lift. */
    public Lift() {
        _master = new TalonFX(Constants.Lift.motor_id);
        _master.setPosition(Constants.Lift.default_setpoint);
    }

    public void setPose(double pos) {
        MotionMagicVoltage _req = new MotionMagicVoltage(pos);
        
        _master.setControl(_req);

    }

    public Command defaultCommand() {
        return Commands.runOnce(() -> setPose(Constants.Lift.default_setpoint), this);
    }

    public Command intakeConfiguration() {
        return Commands.runOnce(() -> setPose(Constants.Lift.intake_setpoint), this);
    }

    public Command outakeConfiguration() {
        return Commands.runOnce(() -> setPose(Constants.Lift.outake_setpoint), this);
    }
}
