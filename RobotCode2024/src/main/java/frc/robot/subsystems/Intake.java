package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    TalonFX _intake;

    /** Creates a new Intake. */
    public Intake() {
        _intake = new TalonFX(Constants.Intake.motor_id);
        _intake.setPosition(Constants.Intake.default_setpoint);

    }

    public void setPose(double pos) {
        MotionMagicVoltage _req = new MotionMagicVoltage(pos);
        _intake.setControl(_req);

    }

    public Command defaultCommand() {
        return Commands.runOnce(() -> setPose(Constants.Intake.default_setpoint), this);
    }

    public Command intakeConfiguration() {
        return Commands.runOnce(() -> setPose(Constants.Intake.intake_setpoint), this);
    }

    public Command outakeConfiguration() {
        return Commands.runOnce(() -> setPose(Constants.Intake.outake_setpoint), this);
    }
}