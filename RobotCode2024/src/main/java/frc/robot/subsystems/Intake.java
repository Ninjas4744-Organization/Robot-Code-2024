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

public class Intake extends SubsystemBase {

    private CANSparkMax _lift_motor;
    private RelativeEncoder _lift_endcoder;
    private SparkPIDController _lift_pid;
    private TrapezoidProfile _lift_profile;

    /** Creates a new Intake. */
    public Intake() {
        _lift_motor = new CANSparkMax(Constants.Intake.motor_id, MotorType.kBrushless);
        _lift_endcoder = _lift_motor.getEncoder();
        _lift_pid = _lift_motor.getPIDController();
        _lift_profile = new TrapezoidProfile(Constants.Intake.IntakeConstants);
        configDriveMotor();
    }

    private void configDriveMotor() {
        _lift_motor.restoreFactoryDefaults();
        _lift_motor.setInverted(Constants.Intake.toInvert);
        _lift_endcoder.setInverted(Constants.Intake.toInvert);
        _lift_motor.setIdleMode(Constants.Intake.driveNeutralMode);

        _lift_pid.setP(Constants.Intake.driveKP);
        _lift_pid.setD(Constants.Intake.driveKD);
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
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.intake_setpoint, 0));
    }

    public Command outakeConfiguration() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.outake_setpoint, 0));
    }

    public Command defaultCommand() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.default_setpoint, 0));
    }
}