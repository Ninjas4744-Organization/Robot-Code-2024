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

    private CANSparkMax _angle_motor;
    private RelativeEncoder _angle_endcoder;
    private SparkPIDController _angle_pid;
    private TrapezoidProfile _angle_profile;

    /** Creates a new Intake. */
    public Intake() {
        _angle_motor = new CANSparkMax(Constants.Intake.motor_id, MotorType.kBrushless);
        _angle_endcoder = _angle_motor.getEncoder();
        _angle_pid = _angle_motor.getPIDController();
        _angle_profile = new TrapezoidProfile(Constants.Intake.IntakeConstants);
        configintakeMotor();
    }

    private void configintakeMotor() {
        _angle_motor.restoreFactoryDefaults();
        _angle_motor.setInverted(Constants.Intake.toInvert);
        _angle_endcoder.setInverted(Constants.Intake.toInvert);
        _angle_motor.setIdleMode(Constants.Intake.intakeNeutralMode);
        _angle_endcoder.setPositionConversionFactor(Constants.Intake.IntakeConversionPositionFactor);
        _angle_endcoder.setVelocityConversionFactor(Constants.Intake.IntakeConversionVelocityFactor);
        _angle_pid.setP(Constants.Intake.intakeKP);
        _angle_pid.setD(Constants.Intake.intakeKD);
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
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.intake_setpoint, 0));
    }

    public Command outakeConfiguration() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.outake_setpoint, 0));
    }

    public Command defaultCommand() {
        return setPoseCommand(() -> new TrapezoidProfile.State(Constants.Intake.default_setpoint, 0));
    }
}