package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
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
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class Intake extends SubsystemBase {

    private CANSparkMax _angle_motor;
    private RelativeEncoder _angle_endcoder;
    private SparkPIDController _angle_pid;
    private TrapezoidProfile _angle_profile;
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_distance = mutable(Degrees.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(0.5)),
            Volts.of(4),
            Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        // set states for all 4 modules
                        _angle_motor.setVoltage(volts.in(Volts));

                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // set states for all 4 modules

                        log.motor("angle")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                _angle_motor.getBusVoltage() * RobotController.getBatteryVoltage(),
                                                Volts))
                                .angularPosition(m_distance.mut_replace(_angle_endcoder.getPosition(), Degrees))
                                .angularVelocity(
                                        m_velocity.mut_replace(_angle_endcoder.getVelocity(), DegreesPerSecond));

                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

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
        // _angle_endcoder.setInverted(Constants.Intake.toInvert);
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
    public void setPercent(double percent){
        _angle_motor.set(percent);
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}