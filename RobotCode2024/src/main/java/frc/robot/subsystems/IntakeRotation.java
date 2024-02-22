package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Ports;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class IntakeRotation extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  private final MutableMeasure<Voltage> _appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Angle> _distance = mutable(Degrees.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Angle>> _velocity = mutable(DegreesPerSecond.of(0));

  private final SysIdRoutine _sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(0.05).per(Seconds.of(0.25)),
            Volts.of(1),
            Seconds.of(7.5)),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        // set states for all 4 modules
                        _motor.setVoltage(volts.in(Volts));

                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // set states for all 4 modules

                        log.motor("angle")
                                .voltage(
                                        _appliedVoltage.mut_replace(
                                                _motor.getAppliedOutput() * RobotController.getBatteryVoltage(),
                                                Volts))
                                .angularPosition(_distance.mut_replace(_motor.getEncoder().getPosition(), Degrees))
                                .angularVelocity(_velocity.mut_replace(_motor.getEncoder().getVelocity(), DegreesPerSecond));

                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));
                    


  public IntakeRotation() {
    _motor = new CANSparkMax(Ports.Intake.kRotationMotor, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.getEncoder().setPositionConversionFactor(Constants.PIDConstants.IntakeRotation.kConversionPosFactor);
    _motor.getEncoder().setVelocityConversionFactor(Constants.PIDConstants.IntakeRotation.kConversionVelFactor);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchRotation);

    _controller = _motor.getPIDController();
    _controller.setP(PIDConstants.IntakeRotation.kP);
    _controller.setI(PIDConstants.IntakeRotation.kI);
    _controller.setD(PIDConstants.IntakeRotation.kD);
    _controller.setIZone(3);

    _controller.setOutputRange(-0.55, 0.55);

    _motor.burnFlash();
  }

  public Command setRotation(double rotation) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(PIDConstants.IntakeRotation.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(rotation, 0),
        () -> new TrapezoidProfile.State(getRotation(), 0),
        this);
  }

  public double getRotation() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max) {
    return Math.abs(max - getRotation()) < 0.2;
  }

  public void setRotationMotor(double percent) {
    _motor.set(percent);
  }

  public void stopRotation() {
    _motor.set(0);
  }

  public Command Override() {
    return setRotation(getRotation());
  }

  public void Reset() {
    if (this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Rotation", getRotation());
    SmartDashboard.putNumber("Intake Rotation Velocity", _motor.getEncoder().getVelocity());
  }

  public Command runOpen(double rotation) {
    return setRotation(rotation);
  }

  public Command runClose() {
    return this.setRotation(0);
  }

  public Command runOpenClose(double rotation) {
    return Commands.either(
        runOpen(rotation),
        runClose(),
        () -> {
          return this.getRotation() == 0;
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.dynamic(direction);
  }
}