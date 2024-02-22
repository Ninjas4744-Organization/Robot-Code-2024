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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Ports;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class IntakeElevator extends SubsystemBase {
  private CANSparkMax _motor;
  private DigitalInput _limitSwitch;
  private SparkPIDController _controller;

  private final MutableMeasure<Voltage> _appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> _distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> _velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine _sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(0.5)),
            Volts.of(4),
            Seconds.of(4)),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        // set states for all 4 modules
                        _motor.setVoltage(volts.in(Volts));

                    },
                    log -> {
                        // set states for all 4 modules

                        log.motor("elevator")
                                .voltage(
                                        _appliedVoltage.mut_replace(
                                                _motor.getBusVoltage() * RobotController.getBatteryVoltage(),
                                                Volts))
                                .linearPosition(_distance.mut_replace(_motor.getEncoder().getPosition(), Meters))
                                .linearVelocity(_velocity.mut_replace(_motor.getEncoder().getVelocity(), MetersPerSecond));

                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

  public IntakeElevator() {
    _motor = new CANSparkMax(Ports.Intake.kElevatorMotor, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motor.setInverted(true);
    _motor.getEncoder().setPositionConversionFactor(Constants.PIDConstants.IntakeElevator.kConversionPosFactor);
    _motor.getEncoder().setVelocityConversionFactor(Constants.PIDConstants.IntakeElevator.kConversionVelFactor);
    _limitSwitch = new DigitalInput(Ports.Intake.kLimitSwitchElevator);

    _controller = _motor.getPIDController();
    _controller.setP(PIDConstants.IntakeElevator.kP);
    _controller.setI(PIDConstants.IntakeElevator.kI);
    _controller.setD(PIDConstants.IntakeElevator.kD);

    _controller.setOutputRange(-0.6, 1);

    _motor.burnFlash();
  }

  public Command setHeight(double height) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(PIDConstants.IntakeElevator.kConstraints),
        output -> _controller.setReference(output.position, ControlType.kPosition),
        () -> new TrapezoidProfile.State(height, 0),
        () -> new TrapezoidProfile.State(getHeight(), 0),
        this);
  }

  public void setElevatorMotor(double percent) {
    _motor.set(percent);
  }

  public double getHeight() {
    return _motor.getEncoder().getPosition();
  }

  public boolean isMax(double max) {
    return Math.abs(max - getHeight()) < 0.2;
  }

  public void stopElevator() {
    _motor.stopMotor();;
  }

  public Command Override() {
    return Commands.startEnd(
      () -> {_motor.stopMotor();},
      () -> {Reset();},
      this
    );
  }

  public void Reset() {
    if (this.getCurrentCommand() != null)
      this.getCurrentCommand().cancel();
  }

  @Override
  public void periodic() {
    if (!_limitSwitch.get())// Check ! later
      _motor.getEncoder().setPosition(0);

    SmartDashboard.putNumber("Intake Elevator Height", getHeight());
  }

  public Command runOpen(double height) {
    return setHeight(height);
  }

  public Command runClose() {
    return setHeight(0);
  }

  public Command runOpenClose(double height) {
    return Commands.either(
        runOpen(height),
        runClose(),
        () -> {
          return this.getHeight() == 0;
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.dynamic(direction);
  }
}