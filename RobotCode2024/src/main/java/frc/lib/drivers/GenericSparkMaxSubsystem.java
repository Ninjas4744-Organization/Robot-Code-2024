// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

public class GenericSparkMaxSubsystem extends NinjaMotorController {

  protected final CANSparkMax _master;
  protected final CANSparkMax[] _slaves;

  protected final RelativeEncoder _relEncoder;
  // protected final AbsoluteEncoder _absEncoder;
  protected final SparkPIDController _controller;

  private final TrapezoidProfile _profile;
  private final Timer trapozoidTimer = new Timer();
  
  /** Creates a new GenericMotorSubsystem. */
  public GenericSparkMaxSubsystem(final NinjaMotorSubsystemConstants constants) {
    super(constants);

    _constants = constants;
    _master = SparkMAXFactory.createDefaultSparkMax(_constants.kMasterConstants.id);
    _slaves = new CANSparkMax[_constants.kSlaveConstants.length];

    _relEncoder = _master.getEncoder();
    // _absEncoder = _master.getAbsoluteEncoder(Spark);

    _profile = new TrapezoidProfile(new Constraints(_constants.kCruiseVelocity, _constants.kAcceleration));
    _controller = _master.getPIDController();
    _relEncoder.setPositionConversionFactor(constants.kGearRatio);
    _relEncoder.setVelocityConversionFactor(constants.kGearRatio/60);
    _master.burnFlash();

    for (int i = 0; i < _slaves.length; ++i) {
      _slaves[i] = SparkMAXFactory.createPermanentSlaveSparkMax(_constants.kSlaveConstants[i].id, _master,
          _constants.kSlaveConstants[i].invert);
      _slaves[i].burnFlash();
    }

    // Send a neutral command.
    stop();
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    if (_controlState == ControlState.MOTION_MAGIC) {
      _controller.setReference(_profile.calculate(trapozoidTimer.get(), new State(getPosition(), 0), new State(demand, 0)).position, ControlType.kPosition);
    } else if (_controlState == ControlState.POSITION_PID || _controlState == ControlState.MOTION_PROFILING) {
      _controller.setReference(demand, ControlType.kPosition);
    } else {
      _master.set(demand);
    }

  }

  @Override
  public void setForwardSoftLimit(double sofLimit) {
    _master.setSoftLimit(SoftLimitDirection.kForward, (float)sofLimit);
    _master.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  @Override
  public void setReverseSoftLimit(double sofLimit) {
    _master.setSoftLimit(SoftLimitDirection.kReverse, (float)sofLimit);
    _master.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public boolean atHomingLocation() {
    return getPosition() - demand == 0;
  }

  public synchronized String getControlState() {
    return _controlState.toString();
  }

  @Override
  public void stop() {
    set(0.0);
    _master.stopMotor();
  }

  public synchronized void setSupplyCurrentLimit(int value) {
    _master.setSmartCurrentLimit(value);
  }

  @Override
  public void set(double percentage) {
    if (_controlState != ControlState.OPEN_LOOP) {
      _controlState = ControlState.OPEN_LOOP;
    }
    demand = percentage;
  }

  @Override
  public void set(State pos) {
    if (_controlState != ControlState.MOTION_MAGIC) {
      _controlState = ControlState.MOTION_MAGIC;
    }
    trapozoidTimer.restart();
    demand = pos.position;
  }

  public synchronized double getVelError() {
    return _relEncoder.getVelocity() - demand;
  }

  public synchronized double getSetpoint() {
    return (_controlState == ControlState.MOTION_MAGIC ||
        _controlState == ControlState.POSITION_PID ||
        _controlState == ControlState.MOTION_PROFILING) ? demand : Double.NaN;
  }

  @Override
  public synchronized double getPosition() {
    return _relEncoder.getPosition();
  }

  @Override
  public double getD() {
    return _controller.getD();
  }

  @Override
  public double getI() {
    return _controller.getI();
  }

  @Override
  public double getIzone() {
    return _controller.getIZone();
  }

  @Override
  public double getP() {
    return _controller.getP();
  }

  @Override
  public double get() {
    return _master.get();
  }

  @Override
  public void setPIDconstants(double Kp, double Ki, double Kd, double KIzone) {
    _controller.setP(Kp, 0);
    _controller.setI(Ki, 0);
    _controller.setIZone(KIzone, 0);
    _controller.setD(Kd, 0);
  }

  @Override
  public void setPosition(double pos) {
    _relEncoder.setPosition(pos);
  }
  @Override
  public void setHomingPosition(double pos) {
    if (_controlState != ControlState.POSITION_PID) {
      _controlState = ControlState.POSITION_PID;
    }
    demand = pos;
  }

  public static GenericSparkMaxSubsystem createSparkMaxMotorGroup(final NinjaMotorSubsystemConstants constants){
    return new GenericSparkMaxSubsystem(constants);
  }

  
  
}
