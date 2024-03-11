// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public abstract class NinjaMotorController {
  protected ControlState _controlState;
  protected GenericEntry position, velocity, setpoint, posError, velError, currentControl;
  protected NinjaMotorSubsystemConstants _constants;
  protected double demand;

  protected NinjaMotorController(NinjaMotorSubsystemConstants constants) {
    _constants = constants;

    position = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("position", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    velocity = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("velocity", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    setpoint = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("setpoint", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    posError = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("posError", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    velError = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("velError", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    currentControl = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("currentControl", "")
        .withWidget("Text View")
        .withSize(3, 3)
        .getEntry();

    setPIDconstants(_constants.kPositionKp, _constants.kPositionKi, _constants.kPositionKd, _constants.kPositionKIzone);
    setForwardSoftLimit(_constants.kMaxUnitsLimit);

    setReverseSoftLimit(_constants.kMinUnitsLimit);
  }

  /** Creates a new NinjasSubsytem. */
  public void writeToLog() {
  }

  // Optional design pattern for caching periodic reads to avoid hammering the
  // HAL/CAN.
  public void readPeriodicInputs() {
  }

  // Optional design pattern for caching periodic writes to avoid hammering the
  // HAL/CAN.
  public void writePeriodicOutputs() {
  }

  public void zeroSensors() {
  }

  public abstract void setForwardSoftLimit(float sofLimit);

  public abstract void setReverseSoftLimit(float sofLimit);

  public abstract void setPIDconstants(double Kp, double Ki, double Kd, double KIzone);

  public abstract double getP();

  public abstract double getI();

  public abstract double getD();

  public abstract double getIzone();

  public abstract double getPosition();

  public abstract void set(double percentage);

  public abstract void set(State pos);

  public abstract double get();

  public abstract void stop();

  public abstract void setPosition(double pos);

  public abstract double getVelError();

  public abstract double getSetpoint();
  public abstract boolean atHomingLocation();
  


  public void outputTelemetry(boolean testing) {
    if (testing) {
      position.setDouble(getPosition());
      currentControl.setString(_controlState.toString());
      setpoint.setDouble(getSetpoint());
      velError.setDouble(getVelError());
    }
  }

  
  

  protected enum ControlState {
    OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
  }

  // Recommend initializing in a static block!
  public static class MotorControllerConstants {
    public int id = -1;
    public boolean invert = false;

  }

  // Recommend initializing in a static block!
  public static class NinjaMotorSubsystemConstants {
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";

    public double kLooperDt = 0.01;
    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    public MotorControllerConstants kMasterConstants = new MotorControllerConstants();
    public MotorControllerConstants[] kSlaveConstants = new MotorControllerConstants[0];

    
    public double kHomePosition = 0.0; // Units
    public double kRotationsPerUnitDistance = 1.0;
    public double kSoftLimitDeadband = 0.0;

    public double kPositionKp = 0;
    public double kPositionKi = 0;
    public double kPositionKIzone = 0;
    public double kPositionKd = 0;
    public double kPositionKf = 0;
    public int kPositionDeadband = 0; // Ticks

    public double kVelocityFeedforward = 0;
    public double kArbitraryFeedforward = 0;
    public double kCruiseVelocity = 0; // Units/s
    public double kAcceleration = 0; // Units/s^2
    public double kJerk = 0; // Units/s^3
    public double kRampRate = 0.0; // s
    public double kMaxVoltage = 12.0;

    public int kSupplyCurrentLimit = 60; // amps
    public boolean kEnableSupplyCurrentLimit = false;

    public int kStatorCurrentLimit = 40; // amps
    public boolean kEnableStatorCurrentLimit = false;

    public float kMaxUnitsLimit = Float.POSITIVE_INFINITY;
    public float kMinUnitsLimit = Float.NEGATIVE_INFINITY;

    public int kStatusFrame8UpdateRate = 1000;
  }
}
