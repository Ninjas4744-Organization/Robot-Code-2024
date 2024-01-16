package frc.robot;

public final class Constants {
  public static final int kJoystickPort = 0;
  public static final int kElevatorMotor1 = 2;
  public static final int kElevatorMotor2 = 4;
  public static final int kElevatorLimitSwitch = 1;
  public static final int kInOutTakeMotor1 = 2; //temporary until InOutTake prototype is ready
  public static final int kInOutTakeMotor2 = 4; //temporary until InOutTake prototype is ready
  public static final int kInOutTakeLimitSwitch = 0; //temporary until InOutTake prototype is ready

  public static final double kMaxElevator = 10;
  public static final double kMotionMagicCruiseVelocity = 80;
  public static final double kMotionMagicAcceleration = 160;
  public static final double kMotionMagicJerk = 1600;
  public static final double kP = 4.8;
  public static final double kI = 0;
  public static final double kD = 0.1;
  public static final double kV = 0.12;
  public static final double kS = 0.25;
  public static final double kSensorToMechanismRatio = 12.8;
}