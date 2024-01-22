package frc.robot;

public final class Constants {
  //Constants
  public static final double kMaxElevator = 100;
  public static final double kMaxInOutTakeElevator = 100;
  public static final double kTimeToOpenCloseSystem = 1;
  public static final double kTimeToClimbUntilTrap = 3;
  public static final double kSourceOpenHeight = 9;
  public static final double kSourceOpenRotation = 10;
  public static final double kAmpOpenHeight = 4;
  public static final double kAmpOpenRotation = 5;
  public static final double kTrapOpenHeight = 6;
  public static final double kTrapOpenRotation = 7;
  
  //Ports
  public static final int kJoystickPort = 0;
  
  //Elevator Ports
  public static final int kElevatorMotor1 = 2;
  public static final int kElevatorMotor2 = 4;
  public static final int kElevatorLimitSwitch = 0;
  
  //InOutTake Ports
  public static final int kInOutTakeMotor = 5;
  public static final int kInOutTakeElevatorMotor = 3;
  public static final int kInOutTakeRotationMotor = 1;
  public static final int kInOutTakeLimitSwitchElevator = 0;
  public static final int kInOutTakeLimitSwitchTop = 0;
  public static final int kInOutTakeLimitSwitchBottom = 0;
  public static final int kInOutTakeBeamBreakerNote = 0;
}