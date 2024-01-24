package frc.robot;

public final class Constants {
  //Constants
  public static final double kMaxElevator = 100;
  public static final double kMaxInOutTakeElevator = 100;
  public static final double kTimeToOpenCloseSystem = 3;
  public static final double kTimeToClimbUntilTrap = 3;
  public static final double kSourceOpenHeight = 90;
  public static final double kSourceOpenRotation = 100;
  public static final double kAmpOpenHeight = 40;
  public static final double kAmpOpenRotation = 50;
  public static final double kTrapOpenHeight = 60;
  public static final double kTrapOpenRotation = 70;
  
  //Ports
  public static final int kJoystickPort = 0;
  public static final int kJoystick2Port = 1;
  
  //Elevator Ports
  public static final int kElevatorMotor1 = 2;
  public static final int kElevatorMotor2 = 4;
  public static final int kElevatorLimitSwitch = 0;
  
  //InOutTake Ports
  public static final int kInOutTakeMotor = 5;
  public static final int kInOutTakeElevatorMotor = 3;
  public static final int kInOutTakeRotationMotor = 1;
  public static final int kInOutTakeLimitSwitchElevator = 1;
  public static final int kInOutTakeLimitSwitchRotation = 2;
  public static final int kInOutTakeBeamBreakerNote = 9;
}