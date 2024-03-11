package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.NinjaMotorController;
import frc.lib.drivers.NinjaMotorController.MotorControllerConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final int kJoystickPort = 0;
  public static final int kJoystick2Port = 1;
  public static final int kCurrentLimit = 40;

  public static final class Climber {
    public static final NinjaMotorController.NinjaMotorSubsystemConstants kClimberConstants = new NinjaMotorController.NinjaMotorSubsystemConstants();
    static {
        kClimberConstants.kSubsystemName = "Climber";

        kClimberConstants.kMasterConstants.invert = false; 

        kClimberConstants.kMasterConstants.id = 21;

        kClimberConstants.kSlaveConstants = new MotorControllerConstants[1];
        kClimberConstants.kSlaveConstants[0].id = 22;
        kClimberConstants.kSlaveConstants[0].invert = false;

        kClimberConstants.kSparkMaxMode = IdleMode.kBrake;

        kClimberConstants.kHomePosition = 1.3; // Stowed Position (inches)
        kClimberConstants.kRotationsPerUnitDistance = 10.6154;
        kClimberConstants.kSoftLimitDeadband = 0.05;

        kClimberConstants.Kp = 12;
        kClimberConstants.Kd = 0;
        kClimberConstants.Ki = 0;

        kClimberConstants.kEnableSupplyCurrentLimit = true;
        kClimberConstants.kSupplyCurrentLimit = 100;

        kClimberConstants.kMinUnitsLimit = 0;
        kClimberConstants.kMaxUnitsLimit = 0.4;
        kClimberConstants.kCruiseVelocity = 30;
        kClimberConstants.kAcceleration = 60;
        kClimberConstants.kGearRatio = 0.0023295454545455;
        
    }
   
    public static final int kLimitSwitchID = 2;

    public static final double kTrapChainHeight = 0.35;

  }

  public static final class Rotation {
    public static final NinjaMotorController.NinjaMotorSubsystemConstants kRotationConstants = new NinjaMotorController.NinjaMotorSubsystemConstants();
    static {
        kRotationConstants.kSubsystemName = "Rotation";

        kRotationConstants.kMasterConstants.invert = false; 

        kRotationConstants.kMasterConstants.id = 22;
        

        kRotationConstants.kSparkMaxMode = IdleMode.kBrake;

        kRotationConstants.kHomePosition = 1.3; // Stowed Position (inches)
        kRotationConstants.kRotationsPerUnitDistance = 10.6154;
        kRotationConstants.kSoftLimitDeadband = 0.05;

        kRotationConstants.Kp = 0.0185;
        kRotationConstants.Kd = 0.0002;
        kRotationConstants.Ki = 0.0;

        kRotationConstants.kEnableSupplyCurrentLimit = true;
        kRotationConstants.kSupplyCurrentLimit = 100;

        kRotationConstants.kMinUnitsLimit = 0;
        kRotationConstants.kMaxUnitsLimit = Double.MAX_VALUE;
        kRotationConstants.kCruiseVelocity = 60;
        kRotationConstants.kAcceleration = 120;
        kRotationConstants.kGearRatio = 7.2;
        
    }
    public static final int kLimitSwitchID = 9;

    public final static class States {
      public static final double kUpRotation = 120;
      public static final double kSourceOpenRotation = 70;
      public static final double kAmpOpenRotation = 0;
      public static final double kTrapOpenRotation = 87;
    }
  }

  public static final class Rollers {
    public static final int kMotorID = 23;
    public static final int kBeamBreakerID = 5;
    public static final double kTimeToOutake = 1;
    // public static final double kTimeToIntake = 0.25;
  }

  public static final class Elevator {
    public static final NinjaMotorController.NinjaMotorSubsystemConstants kElevatorConstants = new NinjaMotorController.NinjaMotorSubsystemConstants();
    static {
        kElevatorConstants.kSubsystemName = "Elevator";

        kElevatorConstants.kMasterConstants.invert = false; 

        kElevatorConstants.kMasterConstants.id = 24;
        

        kElevatorConstants.kSparkMaxMode = IdleMode.kBrake;

        kElevatorConstants.kHomePosition = 1.3; // Stowed Position (inches)
        kElevatorConstants.kRotationsPerUnitDistance = 10.6154;
        kElevatorConstants.kSoftLimitDeadband = 0.05;

        kElevatorConstants.Kp = 1.5;
        kElevatorConstants.Kd = 0.0;
        kElevatorConstants.Ki = 0.0;

        kElevatorConstants.kEnableSupplyCurrentLimit = true;
        kElevatorConstants.kSupplyCurrentLimit = 100;

        kElevatorConstants.kMinUnitsLimit = 0;
        kElevatorConstants.kMaxUnitsLimit = Double.MAX_VALUE;
        kElevatorConstants.kCruiseVelocity = 6;
        kElevatorConstants.kAcceleration = 12;
        kElevatorConstants.kGearRatio = (50/10/100*Math.PI)/16;
        
    }
    
    public static final class States {
      public static final double kSourceOpenHeight = 0.1;
      public static final double kAmpOpenHeight = 0.4;
      public static final double kTrapOpenHeight = 0.38;
      public static final double Close = 0;
    }

  }

  public static final class Swerve {
    public static final double kDriveCoefficient = 1;
    public static final double kDriveRotationCoefficient = 0.5;
    public static final double kActionCoefficient = 0.25;
    public static final double stickDeadband = 0.05;

    // public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(30);
    public static final double wheelBase = Units.inchesToMeters(30);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 35;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = ((wheelDiameter * Math.PI) / driveGearRatio);
    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;

    public static final double angleConversionFactor = 360.0 / 12.8;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final com.revrobotics.CANSparkBase.IdleMode angleNeutralMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
    public static final com.revrobotics.CANSparkBase.IdleMode driveNeutralMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 30;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 31;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 32;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = true;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kMaxSpeedMetersPerSecond = 3.6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration,
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
  }
}