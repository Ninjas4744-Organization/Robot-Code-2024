package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final double kMaxClimber = 100;
  public static final double kTimeToOutake = 1;
  public static final double kFloorUpPositon = 55;

  public final static class IntakeStates{
    public static final double kSourceOpenHeight = 90;
    public static final double kSourceOpenRotation = 100;
    public static final double kAmpOpenHeight = 40;
    public static final double kAmpOpenRotation = 50;
    public static final double kTrapOpenHeight = 60;
    public static final double kTrapOpenRotation = 70;
  }
  
  public final static class Ports{
    public static final class Joystick{
      public static final int kJoystickPort = 0;
      public static final int kJoystick2Port = 1;
    }

    public static final class Climber{
      public static final int kMotor1 = 2;
      public static final int kMotor2 = 4;
      public static final int kLimitSwitch = 0;
    }

    public static final class Intake{
      public static final int kRollersMotor = 5;
      public static final int kElevatorMotor = 3;
      public static final int kRotationMotor = 1;
      public static final int kLimitSwitchElevator = 1;
      public static final int kLimitSwitchRotation = 2;
      public static final int kBeamBreaker = 9;
    }

    public static final class FloorIntake{
      public static final int kRollersMotor = 5;
      public static final int kRotationMotor = 5;
      public static final int kLimitSwitch = 2;
      public static final int kBeamBreaker = 9;
    }
  }

  public static final class PIDConstants {
    public static final class Climber{
      public static final double kP = 4.8;
      public static final double kI = 0.0;
      public static final double kD = 0.01;

      public static final double kMaxVelocity = 80;
      public static final double kMaxAcceleration = 160;
      public static final TrapezoidProfile.Constraints kConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
    }

    public static final class IntakeRotation{
      public static final double kP = 4.8;
      public static final double kI = 0.0;
      public static final double kD = 0.01;

      public static final double kMaxVelocity = 80;
      public static final double kMaxAcceleration = 160;
      public static final TrapezoidProfile.Constraints kConstraints = 
      new Constraints(kMaxVelocity, kMaxAcceleration);
    }

    public static final class IntakeElevator{
      public static final double kP = 4.8;
      public static final double kI = 0.0;
      public static final double kD = 0.01;

      public static final double kMaxVelocity = 80;
      public static final double kMaxAcceleration = 160;
      public static final TrapezoidProfile.Constraints kConstraints = 
      new Constraints(kMaxVelocity, kMaxAcceleration);
    }
  }
  
  public static final class Swerve {
    public static final double kDriveCoefficient = 0.7;
    public static final double stickDeadband = 0.1;

    // public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(30);
    public static final double wheelBase = Units.inchesToMeters(30);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 60;

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
    public static final double driveConversionPositionFactor =
        ((wheelDiameter * Math.PI) / driveGearRatio);
    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor =
        ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
        
    public static final double angleConversionFactor = 360.0 / 12.8;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3.6; // meters per second
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
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 21;
        public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.616);
        public static final boolean isDriverEncoderInverted = false;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 22;
        public static final Rotation2d angleOffset = Rotation2d.fromRadians(-Math.PI/2+0.267);
        public static final boolean isDriverEncoderInverted = false;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(Math.PI/4+0.778);
    //   public static final double / =0;
      public static final boolean isDriverEncoderInverted = true;

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 24;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.635);
      public static final boolean isDriverEncoderInverted = false;

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
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
    public static final PathConstraints constraints =
        new PathConstraints(maxVelocity, maxAcceleration, kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
  }
  
  public static class VisionConstants{
    public static final String cameraName = "";
    public static final Transform3d robotToCam = new Transform3d();
  }
}