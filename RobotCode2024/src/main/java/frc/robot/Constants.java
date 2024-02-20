package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.SwerveModuleConstants;

public class Constants {

  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 0;
  public static final String kJoystickPort = null;

  public static final class Swerve {
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

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
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
    public static final double driveConversionPositionFactor = ((wheelDiameter * Math.PI) / driveGearRatio);
    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;

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
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.109375)
          .rotateBy(Rotation2d.fromDegrees(180));
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      // supposed to be 19 at drive now its 4
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 22;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.06640625)
          .rotateBy(Rotation2d.fromDegrees(180));
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;

      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(200.56640625)
          .rotateBy(Rotation2d.fromDegrees(180));
      // public static final double / =0;
      public static final boolean isDriverEncoderInverted = true;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 24;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1.318359375)
          .rotateBy(Rotation2d.fromDegrees(180));
      public static final boolean isDriverEncoderInverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

  }
  public static final class Cameras{
    public static final class Camera1  {
      public static final String _name = "first";
      public static final Transform3d _camera_to_robot = new Transform3d();

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

  public static class VisionConstants {

    public static final String highRefreshRate = "";
    public static final Transform3d robotToCam = new Transform3d();

           
  }

  public static class Rollers {

    public static final int motor_id = 1;
    public static final int digital_input_id = 1;

    public static final double outake_voltage = 1;
    public static final double intake_voltage = -1;

  }

  public static class Intake {
    public static final int motor_id = 1;

    public static final double default_setpoint = 0;
    public static final double intake_setpoint = 0;
    public static final double outake_setpoint = 0;

    public static final Constraints IntakeConstants = new TrapezoidProfile.Constraints(0, 0);

    public static final boolean toInvert = false;

    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    public static final double driveConversionVelocityFactor = 0;

    public static final double driveConversionPositionFactor = 0;

    public static final double driveKP = 0;

    public static final double driveKI = 0;

    public static final double driveKD = 0;

    public static final double driveKFF = 0;

  }

  public static class Lift {
    public static final int motor_id = 1;

    public static final double default_setpoint = 0;
    public static final double intake_setpoint = 0;
    public static final double outake_setpoint = 0;

    public static final Constraints IntakeConstants = null;

    public static final boolean toInvert = false;

    public static final IdleMode driveNeutralMode = null;

    public static final double driveKP = 0;

    public static final double driveKD = 0;

  }

  public static class LEDs{
    
    public static final int LED_LENGTH = 0;

    public static final int FL_LED_INDEX =0;
    public static final int FR_LED_INDEX =0;
    public static final int BL_LED_INDEX =0;
    public static final int BR_LED_INDEX =0;

    public static final int FL_PORT = 0;
    public static final int FR_PORT = 0;
    public static final int BL_PORT = 0;
    public static final int BR_PORT = 0;
  }
}
