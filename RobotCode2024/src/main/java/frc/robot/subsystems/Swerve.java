// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class Swerve extends SubsystemBase {

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private Pose2d _lastUnFilterred;
  private AHRS gyro;
  private Supplier<List<Optional<EstimatedRobotPose>>> _estimationsSupplier;
  public Field2d m_field = new Field2d();
  public Field2d m_field_tag = new Field2d();
  public Field2d m_field_odo = new Field2d();
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            // set states for all 4 modules
            for (SwerveModule mod : mSwerveMods) {
              mod.sysIdVolt(volts.in(Volts));
            }
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // set states for all 4 modules
            for (SwerveModule mod : mSwerveMods) {
              log.motor("drive-" + mod.moduleNumber)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          mod.getVoltage() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(mod.getPosition().distanceMeters, Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(mod.getState().speedMetersPerSecond, MetersPerSecond));
            }

          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("drive")
          this));
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();

  private Field2d m_field_solution = new Field2d();
  public SwerveDrivePoseEstimator _estimator;
  private String _commandKey = "Outake";

  /** Creates a new SwerveSubsystem. */
  public Swerve(Supplier<List<Optional<EstimatedRobotPose>>> estimationsSupplier) {

    gyro = new AHRS();
    gyro.getRotation2d();
    zeroGyro();
    _estimationsSupplier = estimationsSupplier;
    // Creates all four swerve modules into a swerve drive
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod3.constants),
        new SwerveModule(1, Constants.Swerve.Mod2.constants),
        new SwerveModule(2, Constants.Swerve.Mod1.constants),
        new SwerveModule(3, Constants.Swerve.Mod0.constants)
    };
    Pose2d initPose = LimelightHelpers.getBotPose2d_wpiRed(null);
    _lastUnFilterred = initPose;
    // creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    resetOdometry(initPose);
    _estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), initPose);

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
            new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0), // Rotation PID constants
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.Swerve.trackWidth, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        // fancy way to do an if else statement
        // if field relative == true, use field relative stuff, otherwise use robot
        // centric
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void drive(ChassisSpeeds targetSpeed)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeed,
        new Translation2d());
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);// MIGHT NEED TO CHANGE
    }
  }

  public ChassisSpeeds getChassisSpeeds() {

    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
        // front left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        // front right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
        // back left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
        // back right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
        path,
        this::getPose, // Robot pose supplier
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
            3.6, // Max module speed, in m/s
            0.7, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            System.out.println(alliance.get());
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(360 - gyro.getFusedHeading());
  }

  public Rotation2d getYaw() {
    // fancy if else loop again
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : gyro.getRotation2d();
  }

  public String getCommandKey() {
    return _commandKey;
  }

  public Pose2d getLastCalculatedPosition() {
    return _estimator.getEstimatedPosition();
  }

  private void updatePV() {
    for (Optional<EstimatedRobotPose> estimation : _estimationsSupplier.get()) {
      estimation.ifPresent(pose -> {
        _estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

      });
    }
  }

  @Override
  public void periodic() {
    _commandKey = "";
    if (LimelightHelpers.getTV(null)) {
      if (LimelightHelpers.getBotPose2d_wpiRed(null).getX() != 0) {
        _lastUnFilterred = LimelightHelpers.getBotPose2d_wpiRed(null);
      }
      _estimator.addVisionMeasurement(_lastUnFilterred, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      // updatePV();

    } else {

      _lastUnFilterred = _estimator.getEstimatedPosition();
    }
    publisher.set(getLastCalculatedPosition());
    _estimator.update(getYaw(), getPositions());
    swerveOdometry.update(getYaw(), getPositions());
    m_field_solution.setRobotPose(getLastCalculatedPosition());
    SmartDashboard.putData("Field_Estimation", m_field_solution);
    for (SwerveModule mod : mSwerveMods) {
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Integrated",
      // mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Velocity",
      // mod.getState().speedMetersPerSecond);
    }
    // TODO Auto-generated method stub
    super.periodic();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

// commap.put("7",new DeferredCommand(() -> {
// Pose2d current_pos = _swerve.getLastCalculatedPosition();

// // Pose2d current_tag_pose = new Pose2d(new

// Pose2d current_tag_pose = _vision._currentTag.pose.toPose2d();
// Rotation2d target_rot =
// current_tag_pose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

// List<Translation2d> bezierPoints;
// if(current_tag_pose.minus(current_tag_pose).getRotation().getDegrees()>50){
// bezierPoints = PathPlannerPath.bezierFromPoses(
// current_pos,
// new Pose2d(new Translation2d(
// current_pos.getX()+0.2,// ID 1 aprilTag location
// current_pos.getY() ),target_rot),
// new Pose2d(new Translation2d(
// current_pos.getX(),// ID 1 aprilTag location
// current_tag_pose.getY()),target_rot),
// new Pose2d(new Translation2d(
// current_tag_pose.getX()+2,// ID 1 aprilTag location
// current_tag_pose.getY()),target_rot)
// );
// }
// else{
// bezierPoints = PathPlannerPath.bezierFromPoses(
// current_pos,

// new Pose2d(new Translation2d(
// current_tag_pose.getX()+0.5,// ID 1 aprilTag location
// current_tag_pose.getY()),target_rot)
// );
// }

// // Create the path using the bezier points created above

// PathPlannerPath path = new PathPlannerPath(
// bezierPoints,
// Constants.AutoConstants.constraints,
// new GoalEndState(0.0,
// current_tag_pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
// return new InstantCommand(() -> {

// _swerve.resetOdometry(current_pos);

// })
// .andThen(AutoBuilder.followPath(path))
// ;
// }, Set.of(_swerve)));
