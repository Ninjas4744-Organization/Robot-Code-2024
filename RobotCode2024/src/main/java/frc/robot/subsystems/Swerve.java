package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private Pose2d _lastUnFilterred;
  private AHRS gyro;
  public Field2d m_field = new Field2d();
  public Field2d m_field_tag = new Field2d();
  public Field2d m_field_odo = new Field2d();
  final int smaplesCamer = 10;
  final int smaplesCOmbine = 5;

  LinearFilter filterX = LinearFilter.movingAverage(smaplesCamer);
  LinearFilter filterY = LinearFilter.movingAverage(smaplesCamer);
  LinearFilter filterROT = LinearFilter.movingAverage(smaplesCamer);

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  private Field2d m_field_solution = new Field2d(); 
  public SwerveDrivePoseEstimator _estimator;
  private String _commandKey ="Outake";
  
  public Swerve() {    
    gyro = new AHRS();
    gyro.getRotation2d();
    zeroGyro();

    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    Pose2d initPose = LimelightHelpers.getBotPose2d_wpiRed(null);
    _lastUnFilterred = initPose;
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    resetOdometry(initPose);
    _estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), initPose);// ROBOT MUST BE WITH FACE (LIMELIGHT) TOWARDS TAG AND CLOSE (1 METER MAX)

    AutoBuilder.configureHolonomic(
      this::getLastCalculatedPosition, // Robot pose supplier
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
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return true;
      }, this
    );
  }

  public void Override(){
    //Idk add later...
  }

  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          //if field relative, use field relative stuff, otherwise use robot centric
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
    setModuleStates(swerveModuleStates);
  }

  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  public void drive(ChassisSpeeds targetSpeed){
    SwerveModuleState[] swerveModuleStates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeed, new Translation2d());
    
      setModuleStates(swerveModuleStates);
  }

  public Command followPath(List<Translation2d> path){
    return AutoBuilder.followPath(new PathPlannerPath(
      path,
      Constants.AutoConstants.constraints,
      new GoalEndState(0, getHeading())
    ));
  }

  public Command goToTag(Vision _vision){
    if(_vision.getTag() != null)
      return null;
      
    Pose2d targetPose = _vision.getTagPose();
    Pose2d currentPos = getLastCalculatedPosition();
   
    return followPath(PathPlannerPath.bezierFromPoses(currentPos, targetPose));
  }

  public ChassisSpeeds getChassisSpeeds(){
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

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(360 -gyro.getFusedHeading());
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : gyro.getRotation2d();
  }

  public String getCommandKey(){
    return _commandKey;
  }

  public Pose2d getLastCalculatedPosition(){
    return _estimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    _commandKey = "";
    
    if(LimelightHelpers.getTV(null) ){
      if(LimelightHelpers.getBotPose2d_wpiRed(null).getX() != 0 )
        _lastUnFilterred = LimelightHelpers.getBotPose2d_wpiRed(null);
        
      _estimator.addVisionMeasurement(_lastUnFilterred, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
    else
      _lastUnFilterred = _estimator.getEstimatedPosition();

    publisher.set(getLastCalculatedPosition());
    _estimator.update(getYaw(), getPositions());
    swerveOdometry.update(getYaw(), getPositions());
    m_field_solution.setRobotPose(getLastCalculatedPosition());
    SmartDashboard.putData("Field_Estimation", m_field_solution);
  }
}