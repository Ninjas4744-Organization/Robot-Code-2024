// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldTagsFilter;
import frc.lib.util.PointWithTime;
import frc.lib.util.LimelightHelpers;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout CURRENT_FIELD_LAYOUT;
  private AprilTag _currentTag;

  private boolean _redOrBlue;
  private PointWithTime _currentEstimations;
  private Pose2d _currentSource;
  private FieldTagsFilter _ids;

  private GenericEntry leftsource, centersource, rightsource;
  private boolean left, center, right;

  /** Creates a new Vision. */
  public Vision() {

    try {
      CURRENT_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        _redOrBlue = (alliance.get() == DriverStation.Alliance.Blue);
        if (_redOrBlue) {
                  System.out.println("BLUE");

          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } else {
                  System.out.println("RED");

          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }

      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    leftsource = Shuffleboard.getTab("Dashboard")
        .add("LEFT SOURCE", true)
        .withWidget("Toggle Button")
        .withSize(3, 3)
        .getEntry();
    centersource = Shuffleboard.getTab("Dashboard")
        .add("CENTER SOURCE", false)
        .withWidget("Toggle Button")
        .withSize(3, 3)
        .getEntry();
    rightsource = Shuffleboard.getTab("Dashboard")
        .add("RIGHT SOURCE", false)
        .withWidget("Toggle Button")
        .withSize(3, 3)
        .getEntry();
    _currentSource = new Pose2d();
    _ids = new FieldTagsFilter(_redOrBlue);
    _currentTag = CURRENT_FIELD_LAYOUT.getTags().get(5);

  }

  private void leftSource() {
    left = true;
    center = false;
    right = false;
  }

  private void centerSource() {
    left = false;
    center = true;
    right = false;
  }

  private void rightTsource() {
    left = false;
    center = false;
    right = true;
  }

  public PointWithTime estimationsSupplier() {
    return _currentEstimations;
  }

  public AprilTag getTag() {
    return _currentTag;
  }

  public boolean isRelaventTag() {
    return _ids.isRelavent(_currentTag.ID);
  }

  public boolean isAmp() {
    return _currentTag.ID == 5 || _currentTag.ID == 6;
  }
  public boolean getAlliance(){
    return _redOrBlue;
  }
  public Pose2d getTagPose() {
    return CURRENT_FIELD_LAYOUT.getTagPose(_currentTag.ID ).get().toPose2d();
  }

  public void estimatePosition() {
    if (LimelightHelpers.getTV(null)) {
      if (LimelightHelpers.getBotPose2d_wpiRed(null).getX() != 0 && !_redOrBlue) {
        _currentEstimations = new PointWithTime(LimelightHelpers.getBotPose2d_wpiBlue(null),
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }
      if (LimelightHelpers.getBotPose2d_wpiBlue(null).getX() != 0 && _redOrBlue) {
        _currentEstimations = new PointWithTime(LimelightHelpers.getBotPose2d_wpiBlue(null),
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }
    }
  }
  public double getCalculatedError(Pose2d currentRobotPose){
    double y_error = _currentSource.getY() - currentRobotPose.getY();
    double x_error = _currentSource.getX() - currentRobotPose.getX();
    Rotation2d sinusOfError = Rotation2d.fromDegrees(180).minus(_currentSource.getRotation().plus(Rotation2d.fromDegrees(90)));
    double ratioOfTriangles =  (x_error - y_error/sinusOfError.getTan())/y_error/sinusOfError.getTan();
    return ratioOfTriangles*y_error;
  }
  public Pose2d getSource(){
    return _currentSource;
  }
  public void updateSource() {
    if (left != leftsource.getBoolean(false)) {
      _currentSource = CURRENT_FIELD_LAYOUT.getTagPose(_ids.getLeftSource()).get().toPose2d();

      leftSource();

    } else if (center != centersource.getBoolean(false)) {
      Pose2d leftSource = CURRENT_FIELD_LAYOUT.getTagPose(_ids.getLeftSource()).get().toPose2d();
      Pose2d rightSource = CURRENT_FIELD_LAYOUT.getTagPose(_ids.getRightSource()).get().toPose2d();
      _currentSource = new Pose2d(
          new Translation2d(leftSource.getX() - rightSource.getX(), leftSource.getY() - rightSource.getY()),
          leftSource.getRotation());
      centerSource();
    } else if (right != rightsource.getBoolean(false)) {
      _currentSource = CURRENT_FIELD_LAYOUT.getTagPose(_ids.getRightSource()).get().toPose2d();
      rightTsource();
    }
    leftsource.setBoolean(left);
    centersource.setBoolean(center);
    rightsource.setBoolean(right);
  }

  @Override
  public void periodic() {
    estimatePosition();
    getSource();
    if (LimelightHelpers.getTV(null) && (int) LimelightHelpers.getFiducialID(null) != -1) {
      int proccesed_id = (int) LimelightHelpers.getFiducialID(null);
      _currentTag = CURRENT_FIELD_LAYOUT.getTags().get(proccesed_id-1);
      SmartDashboard.putNumber("current tag", _currentTag.ID);
    }

  }

}
