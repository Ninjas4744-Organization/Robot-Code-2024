// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  public AprilTagFieldLayout BLUE_FIELD_LAYOUT;
  public AprilTagFieldLayout RED_FIELD_LAYOUT;
  public AprilTag _currentTag;
  private boolean _redOrBlue;
  /** Creates a new Vision. */
  public Vision() {
    try {
      BLUE_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      BLUE_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

      RED_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      RED_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

          } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              _redOrBlue = alliance.get() == DriverStation.Alliance.Blue;
          }
          _redOrBlue = false;

  
    _currentTag = false?BLUE_FIELD_LAYOUT.getTags().get(7):RED_FIELD_LAYOUT.getTags().get(7);
  }
  public AprilTag getTag(){
    return _currentTag;
  }
  public Pose2d getTagPose(){
    SmartDashboard.putNumber("REAL YAW TAG",RED_FIELD_LAYOUT.getTagPose(_currentTag.ID).get().toPose2d().getX());
    return RED_FIELD_LAYOUT.getTagPose(_currentTag.ID).get().toPose2d();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("current Tag",_currentTag.ID);
     if (LimelightHelpers.getTV(null) && (int)LimelightHelpers.getFiducialID(null) != -1){
      _currentTag = false?
      BLUE_FIELD_LAYOUT.getTags().get((int)LimelightHelpers.getFiducialID(null)-1):
      RED_FIELD_LAYOUT.getTags().get((int)LimelightHelpers.getFiducialID(null)-1);
      
    }
  
    // This method will be called once per scheduler run
  }
    
}
