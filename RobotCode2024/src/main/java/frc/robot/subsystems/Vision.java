package frc.robot.subsystems;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private FieldTagsFilter _ids;
  
  public Vision() {
    try {
      CURRENT_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        _redOrBlue = !(alliance.get() == DriverStation.Alliance.Blue);
        if (_redOrBlue) {
          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } else {
          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    _ids = new FieldTagsFilter(_redOrBlue);
    _currentTag = CURRENT_FIELD_LAYOUT.getTags().get(_ids.getAmp() - 1);

  }

  public PointWithTime estimationsSupplier() {

    return _currentEstimations;
  }

  public AprilTag getTag() {
    return _currentTag;
  }
  public boolean isRelaventTag(){
    return _ids.isRelavent(_currentTag.ID);
  }
  public boolean isAmp() {
    return _currentTag.ID == 5 || _currentTag.ID == 6;
  }

  public Pose2d getTagPose() {
    return CURRENT_FIELD_LAYOUT.getTagPose(_currentTag.ID-1).get().toPose2d();
  }

  public void estimatePosition() {

    if (LimelightHelpers.getTV(null)) {
      if (LimelightHelpers.getBotPose2d_wpiRed(null).getX() != 0 && !_redOrBlue) {
        _currentEstimations = new PointWithTime(LimelightHelpers.getBotPose2d_wpiRed(null),
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }
      if (LimelightHelpers.getBotPose2d_wpiBlue(null).getX() != 0 && _redOrBlue) {
        _currentEstimations = new PointWithTime(LimelightHelpers.getBotPose2d_wpiBlue(null),
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

      }

    }

  }

  @Override
  public void periodic() {
    estimatePosition();
    if (LimelightHelpers.getTV(null) && (int) LimelightHelpers.getFiducialID(null) != -1) {
      int proccesed_id = (int) LimelightHelpers.getFiducialID(null);
      _currentTag = _ids.isRelavent(proccesed_id) ? CURRENT_FIELD_LAYOUT.getTags().get(proccesed_id) : _currentTag;
      System.out.println(proccesed_id);
    }
    SmartDashboard.putString("current tag", String.valueOf(_currentTag.ID));
  }

  public String tagToString(AprilTag tag) {
    switch(tag.ID){
      case 1:
        return "Blue Source";
      
      case 2:
        return "Blue Source";

      case 5:
        return "Red Amp";

      case 6:
        return "Blue Amp";

      case 9:
        return "Red Source";

      case 10:
        return "Red Source";

      case 11:
        return "Red Stage";

      case 12:
        return "Red Stage";

      case 13:
        return "Red Stage";

      case 14:
        return "Blue Stage";

      case 15:
        return "Blue Stage";

      case 16:
        return "Blue Stage";
    }
    
    return "No Tag In Sight";
  }

  // private boolean isBlueID(int id) {
  // return id == 1 || id == 2 || id == 6 || id == 7 || id == 8 || id == 14 || id
  // == 15 || id == 16;
  // }

  // private void relaventTags(int id) {
  // if (_redOrBlue) {

  // _currentTag = isBlueID(id) ? CURRENT_FIELD_LAYOUT.getTags().get(id) :
  // _currentTag;

  // } else {
  // _currentTag = !isBlueID(id) ? CURRENT_FIELD_LAYOUT.getTags().get(id) :
  // _currentTag;

  // }
  // }

}
