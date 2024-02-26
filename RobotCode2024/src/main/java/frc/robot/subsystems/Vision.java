// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.VisionCamera;
import frc.lib.util.FieldTagsFilter;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout CURRENT_FIELD_LAYOUT;
  private AprilTag _currentTag;

  private List<VisionCamera> _cameras = new ArrayList<VisionCamera>();

  private boolean _redOrBlue;
  private List<Optional<EstimatedRobotPose>> _currentEstimations;
  private FieldTagsFilter _ids;

  /** Creates a new Vision. */
  public Vision() {
    try {
      CURRENT_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        _redOrBlue = alliance.get() == DriverStation.Alliance.Blue;
        if (_redOrBlue) {
          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } else {
          CURRENT_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    _ids = new FieldTagsFilter(_redOrBlue);
    _currentTag = CURRENT_FIELD_LAYOUT.getTags().get(_ids.getAmp());

    _cameras.add(new VisionCamera(Constants.Cameras.Front._name,
        Constants.Cameras.Front._camera_to_robot,
        CURRENT_FIELD_LAYOUT));

    _currentEstimations = getEstimatedGlobalPose();

  }

  public List<Optional<EstimatedRobotPose>> estimationsSupplier() {

    return _currentEstimations;
  }

  public AprilTag getTag() {
    return _currentTag;
  }

  public boolean isAmp() {
    return _currentTag.ID == 5 || _currentTag.ID == 6;
  }

  public Pose2d getTagPose() {
    return CURRENT_FIELD_LAYOUT.getTagPose(_currentTag.ID-1).get().toPose2d();
  }

  public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPose() {

    List<Optional<EstimatedRobotPose>> _res = new ArrayList<Optional<EstimatedRobotPose>>();
    for (VisionCamera cam : _cameras) {
      var res = cam._estimator.update(cam._camera.getLatestResult());
      if (res.isPresent()) {
        for (PhotonTrackedTarget optional : res.get().targetsUsed) {
          _currentTag = _ids.isRelavent(optional.getFiducialId())
              ? CURRENT_FIELD_LAYOUT.getTags().get(optional.getFiducialId())
              : _currentTag;
        }

      }

      _res.add(res);

    }
    return _res;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current tag", _currentTag.ID);
    _currentEstimations = getEstimatedGlobalPose();

  }

}
