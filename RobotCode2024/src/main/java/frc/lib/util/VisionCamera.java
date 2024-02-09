package frc.lib.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
    public PhotonCamera _camera ;

    public Transform3d _camera_to_robot = new Transform3d();
    public PhotonPoseEstimator _estimator;
    
    public VisionCamera(String name,Transform3d newTransform3d, AprilTagFieldLayout _layout){
        _camera = new PhotonCamera(name);
        _camera_to_robot = newTransform3d;
        _estimator = new PhotonPoseEstimator(_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, newTransform3d);
    }
}
