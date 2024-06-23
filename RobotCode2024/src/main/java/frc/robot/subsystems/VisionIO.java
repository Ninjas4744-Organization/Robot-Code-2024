package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    class VisionIOOutput {
        public Pose2d robotPose;
        public double timestamp;

        public int maxAmbiguityTagId;
        public int farthestTagId;
        public int closestTagId;

        public double maxAmbiguity;
        public double farthestTagDist;
        public double closestTagDist;

        public boolean hasTargets = false;
    }

    default VisionIOOutput Update() {
        return null;
    }
}