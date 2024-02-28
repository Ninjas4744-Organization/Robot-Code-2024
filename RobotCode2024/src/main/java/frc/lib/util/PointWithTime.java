package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class PointWithTime {
    Pose2d point;
    double time;

    public PointWithTime(Pose2d newPose2d, double newTime){
        point = newPose2d;
        time = newTime;
    }

    public Pose2d getPoint() {
        return point;
    }
    public double getTime() {
        return time;
    }
}
