package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static final double FIELD_LENGTH_M = Units.feetToMeters(54.0);
    public static AprilTagFieldLayout BLUE_FIELD_LAYOUT;
    public static AprilTagFieldLayout RED_FIELD_LAYOUT;

    static {
        try {
            BLUE_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            BLUE_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            RED_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            RED_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static AprilTagFieldLayout getFieldLayout() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return BLUE_FIELD_LAYOUT;
        } else {
            return RED_FIELD_LAYOUT;
        }
    }
}