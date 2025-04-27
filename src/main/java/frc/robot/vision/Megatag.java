package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Megatag {
    private static List<LimelightConfig> limelights = new ArrayList<>();
    private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    public static void addLimelight(LimelightConfig limelight) {
        limelights.add(limelight);
    }

    // Using Megatag 1
    public static void updateOdometry(LimelightConfig limelight) {
        LimelightHelpers.PoseEstimate limelightPoseEstimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(limelight.name());

        if (limelightPoseEstimate == null ||
                !(limelightPoseEstimate.tagCount == 1 && limelightPoseEstimate.rawFiducials.length == 1))
            return;

        if (limelightPoseEstimate.rawFiducials[0].ambiguity > limelight.ambiguity() ||
                limelightPoseEstimate.rawFiducials[0].distToCamera > limelight.distToCamera())
            return;

        swerve.addVisionMeasurement(limelightPoseEstimate.pose, limelightPoseEstimate.timestampSeconds);
    }

    public static void updateAllOdometry() {
        limelights.forEach((limelight)->updateOdometry(limelight));
    }

    public record LimelightConfig(String name, double ambiguity, double distToCamera) {
    }
}
