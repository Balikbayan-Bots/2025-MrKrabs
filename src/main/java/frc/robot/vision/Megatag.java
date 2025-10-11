package frc.robot.vision;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Megatag {
  private static List<LimelightConfig> limelights = new ArrayList<>();
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  public static void addLimelight(LimelightConfig limelight) {
    limelights.add(limelight);
  }

  // Using Megatag 1
  // TODO: Get std dev to see confidence 
  //TODO: constantly set robot orientation

  public static void updateOdometry(LimelightConfig limelight) {
    boolean usingMT2 = true;
    LimelightHelpers.PoseEstimate limelightPoseEstimate;
    if (usingMT2) {
      LimelightHelpers.SetRobotOrientation(
          limelight.name(), swerve.getState().Pose.getRotation().getDegrees() * -1, 0, 0, 0, 0, 0);
          SmartDashboard.putNumber("heading", swerve.getState().Pose.getRotation().getDegrees() * -1);
      limelightPoseEstimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
    } else {
      limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());
    }

    if (usingMT2
        && (!(Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720)
            || limelightPoseEstimate.tagCount != 0)) {
      return;
    }

    if (limelightPoseEstimate == null
        || !(limelightPoseEstimate.tagCount == 1 && limelightPoseEstimate.rawFiducials.length == 1))
      return;

    if (limelightPoseEstimate.rawFiducials[0].ambiguity > limelight.ambiguity()
        || limelightPoseEstimate.rawFiducials[0].distToCamera > limelight.distToCamera()) return;

    swerve.addVisionMeasurement(limelightPoseEstimate.pose, limelightPoseEstimate.timestampSeconds);
  }

  public static void updateAllOdometry() {
    limelights.forEach((limelight) -> updateOdometry(limelight));
  }

  public static void updateIMU(LimelightConfig limelight, int mode) {
    LimelightHelpers.SetIMUMode(limelight.name(), mode);
  }

  public static void updateAllIMU(int mode) {
    limelights.forEach((limelight) -> updateIMU(limelight, mode));
  }

  public record LimelightConfig(String name, double ambiguity, double distToCamera) {}
}
