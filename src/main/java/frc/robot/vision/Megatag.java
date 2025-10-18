package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class Megatag {
  private static List<LimelightConfig> limelights = new ArrayList<>();
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  public static void addLimelight(LimelightConfig limelight) {
    limelights.add(limelight);
  }

  // Using Megatag 1
  // TODO: Get std dev to see confidence
  // TODO: constantly set robot orientation

  public static void updateOdometry(LimelightConfig limelight) {
    // CRITICAL FIX: The rejection flag must be local or reset every loop.
    boolean doRejectUpdate = false;

    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());

    // Exit early if the pose is null (no data received)
    if (mt1 == null) {
      return;
    }

    Matrix<N3, N1> confid = getStdDev(mt1);

    // Initial rejection if no tags are seen or if the raw data is missing
    if (mt1.tagCount == 0 || mt1.rawFiducials.length == 0) {
      doRejectUpdate = true;
    }

    // Apply filtering based on a single tag (required for MegaTag1 with ambiguity check)
    if (!doRejectUpdate) {
      // NOTE: MegaTag1 is generally unreliable with just one tag unless ambiguity is checked.
      // Keeping this single-tag check based on your desired logic.
      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        // FIX: Use configurable values from LimelightConfig instead of hardcoded 0.5 and 3
        if (mt1.rawFiducials[0].ambiguity > limelight.ambiguity()) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > limelight.distToCamera()) {
          doRejectUpdate = true;
        }
      } else {
        // Reject if it's not exactly one tag (to use the ambiguity filter safely)
        doRejectUpdate = true;
      }
    }

    if (!doRejectUpdate) {
      // CRITICAL FIX: Use the correct addVisionMeasurement overload to apply confidence (StdDev)
      swerve.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, confid);
    }
  }

  public static Matrix<N3, N1> getStdDev(LimelightHelpers.PoseEstimate mt1pos) {

    double xyStdev = 2.0;

    double thetaStdev = 2.0;

    if (mt1pos.tagCount >= 2 && mt1pos.avgTagArea > 0.1) {
      xyStdev = 0.5;
      thetaStdev = 0.5;
    } else if (mt1pos.tagCount >= 2 || mt1pos.avgTagArea > 0.1) {
      xyStdev = 0.75;
      thetaStdev = 0.75;
    } else if (mt1pos.avgTagArea > 0.8) {
      xyStdev = 0.8;
      thetaStdev = 0.8;
    }
    Matrix<N3, N1> confidenceStdDev = VecBuilder.fill(xyStdev, xyStdev, thetaStdev);
    return confidenceStdDev;
  }

  /*/ boolean usingMT2 = true;
    LimelightHelpers.PoseEstimate limelightPoseEstimate;
    if (usingMT2) {
      LimelightHelpers.SetRobotOrientation(
          limelight.name(), swerve.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      SmartDashboard.putNumber("heading", swerve.getState().Pose.getRotation().getDegrees());
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
    */

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
