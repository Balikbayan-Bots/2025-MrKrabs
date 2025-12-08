// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ObjectDetection {

  public record LimelightConfig(String name, Rotation3d rotation, Translation3d position) {}
  ;

  private static final LimelightConfig limelightIntake =
      new LimelightConfig(
          "limelight-cbotint",
          new Rotation3d(-0.0869174, 0.523599, 3.01941961),
          new Translation3d(0.0838454, -0.2179066, 0.770077));

  static final double CORAL_CENTER_HEIGHT = 0.05715;

  public static void switchToCoralMode() {
    LimelightHelpers.setPipelineIndex(limelightIntake.name(), 0);
  }

  public static void switchToAlgaeMode() {
    LimelightHelpers.setPipelineIndex(limelightIntake.name(), 1);
  }

  public static double robotCentricTX() {
    double tx = LimelightHelpers.getTX(limelightIntake.name());
    double cameraYaw = limelightIntake.rotation().getZ();
    return Math.toDegrees(cameraYaw) - tx - 180;
  }

  public static boolean validTarget() {
    return LimelightHelpers.getTV(limelightIntake.name());
  }

  public static Pose2d getCoralPose(Pose2d robotPose) {

    switchToCoralMode();

    Pose3d cameraFieldPose = getCameraFieldPose(robotPose, limelightIntake);

    Translation2d coralFieldTranslation =
        castRayAndFindPlaneIntersection(cameraFieldPose, limelightIntake, CORAL_CENTER_HEIGHT);

    if (coralFieldTranslation != null) {
      return new Pose2d(coralFieldTranslation, robotPose.getRotation());
    } else {
      return new Pose2d(0, 0, new Rotation2d());
    }
  }

  public static Translation2d castRayAndFindPlaneIntersection(
      Pose3d cameraFieldPose, LimelightConfig limelight, double planeHeightZ) {
    double tx = LimelightHelpers.getTX(limelight.name());

    double ty = LimelightHelpers.getTY(limelight.name());

    double txRad = Math.toRadians(tx);

    double tyRad = Math.toRadians(ty);

    // Create the Ray Vector in Camera Space

    Translation3d rayVectorCam = new Translation3d(1.0, -Math.tan(txRad), Math.tan(tyRad));

    // Rotate Ray Vector to Field Space (Requires Camera Field Rotation)

    Translation3d rayVectorField = rayVectorCam.rotateBy(cameraFieldPose.getRotation());

    if (Math.abs(rayVectorField.getZ()) < 1e-9) {
      return null; // Ray is parallel to the ground
    }

    Translation3d cameraTranslationField = cameraFieldPose.getTranslation();

    // Calculate intersection scalar t

    double t = (planeHeightZ - cameraTranslationField.getZ()) / rayVectorField.getZ();

    if (t < 0) {
      return null; // Intersection is behind the camera
    }

    double intersectionX = cameraTranslationField.getX() + t * rayVectorField.getX();

    double intersectionY = cameraTranslationField.getY() + t * rayVectorField.getY();

    return new Translation2d(intersectionX, intersectionY);
  }

  public static Pose3d getCameraFieldPose(Pose2d robotPose, LimelightConfig limelight) {
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d robotToCameraTransform =
        new Transform3d(limelight.position(), limelight.rotation());

    Pose3d cameraFieldPose = robotPose3d.transformBy(robotToCameraTransform);

    return cameraFieldPose;
  }

  public static Pose2d getCameraFieldPosition(Pose2d robotPose) {
    Pose3d cameraFieldPose = getCameraFieldPose(robotPose, limelightIntake);

    return new Pose2d(
        cameraFieldPose.getX(),
        cameraFieldPose.getY(),
        Rotation2d.fromRadians(cameraFieldPose.getRotation().getZ()));
  }

  public static double findDistance(Pose2d a, Pose2d b) {
    return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
  }

  public static double getCoralHeading() {

    double[] array = LimelightHelpers.getT2DArray(limelightIntake.name());

    double longSidePixels = array[12];
    double shortSidePixels = array[13];

    return shortSidePixels / longSidePixels;
  }
}
