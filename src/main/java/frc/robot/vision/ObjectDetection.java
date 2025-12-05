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

  public record LimelightConfig(String name, Rotation3d rotation, Translation3d position) {};

  private static final LimelightConfig limelightIntake = new LimelightConfig("limelight-cbotint", new Rotation3d(-0.0869174,0.523599,3.01941961), new Translation3d(0.0838454,-0.2179066,0.770077));


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

  // 3D Wizardry
  public static Pose2d getCoralPose(Pose2d robotPose) {
    switchToCoralMode();

    Translation2d coralFieldLocation = castRayAndFindPlaneIntersection(robotPose, LimelightHelpers.getTX(limelightIntake.name()), LimelightHelpers.getTY(limelightIntake.name()), 0.05715);


    if(coralFieldLocation != null){
    return new Pose2d(coralFieldLocation, Rotation2d.fromDegrees(robotPose.getRotation().getDegrees() + robotCentricTX()));
    } else {
      return new Pose2d(0,0,new Rotation2d());
    }

  }

  final double CORAL_CENTER_HEIGHT  = 0.05715;

  public static boolean validTarget() {
    return LimelightHelpers.getTV(limelightIntake.name());
  }

  public static Translation2d castRayAndFindPlaneIntersection(Pose2d robotPose, double tx, double ty, double groundPlaneZ){
    Pose3d robotPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0,0, robotPose.getRotation().getRadians()));

    Transform3d robotToCameraTransform = new Transform3d(limelightIntake.position(), limelightIntake.rotation());

    Pose3d cameraFieldPose = robotPose3d.transformBy(robotToCameraTransform);

    Translation3d cameraTranslationField = cameraFieldPose.getTranslation();

    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);

    Translation3d rayVectorCam = new Translation3d(1.0, -Math.tan(txRad), Math.tan(tyRad));
    Translation3d rayVectorField = rayVectorCam.rotateBy(cameraFieldPose.getRotation());

    if(Math.abs(rayVectorField.getZ()) < 1e-9){

      System.out.println("Ray vector is parallel to the ground plane, no intersection found.");

      return null;
    }

    double t = (groundPlaneZ - cameraTranslationField.getZ()) / rayVectorField.getZ();

    if(t < 0){
      System.out.println("Intersection is behind the camera, no valid intersection.");
      System.out.println(t);

      return null;
    }

    double intersectionX = cameraTranslationField.getX() + t * rayVectorField.getX();
    double intersectionY = cameraTranslationField.getY() + t * rayVectorField.getY();

    return new Translation2d(intersectionX, intersectionY);
  }

  public static Pose3d getCameraFieldPose(Pose2d robotPose, LimelightConfig config) {
    Pose3d robotPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d robotToCameraTransform = new Transform3d(config.position(), config.rotation());

    Pose3d cameraFieldPose = robotPose3d.transformBy(robotToCameraTransform);
    
    return cameraFieldPose;
  }

  public static Pose2d getCameraFieldPosition(Pose2d robotPose) {
    Pose3d robotPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d robotToCameraTransform = new Transform3d(limelightIntake.position(), limelightIntake.rotation());

    Pose3d cameraFieldPose = robotPose3d.transformBy(robotToCameraTransform);

    Translation3d cameraTranslationField = cameraFieldPose.getTranslation();

    return new Pose2d(cameraTranslationField.getX(), cameraTranslationField.getY(), Rotation2d.fromRadians(cameraFieldPose.getRotation().getZ()));

  }

  public static double findDistance(Pose2d a, Pose2d b) {
    return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
  }

  public static double getCoralHeading() {
    
    double[] array = LimelightHelpers.getT2DArray(limelightIntake.name());

    double longSidePixels = array[12];
    double shortSidePixels = array[13];

    System.out.println("Long side: " + longSidePixels);
    System.out.println("Short side " + shortSidePixels);
    System.out.println("Ratio: " + (shortSidePixels/longSidePixels));

    return shortSidePixels/longSidePixels;
    
    
  }

}
