// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.manipulators.ManipulatorConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class ObjectDetection {
  public static final String LIMELIGHT = "limelight-cbotint";

  public static void switchToCoralMode() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT, 0);
  }

  public static void switchToAlgaeMode() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT, 1);
  }

  public static double getCoralDistance() {
    double targetOffsetAngle_Vertical = LimelightHelpers.getTY(LIMELIGHT);
    double limelightMountAngleDegrees = -30.0;
    double limelightLenseHeightMeters = 0.770077;
    double goalHeightMeters = .05715;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distanceToGoalMeters =
        (goalHeightMeters - limelightLenseHeightMeters) / Math.tan(angleToGoalRadians);

    return distanceToGoalMeters * ManipulatorConstants.DRIVE_THRU_MULTIPLIER;
  }

  public static Pose2d getCoralPose(Pose2d robotPose) {
    switchToCoralMode();
    double innerAngle = robotPose.getRotation().getRadians() + Math.PI;
    double dy = Math.sin(innerAngle) * getCoralDistance();
    double dx = Math.cos(innerAngle) * getCoralDistance();

    return new Pose2d(robotPose.getX() + dx, robotPose.getY() + dy, robotPose.getRotation());
  }

  public static boolean validTarget() {
    return LimelightHelpers.getTV(LIMELIGHT);
  }

  public static Pose2d squareUpPose(Pose2d robotPose) {

    double tx = LimelightHelpers.getTX(LIMELIGHT);

    return new Pose2d(
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().plus(Rotation2d.fromDegrees(tx)));
  }

  public static double aimAtCoral() {
    double kP = .035;

    double targetingAngularVelocity = LimelightHelpers.getTX(LIMELIGHT) * kP;

    targetingAngularVelocity *= SwerveConstants.MAX_TELEOP_ROT;

    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
}
