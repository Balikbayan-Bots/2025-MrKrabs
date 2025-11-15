// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.manipulators.ManipulatorConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class ObjectDetection {

  public static void switchToCoralMode() {
    LimelightHelpers.setPipelineIndex(limelight.name(), 0);
  }

  public static void switchToAlgaeMode() {
    LimelightHelpers.setPipelineIndex(limelight.name(), 1);
  }

  public static double getCoralDistance() {
    double targetOffsetAngle_Vertical = LimelightHelpers.getTY(limelight.name());
    double limelightMountAngleDegrees = -30.0;
    double goalHeightMeters = .05715;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distanceToGoalMeters =
        (goalHeightMeters - limelight.position().getY()) / Math.tan(angleToGoalRadians);

    return distanceToGoalMeters * ManipulatorConstants.DRIVE_THRU_MULTIPLIER;
  }

  public static Pose2d getCoralPose(Pose2d robotPose) {
    switchToCoralMode();
    double innerAngle = robotPose.getRotation().getRadians() + Math.PI;
    double dx = Math.cos(innerAngle) * getCoralDistance();
    double dy = Math.sin(innerAngle) * getCoralDistance();

    Transform2d coralTranslation = new Transform2d(dx, dy, robotPose.getRotation());

    return robotPose.plus(coralTranslation);
  }

  public static boolean validTarget() {
    return LimelightHelpers.getTV(limelight.name());
  }

  public static Pose2d squareUpPose(Pose2d robotPose) {

    double tx = LimelightHelpers.getTX(limelight.name());

    return new Pose2d(
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().plus(Rotation2d.fromDegrees(tx)));
  }

  public static double aimAtCoral() {
    double kP = .035;

    double targetingAngularVelocity = LimelightHelpers.getTX(limelight.name()) * kP;

    targetingAngularVelocity *= SwerveConstants.MAX_TELEOP_ROT;

    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public record LimelightConfig(String name, Rotation3d rotation, Translation3d position) {};

  private static final LimelightConfig limelight = new LimelightConfig("limelight-cbotint", new Rotation3d(4.98,-30,173), new Translation3d(0.0838454,0.2179066,0.770077));

}
