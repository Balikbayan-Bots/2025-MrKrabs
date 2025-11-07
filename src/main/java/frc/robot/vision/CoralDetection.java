// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class CoralDetection {
    public static final String LIMELIGHT = "limelight-cbotint";
    public static double getCoralDistance() {
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY(LIMELIGHT);
        double limelightMountAngleDegrees = -30.0;
        double limelightLenseHeightMeters = 0.770077;
        double goalHeightMeters = .05715;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        double distanceToGoalMeters = (goalHeightMeters - limelightLenseHeightMeters) / Math.tan(angleToGoalRadians);

        return distanceToGoalMeters;
    }

    public static Pose2d getCoralPose(Pose2d robotPose) {
        return new Pose2d();
        
    }
}