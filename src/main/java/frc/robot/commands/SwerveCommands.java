package frc.robot.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.controls.Controls;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveCommands {
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private SwerveCommands() {
    throw new IllegalStateException("Utility class");
  }

  public static Command manualDrive(double deadband) {
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MAX_TELEOP_SPEED * deadband)
            .withRotationalDeadband(MAX_TELEOP_ROT * deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return swerve.applyRequest(
        () ->
            drive
                .withVelocityX(Controls.Swerve.translateX.get())
                .withVelocityY(Controls.Swerve.translateY.get())
                .withRotationalRate(Controls.Swerve.rotate.get()));
  }

  public static Command driveToPose(Pose2d targetPosition) {
    PathConstraints constraints =
        new PathConstraints(1.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return driveToPose(targetPosition, constraints);
  }

  public static Command driveToPose(Pose2d targetPosition, PathConstraints constraints) {
    Pose2d currentPose = swerve.getState().Pose;
    Trajectory trajectory = generateTrajectory(currentPose, targetPosition);
    var thetaController =
        new ProfiledPIDController(
            0.0, 0, 0, new Constraints(0.25, 120)); //TODO: THESE 12 VALUES ARE JUST PLACEHOLDERS
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    return new SwerveControllerCommand(
    trajectory, 
    () -> getPose(),
    swerve.getKinematics(),

    new PIDController(0, 0, 0),
    new PIDController(0, 0, 0), //TODO: THESE 0.1 VALUES ARE JUST PLACEHOLDERS
    thetaController,
    swerve::setModuleStates, 
    swerve
    );
    // .andThen(stopDrive());
  }

  public static Command driveToRedCRightPeg() {
    return driveToPose(new Pose2d(15, 7, new Rotation2d(0)));
  }

  public static Command reorient() {
    return swerve.runOnce(() -> swerve.seedFieldCentric()); 
  }

  public static Pose2d getPose() {
    return swerve.getState().Pose;
  }

  public static Trajectory generateTrajectory(Pose2d start, Pose2d end) {

    var interiorWaypoints = new ArrayList<Translation2d>();

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));

    return TrajectoryGenerator.generateTrajectory(
        start,
        interiorWaypoints,
        end,
        config);
  }


  public static Command stopDrive() {
    return swerve.applyRequest(
        () ->
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
  }
}
