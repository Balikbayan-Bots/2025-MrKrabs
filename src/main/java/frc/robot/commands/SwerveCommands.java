package frc.robot.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.controls.Controls;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwervePositions;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.ObjectDetection;

import java.util.ArrayList;
import java.util.Set;

public class SwerveCommands {
  private static final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  private SwerveCommands() {
    throw new IllegalStateException("Utility class");
  }

  // private void registerHashMap(HashMap<Integer, Pose2d> map, String suffix){

  //   //method here
  //   for (Map.Entry<Integer, Pose2d> entry : map.entrySet()) {
  //     NamedCommand.registerCommand(suffix, entry.getValue());
  //   }
  // }

  public static Command manualDrive(double deadband) {
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MAX_TELEOP_SPEED * deadband)
            .withRotationalDeadband(MAX_TELEOP_ROT * deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return swerve.applyRequest(
        () ->
            drive
                .withVelocityX(Controls.Swerve.translateX.get() * getDriveMultiplier())
                .withVelocityY(Controls.Swerve.translateY.get() * getDriveMultiplier())
                .withRotationalRate(Controls.Swerve.rotate.get() * getDriveMultiplier()));
  }

  public static double getDriveMultiplier() {
    double currentPos = elevator.getInches();
    double maxPos = BodySetpoint.HIGH_NET.getElevTravel();
    double ratio = Math.abs(currentPos / maxPos);
    double speedMultiplier = 100D - (33D) * (ratio);
    return speedMultiplier / 100D;
  }

  public static Command driveToPose(Pose2d targetPosition) {
    PathConstraints constraints =
        new PathConstraints(1.5, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return driveToPose(targetPosition, constraints);
  }

  public static Command driveToPose(Pose2d targetPosition, PathConstraints constraints) {
    return AutoBuilder.pathfindToPose(targetPosition, constraints, 0.0);
  }

  public static Command driveToCoral() {

    return driveToPose(ObjectDetection.getCoralPose(swerve.getState().Pose));
  }

  public static Command magicCoral() {

    return Commands.sequence(
        Commands.parallel(
                ManipulatorCommands.groundIntake(),
                    Commands.defer(() -> driveToCoral(), Set.of(swerve))));
  }

  public static Command driveToPegProxy(SwervePositions.alignMent align) {
    return Commands.defer(
        () -> {
          return driveToPeg(align);
        },
        Set.of(swerve));
  }

  public static Command driveTagNineLeft() {
    return driveToPose(new Pose2d(12.683, 5.428, new Rotation2d(Units.degreesToRadians(29.8))))
        .withTimeout(3.0);
  }

  public static Command driveTagTwentyTwoLeft() {
    return driveToPose(new Pose2d(4.83, 2.62, new Rotation2d(Units.degreesToRadians(-147.8))))
        .withTimeout(3.0);
  }

  public static Command driveTagTwentyOneLeft() {
    return driveToPose(new Pose2d(5.92, 3.67, new Rotation2d(Units.degreesToRadians(-90.5))))
        .withTimeout(3.0);
  }

  public static Command driveTagTenLeft() {
    return driveToPose(new Pose2d(11.661, 4.386, new Rotation2d(Units.degreesToRadians(91))))
        .withTimeout(3.0);
  }

  public static Command driveTagSeventeenRight() {
    return driveToPose(new Pose2d(3.76, 2.83, new Rotation2d(Units.degreesToRadians(153.0))))
        .withTimeout(3.0);
  }

  public static Command driveToPeg(SwervePositions.alignMent align) {
    return driveToPose(SwervePositions.getScorePostition(swerve.getCurrentBestTag(), align))
        .withTimeout(3);
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

    return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
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
