package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;

// The reef positions go clockwise, index 0 is the farthest reef face at the driverstation that you
// are at.

public class SwervePositions {
  public static final HashMap<Integer, Pose2d> ReefLeft = new HashMap<>();
  public static final HashMap<Integer, Pose2d> ReefCenter = new HashMap<>();
  public static final HashMap<Integer, Pose2d> ReefRight = new HashMap<>();
  public static final ArrayList<Integer> validIDS = new ArrayList<>();
  public static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  public static enum alignMent {
    LEFT,
    CENTER,
    RIGHT
  }

  public static Pose2d getScorePostition(int tagNum, alignMent align) {

    if (!validIDS.contains(tagNum)) {
      return swerve.getState().Pose;
    }

    return switch (align) {
      case LEFT -> ReefLeft.get(tagNum);
      case CENTER -> ReefCenter.get(tagNum);
      case RIGHT -> ReefRight.get(tagNum);
      default -> swerve.getState().Pose;
    };
  }

  static {
    validIDS.add(7);
    validIDS.add(8);
    validIDS.add(9);
    validIDS.add(17);
    validIDS.add(18);
    validIDS.add(19);
    validIDS.add(20);
    validIDS.add(21);
    validIDS.add(22);

    ReefLeft.put(17, new Pose2d(3.45, 3.06, new Rotation2d(Units.degreesToRadians(149.5))));
    ReefLeft.put(18, new Pose2d(3.08, 4.41, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefLeft.put(19, new Pose2d(4.115, 5.429, new Rotation2d(Units.degreesToRadians(30))));
    ReefLeft.put(20, new Pose2d(5.48, 5.03, new Rotation2d(Units.degreesToRadians(-28.5))));
    ReefLeft.put(21, new Pose2d(5.88, 3.67, new Rotation2d(Units.degreesToRadians(-90.5))));
    ReefLeft.put(22, new Pose2d(4.885, 2.628, new Rotation2d(Units.degreesToRadians(-150))));

    ReefCenter.put(21, new Pose2d(6.045, 3.840, new Rotation2d(Units.degreesToRadians(-90))));

    ReefRight.put(17, new Pose2d(3.74, 2.89, new Rotation2d(Units.degreesToRadians(147.8))));
    ReefRight.put(18, new Pose2d(3.11, 4.06, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefRight.put(19, new Pose2d(3.822, 5.244, new Rotation2d(Units.degreesToRadians(30))));
    ReefRight.put(20, new Pose2d(5.23, 5.2, new Rotation2d(Units.degreesToRadians(-31.2))));
    ReefRight.put(21, new Pose2d(5.84, 4, new Rotation2d(Units.degreesToRadians(-90.3))));
    ReefRight.put(22, new Pose2d(5.155, 2.784, new Rotation2d(Units.degreesToRadians(-150))));
  }
}
