package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    // if (index < 0 || index >= positions.length) {
    //   throw new IndexOutOfBoundsException("Index out of bounds for position set");
    // }

    if(!validIDS.contains(tagNum)) {
        return swerve.getState().Pose;
    }

    switch(align) {
      case LEFT:
        return ReefLeft.get(tagNum);
      case CENTER:
        return ReefCenter.get(tagNum);
      case RIGHT:
        return ReefRight.get(tagNum);
      default:
        return swerve.getState().Pose;
    }
  }

  static {
    // Example of setting a position
    // ReefLeft[0] = new Pose2d(1.0, 2.0, new Rotation2d(Units.degreesToRadians(90.0)));
    validIDS.add(17);
    validIDS.add(18);
    validIDS.add(22);
    //TODO: make 12 instaed of 6
    // ReefLeft.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefLeft.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefLeft.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    ReefLeft.put(17,new Pose2d(3.45, 3.06, new Rotation2d(Units.degreesToRadians(149.5))));
    ReefLeft.put(18,new Pose2d(3.08, 4.41, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefLeft.put(22,new Pose2d(4.88, 2.57, new Rotation2d(Units.degreesToRadians(-147.76))));
    
    // ReefCenter.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefCenter.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefCenter.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    ReefCenter.put(17,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    ReefCenter.put(18,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    ReefCenter.put(22,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));

    // ReefRight.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefRight.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    // ReefRight.put(0,new Pose2d(0.0, 0.0, new Rotation2d(Units.degreesToRadians(0.0))));
    ReefRight.put(17,new Pose2d(3.74, 2.89, new Rotation2d(Units.degreesToRadians(147.8))));
    ReefRight.put(18,new Pose2d(3.11, 4.06, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefRight.put(22,new Pose2d(5.16, 2.86, new Rotation2d(Units.degreesToRadians(-148.8))));
  }
}
