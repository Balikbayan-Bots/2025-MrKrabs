package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.CommandRegistry.CommandWrapper;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

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

  private static void registerPositionMap(HashMap<Integer, Pose2d> map, String suffix){

    //method here
    for (Map.Entry<Integer, Pose2d> entry : map.entrySet()) {
      String commandName = entry.getKey() + "-" + suffix;
      System.out.println("Registering command: " + commandName);
      NamedCommands.registerCommand(commandName, SwerveCommands.driveToPose(entry.getValue()));
    }
  }

  public static void registerPositionMaps() {
    registerPositionMap(ReefLeft, "Left");
    registerPositionMap(ReefCenter, "Center");
    registerPositionMap(ReefRight, "Right");
  }

  // Near is the sides facing the driver station's perspective, far is opposite.
  static {
    /** Red Tags */
    validIDS.add(6); // Near Left
    validIDS.add(7); // Near Center
    validIDS.add(8); // Near Right
    validIDS.add(9); // Far Right
    validIDS.add(10); // Far Center
    validIDS.add(11); // Far Left

    ReefLeft.put(6, new Pose2d(13.450, 2.620, new Rotation2d(Units.degreesToRadians(-150.0))));
    ReefLeft.put(7, new Pose2d(14.479, 3.655, new Rotation2d(Units.degreesToRadians(-90))));
    ReefLeft.put(8, new Pose2d(14.079, 5.058, new Rotation2d(Units.degreesToRadians(-30))));
    ReefLeft.put(9, new Pose2d(12.683, 5.428, new Rotation2d(Units.degreesToRadians(30))));
    ReefLeft.put(10, new Pose2d(11.661, 4.386, new Rotation2d(Units.degreesToRadians(90))));
    ReefLeft.put(11, new Pose2d(12.048, 2.982, new Rotation2d(Units.degreesToRadians(150))));

    ReefCenter.put(6, new Pose2d(13.640, 2.611, new Rotation2d(Units.degreesToRadians(-150))));
    ReefCenter.put(7, new Pose2d(14.567, 3.830, new Rotation2d(Units.degreesToRadians(-90))));
    ReefCenter.put(8, new Pose2d(14, 5.224, new Rotation2d(Units.degreesToRadians(-30))));
    ReefCenter.put(9, new Pose2d(12.479, 5.428, new Rotation2d(Units.degreesToRadians(30))));
    ReefCenter.put(10, new Pose2d(11.554, 4.210, new Rotation2d(Units.degreesToRadians(90))));
    ReefCenter.put(11, new Pose2d(12.156, 2.778, new Rotation2d(Units.degreesToRadians(150))));

    ReefRight.put(6, new Pose2d(13.720, 2.780, new Rotation2d(Units.degreesToRadians(-150.0))));
    ReefRight.put(7, new Pose2d(14.479, 4.005, new Rotation2d(Units.degreesToRadians(-90))));
    ReefRight.put(8, new Pose2d(13.796, 5.215, new Rotation2d(Units.degreesToRadians(-30))));
    ReefRight.put(9, new Pose2d(12.383, 5.248, new Rotation2d(Units.degreesToRadians(30))));
    ReefRight.put(10, new Pose2d(11.640, 4.061, new Rotation2d(Units.degreesToRadians(90))));
    ReefRight.put(11, new Pose2d(12.323, 2.838, new Rotation2d(Units.degreesToRadians(150))));

    /** Blue Tags */
    validIDS.add(17); // Near Right
    validIDS.add(18); // Near Center
    validIDS.add(19); // Near Left
    validIDS.add(20); // Far Left
    validIDS.add(21); // Far Center
    validIDS.add(22); // Far Right

    ReefLeft.put(17, new Pose2d(3.45, 3.06, new Rotation2d(Units.degreesToRadians(149.5))));
    ReefLeft.put(18, new Pose2d(3.08, 4.41, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefLeft.put(19, new Pose2d(4.115, 5.429, new Rotation2d(Units.degreesToRadians(30))));
    ReefLeft.put(20, new Pose2d(5.48, 5.03, new Rotation2d(Units.degreesToRadians(-28.5))));
    ReefLeft.put(21, new Pose2d(5.92, 3.67, new Rotation2d(Units.degreesToRadians(-90.5))));
    ReefLeft.put(22, new Pose2d(4.885, 2.628, new Rotation2d(Units.degreesToRadians(-150))));

    ReefCenter.put(17, new Pose2d(3.548, 2.778, new Rotation2d(Units.degreesToRadians(150))));
    ReefCenter.put(18, new Pose2d(2.949, 4.217, new Rotation2d(Units.degreesToRadians(90))));
    ReefCenter.put(19, new Pose2d(3.932,5.452, new Rotation2d(Units.degreesToRadians(30))));
    ReefCenter.put(20, new Pose2d(5.430, 5.248, new Rotation2d(Units.degreesToRadians(-30))));
    ReefCenter.put(21, new Pose2d(6.045, 3.840, new Rotation2d(Units.degreesToRadians(-90))));
    ReefCenter.put(22, new Pose2d(5.070, 2.601, new Rotation2d(Units.degreesToRadians(-150.0))));

    ReefRight.put(17, new Pose2d(3.74, 2.89, new Rotation2d(Units.degreesToRadians(147.8))));
    ReefRight.put(18, new Pose2d(3.11, 4.06, new Rotation2d(Units.degreesToRadians(90.0))));
    ReefRight.put(19, new Pose2d(3.822, 5.244, new Rotation2d(Units.degreesToRadians(30))));
    ReefRight.put(20, new Pose2d(5.23, 5.2, new Rotation2d(Units.degreesToRadians(-31.2))));
    ReefRight.put(21, new Pose2d(5.84, 4, new Rotation2d(Units.degreesToRadians(-90.3))));
    ReefRight.put(22, new Pose2d(5.155, 2.784, new Rotation2d(Units.degreesToRadians(-150))));
  }
}
