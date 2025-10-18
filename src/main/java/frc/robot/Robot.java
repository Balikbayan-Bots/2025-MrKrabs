package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.LimelightConfigs;
import frc.robot.vision.Megatag;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();

    Megatag.addLimelight(LimelightConfigs.ReefLimelight);
    // Megatag.addLimelight(LimelightConfigs.IntLimelight);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Megatag.updateAllOdometry();
  }

  @Override
  public void autonomousInit() {
    autoCommand = robotContainer.getAutonomousCommand();

    if (autoCommand != null) {
      DriverStation.reportWarning("Auto picked" + autoCommand.getName(), true);
      autoCommand.schedule();
    }

    Megatag.updateAllIMU(2);
  }

  @Override
  public void teleopInit() {
    Megatag.updateAllIMU(2);
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void disabledInit() {
    Megatag.updateAllIMU(1);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
