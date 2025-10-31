package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.commands.BodyCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Telemetry;

public class Bindings {
  private static final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private Bindings() {
    throw new IllegalStateException("Utility class");
  }

  public static void configureSwerveBinds() {
    swerve.setDefaultCommand(SwerveCommands.manualDrive(0.1));

    Controls.Swerve.reorient.onTrue(SwerveCommands.reorient());

    Telemetry logger = new Telemetry(SwerveConstants.SPEED_AT_12V.in(MetersPerSecond));
    swerve.registerTelemetry(logger::telemeterize);
  }

  public static void configureClawBinds() {
  }

  public static void configureBodyBinds() {
    Controls.Setpoint.stowUp.onTrue(BodyCommands.positionStow());
    Controls.Setpoint.stowLow.onTrue(BodyCommands.positionStart());
    Controls.Setpoint.lvlTwo.onTrue(BodyCommands.positionLevelTwo());
    Controls.Setpoint.lvlThree.onTrue(BodyCommands.positionLevelThree());
    Controls.Setpoint.lvlFour.onTrue(BodyCommands.positionLevelFour());
    Controls.Setpoint.algaeHigh.onTrue(BodyCommands.positionHighAlgae());
    Controls.Setpoint.algaeLow.onTrue(BodyCommands.spook());
    Controls.Setpoint.netPos.onTrue(BodyCommands.spook());
    Controls.Setpoint.algaeFloor.onTrue(BodyCommands.positionFloorAlgae());
  
  }
}
