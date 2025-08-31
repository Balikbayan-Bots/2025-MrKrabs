package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Telemetry;

public class Bindings {
    private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    private Bindings() {
        throw new IllegalStateException("Utility class");
    }

    public static void configureSwerveBinds() {
        swerve.setDefaultCommand(
                SwerveCommands.manualDrive(0.1));

        Controls.Swerve.reorient
                .onTrue(SwerveCommands.reorient());
            
        // Controls.Swerve.test
        //     .onTrue(SwerveCommands.driveToPose(new Pose2d(16.25, 6.85, Rotation2d.fromDegrees(142.286))));

        Telemetry logger = new Telemetry(SwerveConstants.SPEED_AT_12V.in(MetersPerSecond));
        swerve.registerTelemetry(logger::telemeterize);
    }
}
