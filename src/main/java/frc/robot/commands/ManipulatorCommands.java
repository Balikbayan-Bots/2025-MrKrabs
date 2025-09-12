package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.manipulators.ClawState;
import frc.robot.subsystems.manipulators.ClawSubsystem;

public class ManipulatorCommands {

    private static ClawSubsystem claw = ClawSubsystem.getInstance();

     private static Command runClaw(double speed) {
        return new RunCommand(() -> {
            claw.setSpeed(speed);
        }, claw);
    }

    public static Command stopIntake() {
        return new InstantCommand(() -> {
            claw.setSpeed(0);
        }, claw);
    }

    public static Command beamIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    claw.setState(ClawState.INTAKE);
                }),
                new ParallelCommandGroup(
                        runClaw(0.5)).until(() -> claw.getBeamBreak()),
                new SequentialCommandGroup(
                        new WaitCommand(0.005D),
                        stopIntake()));
    }


}
