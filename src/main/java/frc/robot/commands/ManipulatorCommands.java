package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulators.ClawState;
import frc.robot.subsystems.manipulators.ClawSubsystem;

public class ManipulatorCommands {

    private static ClawSubsystem claw = ClawSubsystem.getInstance();

     private static Command setClawState(ClawState state) {
        return new InstantCommand(() -> {
            claw.setState(state);
        }, claw);
    }

    public static Command stopIntake() {
       return setClawState(ClawState.IDLE)
    }

    public static Command holdAlgae() {
        return setClawState(ClawState.HOLDING_ALGAE)
    }

    public static Command runIntake() {
        return setClawState(ClawState.INTAKE)
    }

    public static Command runoutake() {
        return setClawState(ClawState.OUTAKE)
    }

    public static Command beamIntake() {
        return new SequentialCommandGroup(
                runIntake(),
                runClaw(-0.5).until(() -> claw.getBeamBreak()),
                stopIntake());
    }

}

