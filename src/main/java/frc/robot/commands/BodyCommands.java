package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CommandRegistry.CommandWrapper;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.manipulators.IntakeSubsytem;
import java.util.LinkedList;

public class BodyCommands {

  private static ElevatorSubsystem elev = ElevatorSubsystem.getInstance();
  private static ArmSubsystem arm = ArmSubsystem.getInstance();
  private static IntakeSubsytem intake = IntakeSubsytem.getInstance();

  public static LinkedList<CommandWrapper> bodyCommandList = new LinkedList<>();

  static {
    bodyCommandList.add(new CommandWrapper("Level Four", positionLevelFour()));
  }

  public static Command armSetpointRun(BodySetpoint setpoint) {
    return new InstantCommand(
        () -> {
          arm.updateSetpoint(setpoint);
        },
        arm);
  }

  public static Command elevSetpointRun(BodySetpoint setpoint) {
    return new InstantCommand(
        () -> {
          elev.updateSetpoint(setpoint);
        },
        elev);
  }

  public static Command intakeSetpointRun(IntakeSetpoint setpoint) {
    return new RunCommand(
        () -> {
          intake.updateSetpoint(setpoint);
        },
        intake);
  }

  public static Command positionHandoff() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.HANDOFF),
        new WaitUntilCommand(() -> elev.isAtSetpoint()),
        armSetpointRun(BodySetpoint.HANDOFF),
        new WaitUntilCommand(() -> arm.isAtSetpoint())
        // elevSetpointRun(BodySetpoint.HANDOFF).until(() -> elev.isAtSetpoint()), //TODO:
        // isAtSetpoint doesnt work
        // armSetpointRun(BodySetpoint.HANDOFF).until(() -> arm.isAtSetpoint())
        );
  }

  public static Command positionLevelTwo() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.SAFE_START).until(elev::isAtSetpoint),
            armSetpointRun(BodySetpoint.SAFE_START).until(arm::isAtSetpoint)),

        // intakeSetpointRun(IntakeSetpoint.LVL_ONE),
        // new SequentialCommandGroup(
        new WaitCommand(.5),
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.CORAL_LEVEL2).until(elev::isAtSetpoint),
            armSetpointRun(BodySetpoint.CORAL_LEVEL2).until(arm::isAtSetpoint),
            intakeSetpointRun(IntakeSetpoint.DEPLOYED).withTimeout(1.0)));
  }

  public static Command positionLevelThree() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.SAFE_START).until(elev::isAtSetpoint),
            armSetpointRun(BodySetpoint.SAFE_START).until(arm::isAtSetpoint)),
        // /new SequentialCommandGroup
        new ParallelCommandGroup(
            new WaitCommand(1.0),
            elevSetpointRun(BodySetpoint.CORAL_LEVEL3).until(elev::isAtSetpoint),
            armSetpointRun(BodySetpoint.CORAL_LEVEL3).until(arm::isAtSetpoint)));
  }

  public static Command positionLevelFour() {
    return // new SequentialCommandGroup(
    new SequentialCommandGroup(
        // elevSetpointRun(BodySetpoint.CORAL_LEVEL4).until(elev::isAtSetpoint),
        new RunCommand(() -> elev.updateSetpoint(BodySetpoint.CORAL_LEVEL4), elev).withTimeout(1.0),
        new RunCommand(() -> arm.updateSetpoint(BodySetpoint.CORAL_LEVEL4), arm)
            .until(arm::isAtSetpoint)
        // new SequentialCommandGroup(
        // new WaitCommand(1.0),
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> arm.isAtSetpoint())
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(arm::isAtSetpoint)
        // elevSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> elev.isAtSetpoint()),
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> arm.isAtSetpoint())
        // )
        );
  }

  public static Command positionHighAlgae() {
    return // new SequentialCommandGroup(
    new ParallelCommandGroup(
        elevSetpointRun(BodySetpoint.ALGAE_LEVEL3),

        // new SequentialCommandGroup(
        // new WaitCommand(1.0),
        armSetpointRun(BodySetpoint.ALGAE_LEVEL3)
        // elevSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> elev.isAtSetpoint()),
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> arm.isAtSetpoint())
        // )
        );
  }

  public static Command positionLowAlgae() {
    return new ParallelCommandGroup(
        elevSetpointRun(BodySetpoint.ALGAE_LEVEL2), armSetpointRun(BodySetpoint.ALGAE_LEVEL2));
  }

  public static Command positionFloorAlgae() {
    return new SequentialCommandGroup(
        intakeSetpointRun(IntakeSetpoint.DEPLOYED).withTimeout(0.5),
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.FLOOR_ALGAE), armSetpointRun(BodySetpoint.FLOOR_ALGAE)));
  }

  public static Command positionNet() {
    return // new SequentialCommandGroup(
    new ParallelCommandGroup(
        elevSetpointRun(BodySetpoint.HIGH_NET),

        // new SequentialCommandGroup(
        // new WaitCommand(1.0),
        armSetpointRun(BodySetpoint.HIGH_NET)
        // elevSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> elev.isAtSetpoint()),
        // armSetpointRun(BodySetpoint.CORAL_LEVEL4).until(() -> arm.isAtSetpoint())
        // )
        );
  }

  public static Command positionStow() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.SAFE_STOW), armSetpointRun(BodySetpoint.SAFE_STOW)),
        new WaitUntilCommand(elev::isAtSetpoint),
        armSetpointRun(BodySetpoint.STOW_POS).until(arm::isAtSetpoint));
  }

  public static Command positionStart() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            elevSetpointRun(BodySetpoint.SAFE_START), armSetpointRun(BodySetpoint.SAFE_START)),
        new WaitUntilCommand(arm::isAtSetpoint),
        new ParallelCommandGroup(
            armSetpointRun(BodySetpoint.START_CONFIG).until(arm::isAtSetpoint),
            elevSetpointRun(BodySetpoint.START_CONFIG).until(arm::isAtSetpoint)));
  }
}
