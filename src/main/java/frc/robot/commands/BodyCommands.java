package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.BodySetpoint;
import frc.robot.subsystems.body.ElevatorSubsystem;

public class BodyCommands {

  private static ElevatorSubsystem elev = ElevatorSubsystem.getInstance();
  private static ArmSubsystem arm = ArmSubsystem.getInstance();

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

  public static Command positionLevelOne() {
    return new ParallelCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL1).withTimeout(0.15),
        armSetpointRun(BodySetpoint.CORAL_LEVEL1).withTimeout(0.15));
  }

  public static Command positionLevelTwo() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL2).withTimeout(0.5),
        armSetpointRun(BodySetpoint.CORAL_LEVEL2).withTimeout(0.5));
  }

  public static Command positionLevelThree() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL3).withTimeout(0.85),
        armSetpointRun(BodySetpoint.CORAL_LEVEL3).withTimeout(0.85));
  }

  public static Command positionLevelFour() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.CORAL_LEVEL4).withTimeout(0.75),
        armSetpointRun(BodySetpoint.CORAL_LEVEL4).withTimeout(0.75));
  }

  public static Command positionStow() {
    return new SequentialCommandGroup(
        elevSetpointRun(BodySetpoint.STOW_INTAKE).withTimeout(0.25),
        armSetpointRun(BodySetpoint.STOW_INTAKE).withTimeout(0.25));
  }
}
