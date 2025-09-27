package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.ClawState;
import frc.robot.subsystems.manipulators.ClawSubsystem;
import frc.robot.subsystems.manipulators.IntakeSetpoint;
import frc.robot.subsystems.manipulators.IntakeState;
import frc.robot.subsystems.manipulators.IntakeSubsytem;

public class ManipulatorCommands {

  private static ClawSubsystem claw = ClawSubsystem.getInstance();
  private static IntakeSubsytem intake = IntakeSubsytem.getInstance();
  private static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private static ArmSubsystem arm = ArmSubsystem.getInstance();

  private static Command setClawState(ClawState state) {
    return new 
    ParallelCommandGroup(
    new RunCommand(
        () -> {
          claw.setState(state);
        },
        claw),

        new PrintCommand("YOU SET THE STATE TO" + state)
        );
  }

  private static Command setIntakeState(IntakeState state) {
    return new InstantCommand(
        () -> {
          intake.setState(state);
        },
        intake);
  }

  public static Command intakeSetpointRun(IntakeSetpoint setpoint) {
    return new RunCommand(
        () -> {
          intake.updateSetpoint(setpoint);
        },
        intake);
  }

  public static Command stopIntake() {
    return setClawState(ClawState.IDLE);
  }

  public static Command holdAlgae() {
    return setClawState(ClawState.HOLDING_ALGAE);
  }

  public static Command runIntake() {
    return new RunCommand(()-> claw.setState(ClawState.INTAKE),claw);
  }

  public static Command runOutake() {
    return setClawState(ClawState.OUTAKE);
  }

  public static Command beamIntake() {
        return new RunCommand(
          ()-> claw.setState(ClawState.INTAKE), claw)
          .until(()-> claw.getBeamBreak());
  }

  public static Command score() {
    return setClawState(ClawState.SCORE);
  }

    public static Command intakeLevelOne() {

    return intakeSetpointRun(IntakeSetpoint.LVL_ONE);

  }

    public static Command intakeLevelHandoff() {
    return new SequentialCommandGroup(intakeSetpointRun(IntakeSetpoint.STOWED_HANDOFF), setIntakeState(IntakeState.HOLD));
  }


  public static Command groundIntake() {
    return new SequentialCommandGroup(
        intakeSetpointRun(IntakeSetpoint.DEPLOYED).withTimeout(0.25), 
        new RunCommand(()-> intake.setState(IntakeState.INTAKE), intake)
        );
  }

  public static Command handoff() {
    return new ParallelCommandGroup(
    setIntakeState(IntakeState.HANDOFF), 
    BodyCommands.positionHandoff()
    
    );
  }

  public static Command handover() {
    return new SequentialCommandGroup(
   
        
    BodyCommands.positionHandoff(), 

    //new WaitCommand(4.0), // TODO: BANDIAD FIX FOR ISATSETPOINT
    
    new ParallelDeadlineGroup(
        beamIntake(),
        setIntakeState(IntakeState.HANDOFF)
    ),
    

    stopIntake(),
    setIntakeState(IntakeState.START)
    );  
  }


  
}
