package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BodyCommands;
import frc.robot.commands.CommandRegistry;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.controls.Bindings;
import frc.robot.controls.OperatorInterface;
import frc.robot.subsystems.body.ArmSubsystem;
import frc.robot.subsystems.body.ElevatorSubsystem;
import frc.robot.subsystems.manipulators.ClawSubsystem;
import frc.robot.subsystems.manipulators.IntakeSubsytem;
import frc.robot.subsystems.swerve.SwervePositions;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  // Declare Controls
  public final OperatorInterface oi;

  // Declare Subsystems
  public final SwerveSubsystem swerve;
  public final ClawSubsystem claw;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final IntakeSubsytem intake;
  // Declare Choosers
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize Controls
    oi = OperatorInterface.getInstance();

    // Initialize Subsystems
    swerve = SwerveSubsystem.getInstance();
    arm = ArmSubsystem.getInstance();
    elevator = ElevatorSubsystem.getInstance();
    claw = ClawSubsystem.getInstance();
    intake = IntakeSubsytem.getInstance();
    // Initialize Choosers

    CommandRegistry.registerAllCommands(
        BodyCommands.bodyCommandList.toArray(new CommandRegistry.CommandWrapper[0]));
    NamedCommands.registerCommand("ID 9 Left", SwerveCommands.driveTagNineLeft());
    NamedCommands.registerCommand("ID 10 Left", SwerveCommands.driveTagTenLeft());
    NamedCommands.registerCommand("ID 17 Right", SwerveCommands.driveTagSeventeenRight());
    NamedCommands.registerCommand("ID 21 Left", SwerveCommands.driveTagTwentyOneLeft());
    NamedCommands.registerCommand("ID 22 Left", SwerveCommands.driveTagTwentyTwoLeft());
    NamedCommands.registerCommand("LVL4", BodyCommands.positionLevelFour());
    NamedCommands.registerCommand("score", ManipulatorCommands.autoL4score());
    NamedCommands.registerCommand("upperStow", BodyCommands.positionStow());
    NamedCommands.registerCommand("groundIntake", ManipulatorCommands.groundIntake());
    NamedCommands.registerCommand("stowIntake", ManipulatorCommands.intakeLevelHandoff());
    NamedCommands.registerCommand("grabLowAlgae", BodyCommands.positionLowAlgae());
    NamedCommands.registerCommand("algaeIntake", ManipulatorCommands.beamAlgaeIntake());
    NamedCommands.registerCommand("goNet", BodyCommands.positionNet());
    NamedCommands.registerCommand("scoreNet", ManipulatorCommands.algaeScore());
    NamedCommands.registerCommand("handover", ManipulatorCommands.handover());
    NamedCommands.registerCommand("Left Peg", SwerveCommands.driveToPegProxy(SwervePositions.alignMent.LEFT));
    NamedCommands.registerCommand("Right Peg", SwerveCommands.driveToPegProxy(SwervePositions.alignMent.RIGHT));
    NamedCommands.registerCommand("Middle", SwerveCommands.driveToPegProxy(SwervePositions.alignMent.CENTER));

    autoChooser = AutoBuilder.buildAutoChooser("Tests");

    // NamedCommands.registerCommand("Limelight Source", );
    configureBindings();
    configureDashboard();
  }

  private void configureBindings() {
    Bindings.configureSwerveBinds();
    Bindings.configureClawBinds();
    Bindings.configureBodyBinds();
    Bindings.configureIntakeBinds();
  }

  private void configureDashboard() {
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(claw);
    SmartDashboard.putData(intake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
