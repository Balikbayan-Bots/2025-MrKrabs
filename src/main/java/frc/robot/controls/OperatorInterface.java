package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface {
  private static OperatorInterface instance;

  public static OperatorInterface getInstance() {
    if (instance == null) {
      instance = new OperatorInterface();
    }
    return instance;
  }

  private static final int DRIVER_PORT = 0;
  private static final int CO_DRIVER_PORT = 1;

  private final CommandXboxController driverController;
  private final CommandXboxController coDriverController;

  private OperatorInterface() {
    driverController = new CommandXboxController(DRIVER_PORT);
    coDriverController = new CommandXboxController(CO_DRIVER_PORT);
  }

  public CommandXboxController getDriver() {
    return driverController;
  }

  public CommandXboxController getCoDriver() {
    return coDriverController;
  }
}
