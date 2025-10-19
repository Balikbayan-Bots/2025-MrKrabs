package frc.robot.subsystems.manipulators;

public enum IntakeState {
  START(0, 0),
  INTAKE(0.1, 0.7),
  HOLD(0.2, 0.15),
  HANDOFF(0.0, -0.7),
  SHOOT(0.0, -0.7);

  public final double centerMotorSpeed;
  public final double rollerMotorSpeed;

  private IntakeState(double centerMotorSpeed, double rollerMotorSpeed) {
    this.centerMotorSpeed = centerMotorSpeed;
    this.rollerMotorSpeed = rollerMotorSpeed;
  }

  public double getCenterMotorSpeed() {
    return this.centerMotorSpeed;
  }

  public double getRollerMotorSpeed() {
    return this.rollerMotorSpeed;
  }
}
