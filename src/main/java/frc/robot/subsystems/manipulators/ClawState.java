package frc.robot.subsystems.manipulators;

public enum ClawState {
  INTAKE(-0.8),
  OUTAKE(0.6),
  HOLDING_ALGAE(-0.25),
  IDLE(0.0),
  SCORE(0.45),
  ALGAE_SCORE(1.0);

  public final double speed;

  private ClawState(double speed) {
    this.speed = speed;
  }

  public double getSpeed() {
    return this.speed;
  }
}
