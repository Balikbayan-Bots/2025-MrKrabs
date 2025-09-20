package frc.robot.subsystems.manipulators;

public enum IntakeState {
    
    INTAKE(0.2, 0.5),
    HANDOFF(0.0, -0.5),
    SHOOT(0.0, -0.8);

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
