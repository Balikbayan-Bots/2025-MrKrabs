package frc.robot.subsystems.manipulators;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.BodyConstants.Limits;


public class ManipulatorConstants {
    public static final double algae_speed = -0.5;
    public static final int CLAW_MOTOR_ID =  -1;
    public static final int INTAKE_MOTOR_ID = -1; //not sure if its specifically an intake motor
    public static final int BEAM_BREAK_ID = -1;
    public static BodyConstants.Limits kclawLimits = new Limits(85.0, 65.0, 0, 0);
    public static BodyConstants.Limits kFunnelLimits = new Limits(60.0, 45.0, 0, 0);
}