package frc.robot.subsystems.manipulators;
import frc.robot.subsystems.body.BodyConstants;
import frc.robot.subsystems.body.BodyConstants.Limits;


public class ManipulatorConstants {
    public static final double algae_speed = -0.5;
    public static final int CLAW_MOTOR_ID =  31;
    public static final int INTAKE_DEPLOY_MOTOR_ID = 40; //not sure if its specifically an intake motor
    public static final int INTAKE_SPIN_MOTOR_1 = 41; 
    public static final int INTAKE_SPIN_MOTOR_2 = 42; 
    public static final int BEAM_BREAK_ID = -1;
    public static final int CLAW_MAX_VOLTAGE_FORWARD = 10;
    public static final int CLAW_MAX_VOLTAGE_REVERSE = -10;
    public static BodyConstants.Limits kclawLimits = new Limits(85.0, 65.0, 0, 0);
    public static BodyConstants.Limits kFunnelLimits = new Limits(60.0, 45.0, 0, 0);
}